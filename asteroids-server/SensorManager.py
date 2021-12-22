from typing import Iterable, List, Tuple
import cv2
import cv2.aruco as aruco
import numpy as np
#import os
import yaml
import time


class SensorManager:
    # Size of the AruCo markers in mm
    THINGS_TAG_SIZE = 40
    WORLD_TAG_SIZE = 60
    BOTS_TAG_SIZE = 60
    def __init__(self, int_coeff_path: str, fasttags: List[int], worldtagids: List[int], slowtags: List[int], target_fps: float, camnum=1):
        """
        Create the SensorManager object

        Args:
            int_coeff_path (str): path to the internal calibration matrix file
            fasttags (List[int]): id of fast moving tags, including tags on the robots
            worldtagids (List[int]): ids of the three tags defining the corners of the table/workspace 
            slowtags (List[int]): ids of slower tags. Can be safely ignored by passing in an empty array
            target_fps (float): target tracking frame per second. Would not exceed this frame rate.
            camnum (int, optional): numeric id of the camera being used. You need to experiment from 0 to find out the id of the camera that you want to use. Defaults to 1.
        """
        self._camnum = camnum
        self._fast_things_tags = fasttags
        self._world_tags = worldtagids
        self._slow_things_tags = slowtags
        self._moving_things_tags = fasttags + slowtags
        self._IMG_W = 1280
        self._IMG_H = 720
        self._LOCATE_WORLD_NUM = 10
        self._world_located = False
        #self._locate_world_counter = 0
        self._locate_world_points = []
        self._cw_R = None
        self._cw_T = None
        self._wc_R = None
        self._wc_T = None

        self._aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self._aruco_param = aruco.DetectorParameters_create()

        # with open('c922-720p-calib.yaml') as fr:
        with open(int_coeff_path) as fr:
            c = yaml.load(fr)
            self._cam_mat = np.array(c['camera_matrix'])
            self._distortion = np.array(c['dist_coefs'])
            #newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_mat, distortion, (img_w, img_h), 1, (img_w, img_h))


        self._cap = cv2.VideoCapture(camnum, cv2.CAP_MSMF)
        #self._cap = cv2.VideoCapture()
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        # self._cap.set(cv2.CAP_PROP_FPS, 60.0) # Use higher FPS to reduce motion blur
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._IMG_W)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._IMG_H)
        # self._cap.set(cv2.CAP_PROP_BACKEND, cv2.CAP_MSMF)
        ## NOTE: it seems do this would reset the VideoCapture object properties
        #self._cap.open(camnum, cv2.CAP_MSMF)
        self._fps_frame_count = 0
        self._prev_fps_ts = -1
        ## Do not process too many frames despitve high fps
        self._target_interval = 1.0 / target_fps
        self._last_processed_num = -1
        self._inception_ts = time.time()


    def __FindArucoMarkers(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self._aruco_dict, parameters = self._aruco_param)
        #print(f"Marker ids are {ids}")
        ids = [ii[0] for ii in ids] if ids is not None else []
        # rv, tv, _o = aruco.estimatePoseSingleMarkers(corners, markerSide / 1000, camMat, dist)
        # rm = [cv2.Rodrigues(rvec)[0] for rvec in rv] # convert to rotation matrix
        # res = zip(ids, rm, tv) # (tagid, rotation_mat, translation_vec)
        #print(list(res))
        #print(ids)
        return corners, ids

    def __FindMarkerPoses(self, corners, ids, ids_to_find, camMat, distParams, markerSize) -> Iterable[Tuple]:
        selected_ids = [a for a in ids if a in ids_to_find]
        selected = [corners[i] for i in range(0, len(corners)) if ids[i] in ids_to_find]
        if len(selected_ids) > 0:
            rv, tv, _o = aruco.estimatePoseSingleMarkers(selected, markerSize / 1000, camMat, distParams)
            rm = [cv2.Rodrigues(rvec)[0] for rvec in rv] # convert to rotation matrix
            res = zip(selected_ids, rm, tv) # (tagid, rotation_mat, translation_vec)
            #print(list(res))
            #print(ids)
            return res
        else:
            return iter(())


    def __LocateWorld(self, anchor0, anchor1, anchor2):
        ## Origin is at anchor 1
        ## anchor 1 - anchor 2 set the x-axis, anchor 0 does not have to be on the y axis
        #   ------anchor 0-----
        #   |                 |
        #   y                 |
        #   |                 |
        # anchor 1 ---x--- anchor 2
        #
        print(f"Anchor 0, 1, 2 are {anchor0}, {anchor1}, {anchor2}")
        origin_cam_3d = anchor1
        x_cam_3d = anchor2 - anchor1
        x_cam_3d = x_cam_3d / np.linalg.norm(x_cam_3d)
        fake_y_cam_3d = anchor0 - anchor1
        fake_y_cam_3d = fake_y_cam_3d / np.linalg.norm(fake_y_cam_3d)
        z_cam_3d = np.cross(x_cam_3d, fake_y_cam_3d)
        y_cam_3d = np.cross(z_cam_3d, x_cam_3d)
        wc_R = np.hstack((np.reshape(x_cam_3d, (3, 1)), np.reshape(y_cam_3d, (3,1)), np.reshape(z_cam_3d, (3, 1))))
        wc_T = np.reshape(origin_cam_3d, (3, 1))
        cw_R = np.linalg.inv(wc_R)
        cw_T = -cw_R @ wc_T
        return wc_R, wc_T, cw_R, cw_T


    def __fps(self, ts: float) -> float:
        if self._prev_fps_ts > 0:
            self._fps_frame_count += 1
            if self._fps_frame_count == 30:
                self._fps_frame_count = 0
                fps = 30 / (ts - self._prev_fps_ts)
                self._prev_fps_ts = ts
                return fps
        else:
            self._prev_fps_ts = time.time()
            self._fps_frame_count += 1
        return -1

    ## Return True if camera processing is successful and a list of robot poses in the world frame; return an empty list if 
    ## The world is not located yet 
    def GetSensorReading(self, save_results=None):
        
        success, img = self._cap.read()
        if not success:
            print("Error: cannot read video frame")
            return 2, []
        else:
            curr_ts = time.time()
            # last_proc_ts = self._last_processed_ts
            to_process_num = int((curr_ts - self._inception_ts) / self._target_interval)
            ## Only do the calculation at the target FPS
            if self._last_processed_num > 0:
                if to_process_num <= self._last_processed_num:
                    return 1, []
                else:
                    self._last_processed_num = to_process_num
            else:
                self._last_processed_num = to_process_num
            # Calculate and display fps
            #print(f"Processed frame {to_process_num}!")

            fps = self.__fps(curr_ts)
            if fps > 0:
                print(f"FPS: {fps}")

            # cv2.imshow('img', img)
            marker_corners, marker_ids = self.__FindArucoMarkers(img)
            # world marker size is different from robot markers (50 vs 80)
            irt_world = self.__FindMarkerPoses(marker_corners, marker_ids, self._world_tags, self._cam_mat, self._distortion, SensorManager.WORLD_TAG_SIZE) 
            irt_bots = self.__FindMarkerPoses(marker_corners, marker_ids, self._fast_things_tags, self._cam_mat, self._distortion, SensorManager.BOTS_TAG_SIZE) 
            irt_things = self.__FindMarkerPoses(marker_corners, marker_ids, self._slow_things_tags, self._cam_mat, self._distortion, SensorManager.THINGS_TAG_SIZE)
            if not self._world_located:
                print("Establishing world reference frame ...")
                anchor0 = None
                anchor1 = None
                anchor2 = None
                for id, rot, trans in irt_world:
                    # print(f"Found marker {id}")
                    if id == self._world_tags[0]: ## anchor 0
                        anchor0 = trans
                    elif id == self._world_tags[1]: ## anchor 1
                        anchor1 = trans
                    elif id == self._world_tags[2]: ## anchor 2
                        anchor2 = trans

                if anchor0 is not None and anchor1 is not None and anchor2 is not None:
                    self._locate_world_points.append((anchor0, anchor1, anchor2))
                    print(f"Got frame {len(self._locate_world_points)}/{self._LOCATE_WORLD_NUM} frames for calculating the world reference frame.")
                else:
                    print(f"Anchor 0: {anchor0 is not None} Anchor 1: {anchor1 is not None} Anchor 2: {anchor2 is not None}")

                if len(self._locate_world_points) == self._LOCATE_WORLD_NUM:
                    ## Enough number of frames, compute the transformation matrices
                    anchor0_avg = np.mean(np.vstack(tuple([points[0] for points in self._locate_world_points])), axis=0)
                    anchor1_avg = np.mean(np.vstack(tuple([points[1] for points in self._locate_world_points])), axis=0)
                    anchor2_avg = np.mean(np.vstack(tuple([points[2] for points in self._locate_world_points])), axis=0)

                    #print(f"Avg anchor 0, 1, 2 are {anchor0_avg} {anchor1_avg} {anchor2_avg}.")

                    self._wc_R, self._wc_T, self._cw_R, self._cw_T = self.__LocateWorld(anchor0_avg, anchor1_avg, anchor2_avg)

                    print(f"Camera position in world frame {self._cw_T}")

                    self._world_located = True

                return 0, []

            else:
                ## The world is already located, return robot positions (2D) and heading in 2D frame ([(id, x, y, omega)])
                robot_poses = []
                moving_irt = list(irt_things) + list(irt_bots)
                for id, rot, trans in moving_irt:
                    if id in self._moving_things_tags:
                        robot_trans_world = self._cw_R @ np.reshape(trans, (3, 1)) + self._cw_T
                        robot_heading_vec = (self._cw_R @ rot @ np.array([[0], [1], [0]])).flatten() # robot heading is aligned with the local y direction
                        robot_heading_omega = (1 if np.cross(np.array([1, 0, 0]), robot_heading_vec)[2] > 0 else -1) * np.arccos(np.dot(np.array([1, 0, 0]), robot_heading_vec))
                        robot_poses.append((id, robot_trans_world[0, 0], robot_trans_world[1, 0], robot_trans_world[2, 0], robot_heading_omega))

                        # DEMO
                        #if save_results is not None:
                            #robot_2d, _j = cv2.projectPoints(np.reshape(trans, (1, 3, -1)), cv2.Rodrigues(np.identity(3))[0], np.array([0, 0, 0], dtype=np.float32), self._cam_mat, self._distortion)
                            #img = cv2.putText(img, f"({int(robot_trans_world[0, 0] * 100) / 100.0},{int(robot_trans_world[1, 0] * 100) / 100.0})", (int(robot_2d[0, 0, 0]), int(robot_2d[0, 0, 1])), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 69, 255), thickness=3)
                if save_results is not None:
                    #v2.imwrite(save_results,img)
                    save_results.write(img)

                return 0, robot_poses

    def Close(self):
        self._cap.release()

        

# while True:
#     #img_und = cv2.undistort(img, cam_mat, distortion, None,
#     #                   newCameraMatrix=newcameramtx)
#     #print(img_und.shape)


#     anchor1 = None # 2
#     anchor2 = None # 8
#     anchor3 = None # 5
#     robotr = None
#     robott = None

#     for id, ro, tr in irt:
#         if id == 2:
#             anchor1 = tr
#         elif id == 8:
#             anchor2 = tr
#         elif id == 5:
#             anchor3 = tr
#         elif id == 0:
#             robott = tr
#             robotr = ro
    
#     if anchor1 is not None and anchor2 is not None and anchor3 is not None:
#         # Compute the plane
#         origin_cam_3d = anchor2
#         x_cam_3d = anchor2 - anchor3
#         x_cam_3d = x_cam_3d / np.linalg.norm(x_cam_3d)
#         z_cam_3d = anchor1 - anchor2
#         z_cam_3d = z_cam_3d / np.linalg.norm(z_cam_3d)
#         y_cam_3d = np.cross(z_cam_3d, x_cam_3d)



#         #print(f"Plane origin {origin_cam_3d} x axis {x_cam_3d}") 
#         # draw the origin and the axis
#         points_3d = np.vstack((origin_cam_3d, origin_cam_3d + x_cam_3d * 0.1, origin_cam_3d + y_cam_3d * 0.1, origin_cam_3d + z_cam_3d * 0.1))
#         points_2d, _j = cv2.projectPoints(points_3d, cv2.Rodrigues(np.identity(3))[0], np.array([0, 0, 0], dtype=np.float32), cam_mat, distortion)

#         # Axes
#         img = cv2.line(img, (int(points_2d[0, 0, 0]), int(points_2d[0, 0, 1])), (int(points_2d[1, 0, 0]), int(points_2d[1, 0, 1])), (0, 0, 255), 3)
#         img = cv2.line(img, (int(points_2d[0, 0, 0]), int(points_2d[0, 0, 1])), (int(points_2d[2, 0, 0]), int(points_2d[2, 0, 1])), (0, 255, 0), 3)
#         img = cv2.line(img, (int(points_2d[0, 0, 0]), int(points_2d[0, 0, 1])), (int(points_2d[3, 0, 0]), int(points_2d[3, 0, 1])), (255, 0, 0), 3)

#         boundary_3d = np.vstack((anchor2 + x_cam_3d * 0.03, anchor3 - x_cam_3d * 0.03, anchor3 - x_cam_3d * 0.03 + anchor1 -  anchor2, anchor1 + x_cam_3d * 0.03))
#         boundary_2d, _j = cv2.projectPoints(boundary_3d, cv2.Rodrigues(np.identity(3))[0], np.array([0, 0, 0], dtype=np.float32), cam_mat, distortion)

#         img = cv2.line(img, (int(boundary_2d[0, 0, 0]), int(boundary_2d[0, 0, 1])), (int(boundary_2d[1, 0, 0]), int(boundary_2d[1, 0, 1])), (144, 238, 144), 1)
#         img = cv2.line(img, (int(boundary_2d[1, 0, 0]), int(boundary_2d[1, 0, 1])), (int(boundary_2d[2, 0, 0]), int(boundary_2d[2, 0, 1])), (144, 238, 144), 1)
#         img = cv2.line(img, (int(boundary_2d[2, 0, 0]), int(boundary_2d[2, 0, 1])), (int(boundary_2d[3, 0, 0]), int(boundary_2d[3, 0, 1])), (144, 238, 144), 1)
#         img = cv2.line(img, (int(boundary_2d[3, 0, 0]), int(boundary_2d[3, 0, 1])), (int(boundary_2d[0, 0, 0]), int(boundary_2d[0, 0, 1])), (144, 238, 144), 1)

#         if robotr is not None and robott is not None:
#             robot_3d = np.vstack((robott, np.reshape(robotr @ np.array([[0.1], [0], [0]]) + np.reshape(robott, (3, 1)), (1, 3))))
#             robot_2d, _j = cv2.projectPoints(robot_3d, cv2.Rodrigues(np.identity(3))[0], np.array([0, 0, 0], dtype=np.float32), cam_mat, distortion)
#             img = cv2.line(img, (int(robot_2d[0, 0, 0]), int(robot_2d[0, 0, 1])), (int(robot_2d[1, 0, 0]), int(robot_2d[1, 0, 1])), (0, 69, 255), 3)

#     cv2.imshow('img',img)
#     k = cv2.waitKey(30) & 0xff
#     if k == 27:
#         break
# cap.release()
# cv2.destroyAllWindows()