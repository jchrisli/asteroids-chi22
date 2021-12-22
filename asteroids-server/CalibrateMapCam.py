'''
Calibrate the extrinsic matrix of the map camera
Save results to a json {ext: [[]], int: [[]], dist: []}
'''

import cv2
import cv2.aruco as aruco

import numpy as np
import yaml
import json

INTRINSIC_PATH = 'c922-720p-calib.yaml'
WORLD_TAGS = [14, 13, 12]
CAM_NUM = 1
LOCATE_WORLD_NUM = 10

int_mat = np.zeros((3, 3))
ext_mat = np.zeros((3, 3))
dist_vec = np.zeros(5)

with open(INTRINSIC_PATH) as fr:
    c = yaml.load(fr)
    int_mat = c['camera_matrix']
    dist_vec = c['dist_coefs']

    int_mat_np = np.array(int_mat)
    dist_np = np.array(dist_vec).flatten()

params = {}
params['int'] = int_mat
params['dist'] = dist_np.tolist()

vidcap = cv2.VideoCapture(CAM_NUM)
vidcap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
#vidcap.set(cv2.CAP_PROP_FPS, 60.0) # Use higher FPS to reduce motion blur
vidcap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
vidcap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
# self._cap.set(cv2.CAP_PROP_BACKEND, cv2.CAP_MSMF)
#vidcap.open(CAM_NUM, cv2.CAP_MSMF)

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
aruco_param = aruco.DetectorParameters_create()
locate_world_points = []

debug_img_count = 0

def __FindArucoMarkers(img, a_dict, a_param):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, a_dict, parameters = a_param)
    #print(f"Marker ids are {ids}")
    ids = [ii[0] for ii in ids] if ids is not None else []
    # rv, tv, _o = aruco.estimatePoseSingleMarkers(corners, markerSide / 1000, camMat, dist)
    # rm = [cv2.Rodrigues(rvec)[0] for rvec in rv] # convert to rotation matrix
    # res = zip(ids, rm, tv) # (tagid, rotation_mat, translation_vec)
    #print(list(res))
    #print(ids)
    return corners, ids

def __FindMarkerPoses(corners, ids, ids_to_find, camMat, distParams, markerSize):
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

def __LocateWorld(anchor0, anchor1, anchor2):
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

## Calculate extrinsic matrix based on tag positions

while True:
    success, img = vidcap.read()
    if not success:
        print("Error: cannot read video frame")
        exit()
    else:
        # curr_ts = time.time()
        # last_proc_ts = self._last_processed_ts
        #to_process_num = int((curr_ts - self._inception_ts) / self._target_interval)
        ## Only do the calculation at the target FPS
        #if self._last_processed_num > 0:
        #    if to_process_num <= self._last_processed_num:
        #        return 1, []
        #    else:
                #self._last_processed_num = to_process_num
        #else:
            #self._last_processed_num = to_process_num
        # Calculate and display fps
        #print(f"Processed frame {to_process_num}!")

        #fps = self.__fps(curr_ts)
        #if fps > 0:
        #    print(f"FPS: {fps}")

        # cv2.imshow('img', img)

        if debug_img_count < 1:
            cv2.imwrite('j5restest.jpg', img)
            debug_img_count += 1

        marker_corners, marker_ids = __FindArucoMarkers(img, aruco_dict, aruco_param)
        # world marker size is different from robot markers (50 vs 80)
        irt_world = __FindMarkerPoses(marker_corners, marker_ids, WORLD_TAGS, int_mat_np, dist_np, 60) 
        # irt_bots = self.__FindMarkerPoses(marker_corners, marker_ids, self._robot_tags, self._cam_mat, self._distortion, 80) 
        print("Establishing world reference frame ...")
        anchor0 = None
        anchor1 = None
        anchor2 = None
        for id, rot, trans in irt_world:
            # print(f"Found marker {id}")
            if id == WORLD_TAGS[0]: ## anchor 0
                anchor0 = trans
            elif id == WORLD_TAGS[1]: ## anchor 1
                anchor1 = trans
            elif id == WORLD_TAGS[2]: ## anchor 2
                anchor2 = trans

        if anchor0 is not None and anchor1 is not None and anchor2 is not None:
            locate_world_points.append((anchor0, anchor1, anchor2))
            print(f"Got frame {len(locate_world_points)}/{LOCATE_WORLD_NUM} frames for calculating the world reference frame.")
        else:
            print(f"Anchor 0: {anchor0 is not None} Anchor 1: {anchor1 is not None} Anchor 2: {anchor2 is not None}")

        if len(locate_world_points) == LOCATE_WORLD_NUM:
            ## Enough number of frames, compute the transformation matrices
            anchor0_avg = np.mean(np.vstack(tuple([points[0] for points in locate_world_points])), axis=0)
            anchor1_avg = np.mean(np.vstack(tuple([points[1] for points in locate_world_points])), axis=0)
            anchor2_avg = np.mean(np.vstack(tuple([points[2] for points in locate_world_points])), axis=0)

            #print(f"Avg anchor 0, 1, 2 are {anchor0_avg} {anchor1_avg} {anchor2_avg}.")

            wc_R, wc_T, cw_R, cw_T = __LocateWorld(anchor0_avg, anchor1_avg, anchor2_avg)

            print(f"Camera position in world frame {cw_T}")
            params['ext_R'] = wc_R.tolist()
            params['ext_T'] = wc_T.tolist()

            with open('c922-720p-calib.json', 'w', encoding='utf-8') as f:
                json.dump(params, f, ensure_ascii=False, indent=4)
            #self._world_located = True
            break

vidcap.release()
        