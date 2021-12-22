from ObstacleMap import ObstacleMap, StaticObstacle
from typing import Dict, List, Tuple
import socketio
import json
from Robot import Robot
from RobotManager import RobotManager
import numpy as np
from Person import Person
from Study.DataLogger import DataLogger
import time

from colorama import Fore

class MapCamera:
    def __init__(self, param_path: str) -> None:
        # Read the json file that contains the camera parameters (intrinsic, extrinsic, distortion)
        with open(param_path, 'r') as f:
            map_cam_params = json.load(f)
            self._map_cam_ext_r = map_cam_params['ext_R']
            self._map_cam_ext_t = map_cam_params['ext_T'] ## Note this is a column vector
            self._map_cam_int = map_cam_params['int']
            self._map_cam_dist = map_cam_params['dist']
            self._map_cam_proj = self._map_cam_int @ np.hstack((self._map_cam_ext_r, self._map_cam_ext_t)) 

    '''
        Return a ray in 3d, direction determined by the pixel clicked
        params 3 x 1 pixel position (last element 1), camera matrix 3 * 4: p
        return (np.array(3, 1): origin, np.array(3, 1): direction)
        See http://answers.opencv.org/question/117354/back-projecting-a-2d-point-to-a-ray/?answer=123124#post-id-123124 for details
    '''
    @staticmethod
    def __get_pixel_ray(pix: List[float], P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        pix_arr = np.reshape(np.array([pix[0], pix[1], 1]), (3, 1))
        M_inv = np.linalg.inv(P[:,:3])
        p_4 = P[:, 3].reshape(3, 1)
        direction = M_inv.dot(pix_arr)
        origin = - M_inv.dot(p_4)
        return origin, direction

    '''
    Interset a ray (defined by an origin ray_o and a direction ray_d) with a plane 
    defined by a point plane_p and a normal vector plane_n
    All input arguments are 3-element 1d array
    Return a 3-element 1d array
    '''
    @staticmethod
    def __interset_with_plane(ray_o: np.ndarray, ray_d: np.ndarray, plane_p: np.ndarray, plane_n: np.ndarray) -> np.ndarray:
        o = ray_o
        d = ray_d
        n = plane_n
        p_n = plane_p
        t = n.dot(p_n - o) / (n.dot(d))
        intersection = o + t * d
        return intersection

    def pixel_to_ground(self, pix_pt: List[float], frame_w: float, frame_h: float) -> np.ndarray:
        ## first project pixel to a ray
        IMG_W = 1280.0
        IMG_H = 720.0
        pix_normalized = [pix_pt[0] * IMG_W / frame_w, pix_pt[1] * IMG_H / frame_h]
        rayo, rayd = MapCamera.__get_pixel_ray(pix_normalized, self._map_cam_proj)
        point_on_ground = MapCamera.__interset_with_plane(rayo.flatten(), rayd.flatten(), np.zeros(3), np.array([0, 0, 1.0]))
        return point_on_ground[:2]

    '''
    pos: 2-element array
    heading: float
    return: 3-element array (x2d, y2d, heading2d)
    '''
    def ground_to_pix(self, pos: List[float], heading: float, z_offset :float =0) -> List[float]:
        pos3d = np.reshape(np.array([pos[0], pos[1], z_offset, 1]), (-1, 1))
        pos2d = self._map_cam_proj @ pos3d
        pos2d_pix = [pos2d[0, 0] / pos2d[2, 0], pos2d[1, 0] / pos2d[2, 0]]

        heading_3d = np.reshape(np.array([np.cos(heading), np.sin(heading), 0, 1]), (-1, 1)) + pos3d - np.reshape(np.array([0, 0, 0, 1]), (-1,1))
        heading_3d_proj = self._map_cam_proj @ heading_3d
        heading_2d = np.arctan2(heading_3d_proj[1, 0] / heading_3d_proj[2, 0] - pos2d_pix[1], heading_3d_proj[0, 0] / heading_3d_proj[2, 0] - pos2d_pix[0])
        return pos2d_pix + [heading_2d]

        

class CommandManager:
    def __init__(self, robot_manager: RobotManager, mp_cam: MapCamera, person: Person, obstacles: ObstacleMap, logger: DataLogger=None) -> None:
        self._rmanager = robot_manager
        self._p = person
        debug_ws = False
        self._sio = socketio.Client(logger=debug_ws, engineio_logger=debug_ws)
        self._WS_ADDR = 'https://videochat-new.herokuapp.com'
        #self._WS_ADDR = 'http://localhost:5000'

        ## WebSocket event handlers
        self._sio_alive = False
        self._sio.on('robot-go', self.ws_on_robotgo, namespace='/robots')
        self._sio.on('robot-rc', self.ws_on_robotrc, namespace='/robots')
        self._sio.on('robot-select', self.ws_on_user_select_robot, namespace='/robots')
        self._sio.on('connect', self.ws_on_connect, namespace='/robots')
        self._sio.on('disconnect', self.ws_on_disconnect, namespace='/robots')


        self._sio.connect(self._WS_ADDR, namespaces=['/robots'])
        self._cam = mp_cam

        self._obs = obstacles

        ## Callbacks for demonstrator robot management
        self._rmanager.set_bot_spotlight_changed_cb(self.__send_spotlight_update)

        ## Data logging
        self._logger = logger

    # @self._sio.on('robot-go')
    def ws_on_robotgo(self, data):
        #data_j = json.loads(data)
        if not self._rmanager.check_spotlight_loc():
            data_j = data
            raw_x, raw_y, raw_w, raw_h, workspace_id = data_j['x'], data_j['y'], data_j['w'], data_j['h'], data_j['workspace']
            field_pos = self._cam.pixel_to_ground([raw_x, raw_y], raw_w, raw_h)
            print(f"Robot goal {field_pos}")

            username = data['username']
            valid_robot = self._rmanager.get_closest_valid_robot(pos=field_pos, username=username)
            if valid_robot is not None:
                ## So the user control some robot, check if there are any unoccupied robots that are closer to the goal
                ## For now we just direct the robot there
                self._rmanager.add_user_to_robot(valid_robot, username)
                ## Retrieve workspace 
                workspace = self._obs.get_static_obs(workspace_id)
                if workspace is not None:
                    workspace_c = workspace.get_center()
                    robot_heading = np.arctan2(workspace_c[1] - field_pos[1], workspace_c[0] - field_pos[0]) 
                else:
                    person_pos = self._p.get_person_pos()
                    robot_heading = np.arctan2(person_pos[1] - field_pos[1], person_pos[0] - field_pos[0]) if person_pos is not None else np.pi / 2
                print(Fore.CYAN + f"Set target heading {robot_heading} for robot {valid_robot}" + Fore.RESET)
                self._rmanager.set_robot_goal(valid_robot, field_pos[0], field_pos[1], robot_heading)

                ## Logging
                if self._logger is not None:
                    self._logger.write_interaction_log(time.time(), username, 'robot-go', [field_pos[0], field_pos[1], robot_heading, workspace_id])
            else:
                ## Do not do anything. TODO: give some feedback?
                pass

    def ws_on_robotrc(self, data):
        # print(f"Received RC data {data}")
        if not self._rmanager.check_spotlight_loc():
            username, robot_id = data['username'], data['id']
            if self._rmanager.user_has_control(robot_id, username):
                self._rmanager.rc_robot(robot_id, data['direction'])

                if self._logger is not None:
                    self._logger.write_interaction_log(time.time(), username, 'robot-rc', [robot_id, data['direction']])

    ## When a user selects a robot
    def ws_on_user_select_robot(self, data):
        username, robot_id = data['username'], data['id']
        self._rmanager.add_user_to_robot(robot_id, username)
        if self._logger is not None:
            self._logger.write_interaction_log(time.time(), username, 'robot-select', [robot_id])
    
    def ws_on_connect(self):
        self._sio_alive = True

    def ws_on_disconnect(self):
        self._sio_alive = False

    @staticmethod
    def __make_workspace_update(sobs: StaticObstacle, cam: MapCamera) -> Dict:
        workspace_id = sobs.get_id1()
        corner1_on_cam = cam.ground_to_pix(list(sobs.get_id1_pos()), heading=0, z_offset=0)
        corner2_on_cam = cam.ground_to_pix(list(sobs.get_id2_pos()), heading=0, z_offset=0)
        wp_update = {'id': workspace_id, 'x1': corner1_on_cam[0], 'x2': corner2_on_cam[0], 
                    'y1': corner1_on_cam[1], 'y2': corner2_on_cam[1]}
        # print(f"Createing update {wp_update}")
        return wp_update

    @staticmethod
    def __make_robot_update(r: Robot, cam: MapCamera, pinned_robots: List[int]) -> Dict:
        pos = r.get_pos()
        heading = r.get_heading()
        xcam = cam.ground_to_pix(pos.tolist(), heading, z_offset=0.195) # Robot height is 19.5 cm
        #print(f"3D heading is {heading} 2d heading {xcam[2]}")
        update = {'id': int(r.get_id()), 'x': xcam[0], 'y': xcam[1], 'heading': xcam[2], 'users': r.get_users(), 'pinned': r.get_id() in pinned_robots}
        # print(f"Type of message {type(update['id'])} {type(update['x'])} {type(update['y'])} {type(update['heading'])} {type(update['users'])}")
        return update

    def send_robot_update(self) -> None:
        # robots = self._rmanager.get_all_robots()
        robots = self._rmanager.get_all_unmanaged_robot()
        robot_update = [CommandManager.__make_robot_update(r, cam=self._cam, pinned_robots=self._rmanager.get_pinned_robot())
                         for r in robots]
        # print(robot_update)
        if self._sio_alive:
            self._sio.emit('robot-update', robot_update, namespace='/robots')

    def send_workspace_update(self) -> None:
        static_obs = self._obs.get_all_static_obs()
        static_update = [CommandManager.__make_workspace_update(s, cam=self._cam) for s in static_obs]
        if self._sio_alive:
            self._sio.emit('workspace-update', static_update, namespace='/robots')

    def __send_spotlight_update(self, bot_id: int, spotlight_on: bool) -> None:
        spotlight_msg = {'id': int(bot_id), 'on': spotlight_on}
        if self._sio_alive:
            self._sio.emit('spotlight', spotlight_msg, namespace='/robots')

    def close(self) -> None:
        self._sio.disconnect()
            
