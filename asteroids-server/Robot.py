###
# Robot states and control
###
from astar import AStarPlanner
from enum import Enum
from APF import APF
from ObstacleMap import ObstacleMap
from typing import Callable, List, Tuple, Union
from functools import partial

import numpy as np
import math
from Dwa import Config, DWAPlanner
import Util
import time
from LowPass import LowPass
from EKF import EKF

import matplotlib.pyplot as plt

class RobotStatus(Enum):
    STANDBY = 1
    AUTONAV = 2
    PINNED = 3
    RC = 4

class Robot:
    # Robot constants
    ROBOT_WHEEL_RADIUS = 0.021
    ROBOT_WHEEL_DIST = 0.101
    POS_FILTER_CUTOFF = 5.0
    ANG_FILTER_CUTOFF = 5.0
    FILTER_TIME_STEP = 1 / 25.0
    EXISTENCE_THRES = 60 ## About 2s given the current FPS 
    DISENGAGE_IN = 30 # disengage after 30s if no user is on this robot

    def __init__(self, id: int, obs_map: ObstacleMap, global_planner: AStarPlanner) -> None:
        self._id = id
        self._curr_target = None
        self._pos = np.zeros(2)
        self._z = 0
        self._heading = 0
        self._prev_ts = time.time()
        self._v = 0
        self._omega = 0
        self._u = np.zeros(2)
        self._target_pos = np.zeros(2)
        self._target_h = 0
        self._path_to_target = np.array([[0, 0]])
        self._ind_on_path = 0
        self._dwa_config = Config(id)
        # custom config parameters
        self._dwa_config.robot_radius = Robot.ROBOT_WHEEL_DIST * 1.1 / 2.0

        ## Obstacles, local planners etc.
        self._ob_map = obs_map
        #self._local_planner = DWAPlanner(self._dwa_config, self._ob_map)
        self._local_planner = APF(id, obs_map)
        self._local_planner.set_waypoint_reached_cb(self.__waypoint_cb)
        self._global_planner = global_planner
        ## Only initialize filters when there is data coming in 
        self._pos_x_lp = None
        self._pos_y_lp = None
        self._pos_z_lp = None
        self._heading_lp = None
        ## remove the robot if not tracked for some number of consecutive frames
        self._existence_count = Robot.EXISTENCE_THRES 
        ## Extended Kalman filter
        self._ekf = EKF()
        #self._proc_cov = np.diag([0.01, 0.01, 0.001, 0.3, 0.3])
        #self._sens_cov = np.diag([0.01, 0.01, 0.04])
        self._proc_cov = np.diag([0.01, 0.01, 0.01])
        self._sens_cov = np.diag([0.01, 0.01, 0.09])
        self._ekf_P = np.identity(5)
        self._ekf_P = np.identity(3)

        ## User control and ownership
        self._users = []
        self._status = RobotStatus.STANDBY
        # self._status_before_pickedup = RobotStatus.STANDBY
        self._up = False
        self._z_switch_count = 0
        self._z_switch_threshold = 30
        # Engaged robot is trying to fulfill some user commands, non-engaged bots are managed by the system
        self._engaged = False 
        self._disengage_start_ts = float('Inf') # Max value means already engaged or disengaged
        self._engagement_changed_cb = None


        ## Debug
        self._computed_v = []
        self._computed_omega = []
        self._vel_ts = []

    ## TODO: use a getter?
    def get_id(self) -> int:
        return self._id

    def get_pos(self) -> np.ndarray:
        return self._pos
    
    def get_heading(self) -> float:
        return self._heading

    def get_v(self) -> float:
        return self._v

    def get_omega(self) -> float:
        return self._omega

    def check_standby(self) -> bool:
        return self._status == RobotStatus.STANDBY

    def check_rc(self) -> bool:
        return self._status == RobotStatus.RC

    def check_autonav(self) -> bool:
        return self._status == RobotStatus.AUTONAV

    def check_pinned(self) -> bool:
        return self._status == RobotStatus.PINNED

    def check_engaged(self) -> bool:
        return self._engaged

    def set_engagement_changed_cb(self, engagement_changed_cb: Callable[[bool], None]) -> None:
        self._engagement_changed_cb = engagement_changed_cb

    def set_status(self, status: RobotStatus) -> None:
        # self._prev_status = self._status
        self._status = status

    def check_in_air(self) -> bool:
        return self._up

    def pick_up_bot(self) -> bool:
        if not self._up:
            self._z_switch_count += 1
            if self._z_switch_count == self._z_switch_threshold:
                self._up = True
                self._z_switch_count = 0
                return True
        else:
            self._z_switch_count = 0
        return False

    def put_down_bot(self):
        if self._up:
            self._z_switch_count += 1
            if self._z_switch_count == self._z_switch_threshold:
                self._up = False
                self._z_switch_count = 0
                return True
        else:
            self._z_switch_count = 0
        return False


    def add_user(self, username: str) -> None:
        if username not in self._users:
            print(f"Adding user {username} to robot {self._id}")
            self._users.append(username)
            if not self._engaged: 
                self._engaged = True
                self._disengage_start_ts = float('Inf') # User added, so reset disengagement counter
                if self._engagement_changed_cb is not None:
                    self._engagement_changed_cb(True)

    def remove_user(self, username) -> None:
        if username in self._users:
            self._users = list(filter(lambda u: u != username, self._users))
            # Start disengagement process
            if len(self._users) == 0:
                self._disengage_start_ts = time.time()                  

    def check_should_disengage(self, current_ts: float) -> bool:
        return self._engaged and current_ts - self._disengage_start_ts > Robot.DISENGAGE_IN

    def disengage(self) -> None:
        self._engaged = False
        self._disengage_start_ts = float('Inf') ## Fully disengaged, also reset the timer
        if self._engagement_changed_cb is not None:
            self._engagement_changed_cb(False)

    def get_users(self) -> List[str]:
        return self._users

    def get_owner(self) -> Union[str, None]:
        return self._users[0] if len(self._users) > 0 else None

    @classmethod
    def __to_wheel_velocity(cls, v: float, omega: float) -> Tuple[float, float]:
        # See Control of Mobile Robots – Differential Drive Robots video 7:39
        #print(f"v {v} theta {theta}")
        wr = (2 * v + omega * cls.ROBOT_WHEEL_DIST) / 2.0 
        wl = (2 * v - omega * cls.ROBOT_WHEEL_DIST) / 2.0 
        #print(f"l {wl} r {wr}")
        return wl, wr

    def __update_pose_lp_filtered(self, x: float, y: float, h: float, delta: float, predict: bool) -> None:
        if self._pos_x_lp is None:
            self._pos_x_lp = LowPass(Robot.POS_FILTER_CUTOFF, Robot.FILTER_TIME_STEP, x, False)
        if self._pos_y_lp is None:
            self._pos_y_lp = LowPass(Robot.POS_FILTER_CUTOFF, Robot.FILTER_TIME_STEP, y, False)
        if self._heading_lp is None:
            self._heading_lp = LowPass(Robot.ANG_FILTER_CUTOFF, Robot.FILTER_TIME_STEP, h, True)

        if not predict:
            self._pos[0] = self._pos_x_lp.filter(x)

            self._pos[1] = self._pos_y_lp.filter(y)

            self._heading = self._heading_lp.filter(h)
        else:
            self._pos[0] = self._pos[0] + np.cos(self._heading) * self._v * delta
            self._pos[1] = self._pos[1] + np.sin(self._heading) * self._v * delta
            pred_heading = self._heading + self._omega * delta
            self._heading = np.arctan2(np.sin(pred_heading), np.cos(pred_heading))
            self._pos_x_lp.force_update(self._pos[0])
            self._pos_y_lp.force_update(self._pos[1])

    def __update_z_lp_filtered(self, z: float) -> None:
        if self._pos_z_lp is None:
            self._pos_z_lp = LowPass(Robot.POS_FILTER_CUTOFF, Robot.FILTER_TIME_STEP, z, False)
        
        self._z = self._pos_z_lp.filter(z)

    def __update_state_kalman(self, x: float, y: float, h: float, delta: float, guess=False) -> None:
        observation = np.reshape(np.array([x, y, h]), (-1, 1))
        curr_state = np.reshape(np.array([self._pos[0], self._pos[1], self._heading]), (-1, 1))
        if not guess:
            state, self._ekf_P = self._ekf.update(curr_state, np.reshape(self._u, (-1, 1)), self._ekf_P, observation, self._proc_cov, self._sens_cov, delta)
        else:
            state, self._ekf_P = self._ekf.predict(curr_state, np.reshape(self._u, (-1, 1)), self._ekf_P, self._proc_cov, delta)
        self._pos = np.array([state[0, 0], state[1, 0]])
        self._heading = state[2, 0]
        # self._v= state[3]
        # self._omega = state[4]


    # def predict_robot_state(self, ts: float) -> None:
    #     ## Predict where the robot would be if we assume linear and angular velocities do not change
    #     ## For temporary tracking lost
    #     delta = ts - self._prev_ts
    #     # heading_next = self._heading + self._omega * delta
    #     # heading_next = math.atan2(math.sin(heading_next), math.cos(heading_next))
    #     # x_next = self._pos[0] + self._v * math.cos(heading_next) * delta
    #     # y_next = self._pos[1] + self._v * math.sin(heading_next) * delta

    #     # self.__update_pose_lp_filtered(x_next, y_next, heading_next)
    #     ## Make a guess based on previously sent control signal
    #     previous
    #     self.__update_state_kalman(-1, -1, -1, delta, True)
    #     self._prev_ts = ts

    #     ## Debug
    #     # self._computed_theta.append(self._theta)
    #     # self._computed_v.append(self._v)
    #     self._computed_v.append(self._pos[0])
    #     self._computed_omega.append(self._heading)
    #     self._vel_ts.append(ts)

    def update_robot_state(self, x: float, y: float, heading: float, ts: float, predict = False) -> None:
        ## Keep internal filters for robot states. 
        delta = ts - self._prev_ts

        prev_xyh = (np.copy(self._pos), self._heading)

        ## Would ignore whatever state value passed in if predicit == True
        self.__update_pose_lp_filtered(x, y, heading, delta, predict)
        # self.__update_state_kalman(x, y, heading, delta, predict)

        #TODO: add some kind of boudary check if delta is too large (due to lost tracking etc) or too small
        ## Should probably stop a robot after it has been considered lost
        if delta < 0.01 or delta > 1:
            self._v = self._v
            self._omega = self._omega
        else:
            displacement = self._pos - prev_xyh[0]
            prev_heading_vec = np.array([np.cos(prev_xyh[1]), np.sin(prev_xyh[1])])
            sign = 1 if np.dot(displacement, prev_heading_vec) > 0 else -1 # There is an edge case of moving sideways, which in theory should not happen
            self._v = np.linalg.norm(displacement) * sign / delta 
            self._omega = Util.shortest_angular_dist(prev_xyh[1], self._heading) / delta 

        self._prev_ts = ts
        ## Reset the counter if ever found
        if not predict:
            self._existence_count = Robot.EXISTENCE_THRES

        ## Debug
        #self._computed_theta.append(self._theta)
        #self._computed_v.append(self._v)
        self._computed_v.append(self._v)
        self._computed_omega.append(self._heading)
        self._vel_ts.append(ts)

    def update_pos_z(self, zval: float) -> None:
        self.__update_z_lp_filtered(zval)

    def get_pos_z(self)-> float:
        return self._z

    def set_target(self, x: float, y: float, heading: float) -> None:
        ## Get path using the global planner TODO: when to run it again?
        if self._global_planner.check_ready():
            p2t_x, p2t_y, found = self._global_planner.planning(sx=self._pos[0], sy=self._pos[1], gx=x, gy=y, mask_c=tuple(self._pos), mask_r=0.06 * 2.1)
            if found: 
                self._path_to_target = np.vstack((np.flip(p2t_x), np.flip(p2t_y))).T
                # print(f"Path planned is {self._path_to_target}")
            #else:
            #    self._path_to_target = np.array([[x, y]])

                self._ind_on_path = 0
                self._target_pos = np.array([x, y])
                self._target_h = heading
                self.set_status(RobotStatus.AUTONAV)

    def just_go(self, x: float, y: float, heading: float) -> None:
        self._path_to_target = np.array([[x, y]])
        self._ind_on_path = 0
        self._target_pos = np.array([x, y])
        self._target_h = heading
        self.set_status(RobotStatus.AUTONAV)

    def find_viable_target(self, candidate_targets: np.ndarray) -> np.ndarray:
        viable = np.apply_along_axis(lambda target: self._global_planner.planning(sx=self._pos[0], sy=self._pos[1], gx=target[0], gy=target[1], mask_c=tuple(self._pos), mask_r=0.06 * 2.1)[2],
                                    axis=1,
                                    arr=candidate_targets)
        if np.any(viable):
            return candidate_targets[viable][0]
        else:
            return np.array([])

    def set_reached_cb(self, cb: Callable[[int], None]) -> None:
        self._local_planner.set_goal_reached_cb(partial(cb, self._id))

    def __waypoint_cb(self) -> None:
        if self._ind_on_path + 1 < len(self._path_to_target):
            self._ind_on_path += 1

    # TODO: still run local planner if the robot is not found? Maybe not.
    def run_local_planner(self) -> Tuple[float, float]:
        x = [self._pos[0], self._pos[1], self._heading, self._v, self._omega]
        # v, theta = self._local_planner.goto(x, self._target_pos[0], self._target_pos[1], self._target_h)
        if self._ind_on_path == len(self._path_to_target) - 1:
            v, theta = self._local_planner.goto(x, self._target_pos[0], self._target_pos[1], self._target_h)
        else:
            wp_now = self._path_to_target[self._ind_on_path]
            v, theta = self._local_planner.goto(x, wp_now[0], wp_now[1], None)

        u = np.array([v, theta])
        self._u = u
        # Convert to robot wheel velocities
        # wheel_vel = Robot.__to_wheel_velocity(u_global[0], u_global[1])
        # print(f"Wheel velocity {wheel_vel}")

        ## Do not clip?
        #s_left = wheel_vel[0] 
        #s_right = wheel_vel[1] 
        #return s_left, s_right
        return v, theta

    ## Return true is the robot is still considered existent
    def check_existence(self) -> bool:
        self._existence_count -= 1
        # print(f"existence count for robot {self._id} is {self._existence_count}")
        return self._existence_count > 0

    def stop(self) -> Tuple[float, float]:
        ## Debug
        ## Plot things
        #fig, ax = plt.subplots(2, 1)
        #ax[0].plot(self._vel_ts, self._computed_v)
        #ax[1].plot(self._vel_ts, self._computed_omega)
        #plt.show()

        return 0, 0


        
        


    