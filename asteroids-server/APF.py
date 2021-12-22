'''
Artifical Potential Field path planning 
Loosely based on https://github.com/AtsushiSakai/PythonRobotics/blob/01874cee24e6944015a72e701140b23e5d291d47/PathPlanning/PotentialFieldPlanning/potential_field_planning.py
and
https://www.cs.mcgill.ca/~hsafad/robotics/
https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf
Calculate force directly, instead of a global potential map
'''

from typing import Callable, List, Tuple

from ObstacleMap import ObstacleMap
import numpy as np
from Util import shortest_angular_dist



class APF:
    ROBOT_R = 0.06
    def __init__(self, id: int, obs_map: ObstacleMap, reach_thres=0.08) -> None:
        self._obs = obs_map
        self._world_scaling = 1.0
        self._attr_co = 1.0 * self._world_scaling
        self._rep_co = 5.0 * self._world_scaling
        self._min_rep_dist = APF.ROBOT_R
        self._robot_id = id
        self._goal_reached_dist = reach_thres
        self._heading_reached_range = 10 / 180.0 * np.pi
        self._attr_reach = 0.3 # Attraction force does not increase after this distance
        self._max_v = 0.3
        ## Detect 
        self._goal_reached_cb = None
        self._waypoint_reached_cb = None

    ## Quatratic attractive potential
    def __cal_attr_force(self, x: List[float], gx: float, gy: float, to_goal: bool) -> np.ndarray:

        disp = np.array([gx, gy]) - np.array([x[0], x[1]])
        dist = np.linalg.norm(disp)
        
        if to_goal:
            if dist > self._attr_reach:
                disp = disp / dist * self._attr_reach
            return self._attr_co * disp
        else:
            return disp / dist * self._attr_reach * self._attr_co

    def __cal_rep_force(self, x: List[float], obx: float, oby: float, obr: float) -> np.ndarray:
        to_obs = np.array([x[0] - obx, x[1] - oby])
        to_obs_dist = np.linalg.norm(to_obs) - obr
        ## Is this necessary?
        if to_obs_dist <= 0:
            to_obs_dist = 0.01
        if to_obs_dist < self._min_rep_dist:
            repf = self._rep_co * (1 / to_obs_dist - 1 / self._min_rep_dist) * (to_obs / np.linalg.norm(to_obs))
            return repf
        else:
            return np.array([0, 0])
        
    def set_goal_reached_cb(self, cb: Callable[[], None]) -> None:
        self._goal_reached_cb = cb


    def set_waypoint_reached_cb(self, cb: Callable[[], None]) -> None:
        self._waypoint_reached_cb = cb
        
    ## Return the v and theta to track
    ## If gh is None, the robot is going to a waypoint rather than the final goal
    def goto(self, x, gx, gy, gh) -> Tuple[float, float]:
        ## Going to the final goal
        if np.hypot(x[0] - gx, x[1] - gy) < self._goal_reached_dist:
            if gh is not None:
                if np.absolute(gh - x[2]) < self._heading_reached_range and self._goal_reached_cb is not None:
                    self._goal_reached_cb()
                print(f"Tracking angle {gh}")
                return 0, gh
            elif self._waypoint_reached_cb is not None:
                self._waypoint_reached_cb()
        ## Only consider moving robot obstacles in local planning
        ## How about hands? Maybe just trust on the human 
        all_obs = self._obs.get_all_moving_obs_positions_sizes(except_id = self._robot_id)
        rep_force = np.sum(np.array([self.__cal_rep_force(x, ob[0], ob[1], ob[2]) for ob in all_obs]), axis=0)
        attr_force = self.__cal_attr_force(x, gx, gy, to_goal=(gh is not None))
        force = rep_force + attr_force
        force_mag = np.linalg.norm(force)
        force_mag += 0.1
        if force_mag > self._max_v:
            force = force / force_mag * self._max_v
            force_mag = self._max_v

        theta = np.arctan2(force[1], force[0])
        mag = force_mag * (1 if np.dot(np.array([np.cos(x[2]), np.sin(x[2])]), force) > 0 else -1)

        return mag, theta
    
