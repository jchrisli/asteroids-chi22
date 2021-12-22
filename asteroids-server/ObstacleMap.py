import math
from typing import List, Tuple, Union

import numpy as np

class RobotObstacle:
    ROBOT_R_ = 0.06 # class variable
    def __init__(self, id: int, x: float, y: float, v: float) -> None:
        self.id = id # use robot id (positive) for moving obstacles (robots); static obstacles have negative ids
        self.x = x
        self.y = y
        self.r = RobotObstacle.ROBOT_R_
        self.r_inf = self.r + RobotObstacle.ROBOT_R_
        self._v = v

    def get_discrete_obs(self, resolution: float) -> np.ndarray:
        left = self.x - self.r_inf
        right = self.x + self.r_inf
        top = self.y + self.r_inf
        bottom = self.y - self.r_inf

        y_num = int((top - bottom) / resolution) + 1
        x_num = int((right - left) / resolution) + 1

        x_in_row = np.arange(start = left + resolution/2, stop = left + x_num * resolution,
                            step = resolution)
        x_all_rows = np.tile(x_in_row, y_num)
        y_in_col = np.arange(start = bottom + resolution/2, stop = bottom + y_num * resolution,
                            step = resolution)
        y_all_cols = np.repeat(y_in_col, x_num)

        rect_points =  np.vstack((x_all_rows, y_all_cols)).T
        return rect_points[np.hypot(rect_points[:,0] - self.x, rect_points[:,1] - self.y) < self.r_inf]

    def update_state(self, x: Union[float, None], y:Union[float, None], v: Union[float, None])-> None: 
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if v is not None:
            self._v = v

    def get_static(self) -> bool:
        return self._v  < 0.1 ##TODO: A good threshold?

    def check_within(self, x: float, y: float) -> bool:
        return np.hypot(x -self.x, y - self.y) < self.r_inf

class StaticObstacle:
    def __init__(self, id1: int, id2: int) -> None:
        self._id1 = id1
        self._id2 = id2
        self._id1_xy_set = False
        self._id2_xy_set = False
        self._id1_xy = (0, 0)
        self._id2_xy = (0, 0)
        
    def check_id_match(self, id_to_check :int) -> bool:
        return id_to_check == self._id1 or id_to_check == self._id2 

    def check_initialized(self) -> bool:
        return self._id1_xy_set and self._id2_xy_set

    def get_id1(self) -> int:
        return self._id1 

    def get_id2(self) -> int:
        return self._id2

    def get_id1_pos(self) -> Tuple[float, float]:
        return self._id1_xy

    def get_id2_pos(self) -> Tuple[float, float]:
        return self._id2_xy

    def update(self, id:int, x:float, y: float) -> None:
        if id == self._id1:
            self._id1_xy = (x, y)
            if not self._id1_xy_set:
                self._id1_xy_set = True
        elif id == self._id2:
            self._id2_xy = (x, y)
            if not self._id2_xy_set:
                self._id2_xy_set = True

    def get_discrete_obs(self, resolution: float, expand_by: float = 0) -> np.ndarray:
        if self.check_initialized():
            left = min(self._id1_xy[0], self._id2_xy[0]) - expand_by
            right = max(self._id1_xy[0], self._id2_xy[0]) + expand_by
            bottom = min(self._id1_xy[1], self._id2_xy[1]) - expand_by
            top = max(self._id1_xy[1], self._id2_xy[1]) + expand_by

            y_num = int((top - bottom) / resolution) + 1
            x_num = int((right - left) / resolution) + 1

            x_in_row = np.arange(start = left + resolution/2, stop = left + x_num * resolution,
                                step = resolution)
            x_all_rows = np.tile(x_in_row, y_num)
            y_in_col = np.arange(start = bottom + resolution/2, stop = bottom + y_num * resolution,
                                step = resolution)
            y_all_cols = np.repeat(y_in_col, x_num)

            return np.vstack((x_all_rows, y_all_cols)).T
        else:
            return np.array([])

    def get_center(self) -> Tuple[float, float]:
        return ((self._id1_xy[0] + self._id2_xy[0])/2, (self._id1_xy[1] + self._id2_xy[1])/2 )

    def get_corners(self, expand_by: float = 0) -> np.ndarray:
        if self.check_initialized():
            left = min(self._id1_xy[0], self._id2_xy[0]) - expand_by
            right = max(self._id1_xy[0], self._id2_xy[0]) + expand_by
            bottom = min(self._id1_xy[1], self._id2_xy[1]) - expand_by
            top = max(self._id1_xy[1], self._id2_xy[1]) + expand_by
            return np.array([[left, bottom], [right, bottom], [right, top], [left, top]])
        else:
            return np.array([])

    def check_within(self, x: float, y: float, expand_by: float) -> bool:
        if self.check_initialized():
            left = min(self._id1_xy[0], self._id2_xy[0]) - expand_by
            right = max(self._id1_xy[0], self._id2_xy[0]) + expand_by
            bottom = min(self._id1_xy[1], self._id2_xy[1]) - expand_by
            top = max(self._id1_xy[1], self._id2_xy[1]) + expand_by
            return x > left and x < right and y > bottom and y < top
        else:
            return False

## Use ObstacleMap to avoid obstacles for both autonomous navigation and teleoperation (reject commands that will lead to collision)
class ObstacleMap:
    def __init__(self, workspace_o: List[float], workspace_wh: List[float], workspace_resolution: float) -> None:
        # Static obstacles are working areas and field boundaries
        # Dynamic obstacles are robots (hands?)
        self.obs_ = {}
        # For now we assume all dynamic obstacles has the same dimension
        self.DYN_OBS_R_ = 0.06
        # Static obstacles (working areas)
        self._static_obs = []
        self._grid_resolution = workspace_resolution
        boundary_x_num = int(workspace_wh[0] / workspace_resolution)
        boundary_y_num = int(workspace_wh[1] / workspace_resolution)
        bottom_boundary = np.vstack((np.arange(start=workspace_o[0] + workspace_resolution / 2, 
            stop=workspace_o[0] + boundary_x_num * workspace_resolution, step = workspace_resolution ), 
            np.repeat(workspace_o[1], boundary_x_num))).T
        #print(f"Bottom {bottom_boundary}")
        top_boundary = bottom_boundary + np.vstack((np.zeros(boundary_x_num), np.repeat(workspace_wh[1], boundary_x_num))).T
        #print(f"Top {top_boundary}")
        left_boundary = np.vstack((np.repeat(workspace_o[0], boundary_y_num), np.arange(start=workspace_o[1] + workspace_resolution / 2, 
            stop=workspace_o[1] + boundary_y_num * workspace_resolution, step = workspace_resolution))).T
        #print(f"Left {left_boundary}")
        right_boundary = left_boundary + np.vstack((np.repeat(workspace_wh[0], boundary_y_num), np.zeros(boundary_y_num))).T
        #print(f"Right {right_boundary}")
        self._boundary_obs = np.concatenate((left_boundary, bottom_boundary, right_boundary, top_boundary), axis=0)
        # self._boundary_obs = np.concatenate((left_boundary, np.array([[1.5, 0]])), axis=0)
        # self._boundary_obs = np.concatenate((left_boundary, right_boundary), axis=0)
        #self._boundary_obs = np.array([[0, 0], [1.5, 0], [0, 0.7], [1.5, 0.7]])


    ## TODO: maybe only add a new obstacle when there is sufficient evidience of its existence?
    def add_or_update_robot_obs(self, obs_id: int, x: float, y: float, v: float) -> None:
        # TODO: maybe add a counter for each obstacle to determine when to remove it
        if obs_id not in self.obs_:
            self.obs_[obs_id] = RobotObstacle(obs_id, x, y, v)
        else:
            self.obs_[obs_id].update_state(x, y, v)


    # When should an obstacle be removed? After a number of frames where it is not detected?
    def remove_obs(self, obs_id: int) -> bool:
        if obs_id in self.obs_:
            del self.obs_[obs_id]
            return True
        else:
            print(f"Error: cannot remove obstacle {obs_id} as an obstacle with this id does not exist.")
            return False

    # def update_obs(self, obs_id, x, y) -> None:
    #     # Do we allow for updating the radius of an obstacle?
    #     if obs_id in self.obs_:
    #     else:
    #         print(f"Error: cannot update obstacle {obs_id} as it does not exist.")

    #def check_robot_hit(self, robot_x, robot_y, robot_id) -> bool:
        #for obs in self.obs_.values():
            #if robot_id != obs.id:
                ## Do not check with oneself
                #d = math.hypot(obs.x - robot_x, obs.y - robot_y) - obs.r_inf
                #if d < 0:
                    ## print(f"Error: already hitting obstacle {obs.id}")
                    #return True
        #return False

    #def get_dist_closest_obs(self, robot_x: float, robot_y: float, robot_id: float) -> float:
        ## Get the distance to the closest point on obstacles
        #closest_dist = float('inf')
        #for obs in self.obs_.values():
            #if robot_id != obs.id:
                ## Do not check with oneself
                #d = math.hypot(obs.x - robot_x, obs.y - robot_y) - obs.r_inf
                #if d > 0:
                    #if d < closest_dist:
                        #closest_dist = d
                #else:
                    #print(f"Warning: already hitting obstacle {obs.id}")
                    #return -1

        #return closest_dist

    def get_all_obs_positions_sizes(self, except_id: int) -> np.ndarray:
        return np.array([[obs.x, obs.y, obs.r_inf] for obs in self.obs_.values() if obs.id != except_id])

    def get_all_moving_obs_positions_sizes(self, except_id: int) -> np.ndarray:
        return np.array([[obs.x, obs.y, obs.r_inf] for obs in self.obs_.values() if obs.id != except_id and not obs.get_static()])

    def get_discrete_static_robot_obs(self) -> Tuple[List[float], List[float]]:
        static_robot_obs = [obs for obs in self.obs_.values() if obs.get_static()]
        # print(f"Number of static robot obstacles is len {len(static_robot_obs)}")
        if len(static_robot_obs) > 0:
            discrete_robot_obs = np.vstack(tuple([o.get_discrete_obs(resolution=self._grid_resolution) for o in static_robot_obs]))
            return discrete_robot_obs[:,0].tolist(), discrete_robot_obs[:,1].tolist()
        else:
            return [], [] 
        

    #TODO: remove static object following similar logic?
    ## Static obstacles
    def update_static_obs(self, obs_id: int, x: float, y: float) -> None:
        # print(f"Try to update workspace tag {obs_id}")
        for sobs in self._static_obs:
            if sobs.check_id_match(obs_id):
                sobs.update(obs_id, x, y)
                # self._static_moved = True
                break

    # def check_static_moved(self) -> bool:
        # return self._static_moved

    def add_static_obs(self, obs_id1:int, obs_id2: int) -> None:
        self._static_obs.append(StaticObstacle(obs_id1, obs_id2))

    def get_all_static_obs(self) -> List[StaticObstacle]:
        """Get all initialized static obstacles

        Returns:
            List[StaticObstacle]: A list of initialized static obstacles
        """
        return [so for so in self._static_obs if so.check_initialized()]

    def get_static_obs(self, obs_id: int) -> Union[None, StaticObstacle]:
        static_obs_found = [so for so in self._static_obs if so.check_id_match(obs_id) and so.check_initialized()]
        return static_obs_found[0] if len(static_obs_found) > 0 else None

    def get_discrete_static_obs(self) -> Tuple[List[float], List[float]]: 
        oxy = self._boundary_obs
        workspace_obs = tuple([sobs.get_discrete_obs(self._grid_resolution) for sobs in self._static_obs if sobs.check_initialized()])
        if len(workspace_obs) > 0:
            oxy_workspace = np.concatenate(workspace_obs, axis=0)
            oxy =np.concatenate((self._boundary_obs, oxy_workspace), axis=0)
        # self._static_moved = False
        return oxy[:,0].tolist(), oxy[:,1].tolist()

    def check_on_any_obs(self, x: float, y: float) -> bool:
        robot_obs = self.obs_.values()
        on_robot_obs = [o.check_within(x, y) for o in robot_obs]
        on_static_obs = [o.check_within(x, y, expand_by=0.06) for o in self._static_obs]
        return any(on_robot_obs + on_static_obs)

    def check_all_static_obs_initialized(self) -> bool: 
        return all([o.check_initialized() for o in self._static_obs])