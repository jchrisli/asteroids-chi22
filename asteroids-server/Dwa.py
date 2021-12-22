## Based on https://github.com/AtsushiSakai/PythonRobotics

"""
Mobile robot motion planning sample with Dynamic Window Approach
author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı
"""

from typing import List, Tuple, Any

from ObstacleMap import ObstacleMap
import math
from enum import Enum
from functools import partial

# import matplotlib.pyplot as plt
import numpy as np

import Util

# show_animation = True


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self, id: int):
        # robot parameter
        #self.true_max_speed = 0.25
        self.max_speed = 0.4  # [m/s]
        self.min_speed = -0.4  # [m/s] 
        self.max_yaw_rate = math.pi / 2  # [rad/s]
        self.max_accel = 1.0  # [m/ss]
        #self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.max_delta_yaw_rate = math.pi / 2 / 0.1  # [rad/ss]
        self.v_resolution = 0.02  # [m/s]
        self.yaw_rate_resolution = 8.0 / 180.0 * math.pi  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 1.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 1.0  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check

        self.robot_id = id
        self.xy_tolerance = 0.1
        self.theta_tolerance = 10.0 / 180 * np.pi
        self.stopped_max_vel = 0.05


    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


# config = Config()

class DWAPlanner:
    def __init__(self, config: Config, obs_map: ObstacleMap) -> None:
        self._config = config
        self._obm = obs_map

    @staticmethod
    def motion(x: Any, u: List[float], dt: float) -> List[float]:
        """
        motion model
        """

        x[2] += u[1] * dt
        x[2] = math.atan2(math.sin(x[2]), math.cos(x[2])) # this might be slow, we will see
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x


    def calc_dynamic_window(self, x: List[float]) -> List[float]:
        """
        calculation dynamic window based on current state x
        """

        # Dynamic window from robot specification
        #Vs = [self._config.min_speed, self._config.max_speed,
        #    - self._config.max_yaw_rate, self._config.max_yaw_rate]
        # Need to clip v and omega since they can get completely wild TODO: fix this!
        v = Util.clip(x[3], self._config.min_speed, self._config.max_speed)
        #v = target_v
        omega = Util.clip(x[4], -self._config.max_yaw_rate, self._config.max_yaw_rate)

        
        # Dynamic window from motion model
        Vd = [v - self._config.max_accel * self._config.dt,
            v + self._config.max_accel * self._config.dt,
            omega - self._config.max_delta_yaw_rate * self._config.dt,
            omega + self._config.max_delta_yaw_rate * self._config.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        #dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
        #    max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        dw = [Util.clip(Vd[0], self._config.min_speed, self._config.max_speed),
            Util.clip(Vd[1], self._config.min_speed, self._config.max_speed),
            Util.clip(Vd[2], -self._config.max_yaw_rate, self._config.max_yaw_rate),
            Util.clip(Vd[3], -self._config.max_yaw_rate, self._config.max_yaw_rate)]
        # dw = [self._config.min_speed, self._config.max_speed, -self._config.max_yaw_rate, self._config.max_yaw_rate]

        return dw


    def predict_trajectory(self, x_init, v, y) -> np.ndarray:
        """
        predict trajectory with an input
        """

        x = np.array(x_init)
        trajectory = np.array(x)
        # time = 0
        nstep = int(self._config.predict_time / self._config.dt) + 1
        trajectory = np.zeros((nstep, len(x)))
        #while time <= self._config.predict_time:
        for i in range(0, nstep):
            x = DWAPlanner.motion(x, [v, y], self._config.dt)
            #trajectory = np.vstack((trajectory, x))
            #time += self._config.dt
            trajectory[i,:] = x

        return trajectory

    ## Bind x_init later 
    ## Returns a flattened trajectory. Each pose has 5 elements.
    def __predict_trajectory_numpy(self, vo, x_init):
        return self.predict_trajectory(x_init, vo[0], vo[1]).flatten()

    # Based on ROS navigation stack - base_local_planner - latched_stop_rotate_controller.cpp/stopWithAccLimits
    def stop_within_limit(self, x: List[float]) -> List[float]:
        v = x[3]
        theta = x[4]
        v_next = (1 if v > 0 else -1) * max(0.0, abs(v) - self._config.max_accel * self._config.dt)
        theta_next = (1 if theta > 0 else -1) * max(0.0, abs(theta) - self._config.max_delta_yaw_rate * self._config.dt)
        print(f"Trying to stop with linear {v_next} and angular {theta_next}")

        # Check if will hit things, generate a one-point trajectory
        # x_next = DWAPlanner.motion(x, [v_next, theta_next], self._config.dt)
        # if self.calc_obstacle_cost(np.array([x_next])) < float('Inf'): 
        #     # if does not hit things, we use the maximum acceleration
        #     return [v_next, theta_next]
        # else:
        #     # will hit things even if we slow down with
        #     # the max acc, just send 0 velocity command
        #     return [0, 0]
        return [0, 0]

    # Based on ROS navigation stack - base_local_planner - latched_stop_rotate_controller.cpp/rotateToGoal
    def rotate_inplace(self, x: List[float], gh: float) -> List[float]:
        theta_now = x[4] 
        ang_diff = Util.shortest_angular_dist(x[2], gh)
        theta_samp = min(self._config.max_yaw_rate, max(0, abs(ang_diff)))

        max_acc_theta = abs(theta_now) + self._config.max_delta_yaw_rate * self._config.dt
        min_acc_theta = abs(theta_now) - self._config.max_delta_yaw_rate * self._config.dt

        theta_samp = min(max(abs(theta_samp), min_acc_theta), max_acc_theta)

        max_theta_to_stop = math.sqrt(2 * self._config.max_delta_yaw_rate * abs(ang_diff))
        theta_samp = min(max_theta_to_stop, abs(theta_samp))
        theta_samp = min(self._config.max_yaw_rate, max(0, theta_samp))

        if ang_diff < 0:
            theta_samp = -theta_samp
        
        print(f"Trying to rotate with angular velocity {theta_samp}")

        # Do not need to check for collision now as we assume the robot is a cylinder
        #return [0, theta_samp]
        return [0, np.pi if ang_diff > 0 else -np.pi]


    def calc_control_and_trajectory(self, x, dw, goal) -> Tuple[List[float], np.ndarray]:
        """0
        calculation final input with dynamic window
        """

        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        ## Try to parallelize the computation using matrices
        v_vec = np.arange(dw[0], dw[1], self._config.v_resolution)
        y_vec = np.arange(dw[2], dw[3], self._config.yaw_rate_resolution)
        print(f"Dynamic window is {dw}, dim is {len(v_vec)}, {len(y_vec)}")
        ## These two should match in shape
        v_mat = np.tile(v_vec, (len(y_vec), 1))
        y_mat = np.tile(y_vec[:, np.newaxis], (1, len(v_vec)))
        sample_mat = np.stack((v_mat, y_mat), 2)

        ## Get the trajectory matrix
        ## This should return len(y_vec) * len(v_vec) * (n_steps * 5)
        print(f"sampling space dim is {sample_mat.shape}")
        trajectories = np.apply_along_axis(self.__predict_trajectory_numpy, axis=-1, arr=sample_mat, x_init=x_init)
        #print(f"Shape of trajectories is {trajectories.shape}")

        to_goal_cost = self._config.to_goal_cost_gain * np.apply_along_axis(DWAPlanner.calc_to_goal_cost, axis=-1, arr=trajectories, goal=goal)
        speed_cost = self._config.speed_cost_gain * np.apply_along_axis(lambda traj: self._config.max_speed - np.reshape(traj, (-1, 5))[-1, 3], axis=-1, arr=trajectories) # Prefer going forward
        ob_cost = self._config.obstacle_cost_gain * np.apply_along_axis(self.calc_obstacle_cost, axis=-1, arr=trajectories)

        final_cost = to_goal_cost + speed_cost + ob_cost

        ## Find the minimum cost 
        yv_ind = np.unravel_index(np.argmin(final_cost), final_cost.shape)
        y_val = y_vec[yv_ind[0]]
        v_val = v_vec[yv_ind[1]]
        best_u = [v_val, y_val]
        best_trajectory = np.reshape(trajectories[yv_ind[0], yv_ind[1], :], (-1, 5))
        if abs(best_u[0]) < self._config.robot_stuck_flag_cons \
                            and abs(x[3]) < self._config.robot_stuck_flag_cons:
            # to ensure the robot do not get stuck in
            # best v=0 m/s (in front of an obstacle) and
            # best omega=0 rad/s (heading to the goal with
            # angle difference of 0)
            best_u[1] = -self._config.max_delta_yaw_rate

        # evaluate all trajectory with sampled input in dynamic window
        # for v in np.arange(dw[0], dw[1], self._config.v_resolution):
        #     for y in np.arange(dw[2], dw[3], self._config.yaw_rate_resolution):

        #         trajectory = self.predict_trajectory(x_init, v, y)
        #         # calc cost
        #         to_goal_cost = self._config.to_goal_cost_gain * DWAPlanner.calc_to_goal_cost(trajectory, goal)
        #         speed_cost = self._config.speed_cost_gain * (self._config.max_speed - trajectory[-1, 3]) # Prefer going forward
        #         ob_cost = self._config.obstacle_cost_gain * self.calc_obstacle_cost(trajectory)

        #         final_cost = to_goal_cost + speed_cost + ob_cost

        #         # search minimum trajectory
        #         if min_cost >= final_cost:
        #             min_cost = final_cost
        #             best_u = [v, y]
        #             best_trajectory = trajectory
        #             if abs(best_u[0]) < self._config.robot_stuck_flag_cons \
        #                     and abs(x[3]) < self._config.robot_stuck_flag_cons:
        #                 # to ensure the robot do not get stuck in
        #                 # best v=0 m/s (in front of an obstacle) and
        #                 # best omega=0 rad/s (heading to the goal with
        #                 # angle difference of 0)
        #                 best_u[1] = -self._config.max_delta_yaw_rate
        return best_u, best_trajectory


    def calc_obstacle_cost(self, trajectory) -> float:
        """
        calc obstacle cost inf: collision
        """
        trajectory = np.reshape(trajectory, (-1, 5))
        ob = self._obm.get_all_obs_positions_sizes(except_id=self._config.robot_id)
        # print(f"Obstacles are {ob} for robot {self._config.robot_id}")
        if len(ob.shape) > 1:
            ox = ob[:, 0]
            oy = ob[:, 1]
            dx = trajectory[:, 0] - ox[:, None]
            dy = trajectory[:, 1] - oy[:, None]
            # TODO: should use obstacle size rather than robot size for one of the terms
            r = np.hypot(dx, dy) - self._config.robot_radius - self._config.robot_radius

            if self._config.robot_type == RobotType.rectangle:
                yaw = trajectory[:, 2]
                rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
                rot = np.transpose(rot, [2, 0, 1])
                local_ob = ob[:, None] - trajectory[:, 0:2]
                local_ob = local_ob.reshape(-1, local_ob.shape[-1])
                local_ob = np.array([local_ob @ x for x in rot])
                local_ob = local_ob.reshape(-1, local_ob.shape[-1])
                upper_check = local_ob[:, 0] <= self._config.robot_length / 2
                right_check = local_ob[:, 1] <= self._config.robot_width / 2
                bottom_check = local_ob[:, 0] >= -self._config.robot_length / 2
                left_check = local_ob[:, 1] >= -self._config.robot_width / 2
                if (np.logical_and(np.logical_and(upper_check, right_check),
                                np.logical_and(bottom_check, left_check))).any():
                    return float("Inf")
            elif self._config.robot_type == RobotType.circle:
                if np.array(r <= 0).any():
                    return float("Inf")

            min_r = np.min(r)
        else:
            min_r = float('Inf')
        return self._config.robot_radius * 0.3 / min_r  # OK

    ## The return range of this function is [0, 3.1415...)
    @staticmethod
    def calc_to_goal_cost(trajectory, goal):
        """
            calc to goal cost with angle difference
        """
        trajectory = np.reshape(trajectory, (-1, 5))

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost

    def dwa_control(self, x, goal):
        """
        Dynamic Window Approach control
        """
        dist_goal = math.hypot(goal[0] - x[0], goal[1] - x[1])
        # tv = 0.35 if dist_goal > 0.2 else 0.3

        dw = self.calc_dynamic_window(x)

        u, trajectory = self.calc_control_and_trajectory(x, dw, goal)

        return u, trajectory


# def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
#     plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
#               head_length=width, head_width=width)
#     plt.plot(x, y)


# def plot_robot(x, y, yaw, config):  # pragma: no cover
#     if config.robot_type == RobotType.rectangle:
#         outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
#                              (config.robot_length / 2), -config.robot_length / 2,
#                              -config.robot_length / 2],
#                             [config.robot_width / 2, config.robot_width / 2,
#                              - config.robot_width / 2, -config.robot_width / 2,
#                              config.robot_width / 2]])
#         Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
#                          [-math.sin(yaw), math.cos(yaw)]])
#         outline = (outline.T.dot(Rot1)).T
#         outline[0, :] += x
#         outline[1, :] += y
#         plt.plot(np.array(outline[0, :]).flatten(),
#                  np.array(outline[1, :]).flatten(), "-k")
#     elif config.robot_type == RobotType.circle:
#         circle = plt.Circle((x, y), config.robot_radius, color="b")
#         plt.gcf().gca().add_artist(circle)
#         out_x, out_y = (np.array([x, y]) +
#                         np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
#         plt.plot([x, out_x], [y, out_y], "-k")


    # Format of x -> [position_x, position_y, heading_angle, velocity, angular_velocity]
    def goto(self, x: List[float], gx: float, gy: float, gh: float) -> List[float]:
        # print(__file__ + " start!!")
        # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        # x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
        # goal position [x(m), y(m)]
        goal = np.array([gx, gy, gh])

        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        # print(f'Distance to {gx} {gy} is {dist_to_goal}')
        stopped = abs(x[3]) < self._config.stopped_max_vel
        #if dist_to_goal <= self._config.robot_radius:
        u = [0.0, 0.0]
        if dist_to_goal <= self._config.xy_tolerance:
            # Stop
            # if the robot has not stopped yet, stop it within acceleration limits first
            if not stopped:
                print("Stopping...")
                u = self.stop_within_limit(x)
            # elif abs(Util.shortest_angular_dist(x[2], gh)) > self._config.theta_tolerance:
            #     print("Rotating...")
            #     u = self.rotate_inplace(x, gh)
        else:
            u, predicted_trajectory = self.dwa_control(x, goal)
        return u
        # return [0.3, 0.0]


        # config.robot_type = robot_type
        # trajectory = np.array(x)
        # ob = self._ob.get
        # input [forward speed, yaw_rate]
        # x = motion(x, u, self._config.dt)  # simulate robot
        # trajectory = np.vstack((trajectory, x))  # store state history

        # TODO: have a dedicated visualization environment ?
        # if show_animation:
        #     plt.cla()
        #     # for stopping simulation with the esc key.
        #     plt.gcf().canvas.mpl_connect(
        #         'key_release_event',
        #         lambda event: [exit(0) if event.key == 'escape' else None])
        #     plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
        #     plt.plot(x[0], x[1], "xr")
        #     plt.plot(goal[0], goal[1], "xb")
        #     plt.plot(ob[:, 0], ob[:, 1], "ok")
        #     plot_robot(x[0], x[1], x[2], config)
        #     plot_arrow(x[0], x[1], x[2])
        #     plt.axis("equal")
        #     plt.grid(True)
        #     plt.pause(0.0001)

        # check reaching goal

        # print("Done")
        # if show_animation:
        #     plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        #     plt.pause(0.0001)

        # plt.show()


# if __name__ == '__main__':
#    main(robot_type=RobotType.rectangle)
    # main(robot_type=RobotType.circle)