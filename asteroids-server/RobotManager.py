from os import stat
import socket
from Robot import Robot, RobotStatus
#import time
from typing import Callable, Dict, List, Tuple, Union
import json
import numpy as np
from RobotManagement.Redistribution import Redistribution
import copy

#Debug
from colorama import Fore

class RobotManager:
    def __init__(self, server_port: int, disengaged_track: np.ndarray) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('0.0.0.0', server_port))
        self._robots: Dict[int, Robot] = {}
        ## Store the ip address of each robot as a tuple
        ## TODO: listen to the ping message and register ip addresses
        self._robot_addr: Dict[int, str] = {
            1: "192.168.137.249", # robot 0
            4: "192.168.137.103", # robot 1
            5: "192.168.137.137", # robot 3
            8: "192.168.137.132", # robot 2
            6: "192.168.137.76", # robot 4
        }
        self._robot_recv_port = 5000
        self._redistribution_due = True # Always try to redistribute robots at the beginning
        self._redist_track = disengaged_track
        self._redist = Redistribution(self._redist_track)
        self._disengaged_bots = []
        self._engaged_bots = [] # Occupied/unoccupied bots
        self._retinue_bot = -1
        self._cameraperson_bot = -1
        self._pinned_bots = []
        self._spotlight_bot = -1
        self._bot_spotlight_changed_cb = None
        self._robot_ground_z = 0.19
        self._bot_users_before_spotlight = {}
        self._known_users = []

        self._spotlight_loc = False
        self._spotlight_cam = False

    def __open_to_user_input(self) -> bool:
        return (not self._spotlight_cam) and (not self._spotlight_loc)

    def toggle_spotlight_loc(self, on: bool) -> None:
        self._spotlight_loc = on

    def check_spotlight_loc(self) -> bool:
        return self._spotlight_loc

    def check_under_token(self, bot_id) -> bool:
        return bot_id == self._spotlight_bot or bot_id in self._pinned_bots

    def check_new_robot(self, marker_id) -> bool:
        return marker_id not in self._robots

    def get_cameraperson_bot(self) -> int:
        return self._cameraperson_bot

    def get_retinue_bot(self) -> int:
        return self._retinue_bot

    def set_cameraperson_bot(self, camperson_bot_id: int) -> None:
        self._cameraperson_bot  = camperson_bot_id

    def set_retinue_bot(self, retinue_bot_id: int) -> None:
        self._retinue_bot = retinue_bot_id

    def recruit_bot(self, goto: List[float]) -> int:
        """ Find a robot from the disengaged robot

        Returns:
            int: Id of the bot found, -1 if nothing found
        """
        ## Only for debug
        #if 8 in self._disengaged_bots:
            #self._disengaged_bots.remove(8)
            #return 8

        disengaged = self.get_all_disengaged_robot()
        disengaged_pos = [bot.get_pos() for bot in disengaged]
        dist_to_goto = np.array([np.hypot(goto[0] - bot_pos[0], goto[1] - bot_pos[1]) for bot_pos in disengaged_pos])
        ## Find the closest one to the goto position
        if len(dist_to_goto) > 0:
            closest_ind = np.argmin(dist_to_goto)
            ## TODO: Would obstacle be a huge problem? What if there is no valid path?
            recruited_bot_id = disengaged[closest_ind].get_id()
            ## Recruited bot is no longer considered disengaged
            self._disengaged_bots.remove(recruited_bot_id)
            return recruited_bot_id
        else:
            return -1

    def get_spotlight_bot(self) -> int:
        return self._spotlight_bot

    def set_spotlight_bot(self, spotlight_bot_id: int) -> None:
        if not self.check_new_robot(spotlight_bot_id):
            ## If not engaged, make it an engaged bot
            if spotlight_bot_id in self._disengaged_bots:
                self._disengaged_bots.remove(spotlight_bot_id)
            if spotlight_bot_id not in self._engaged_bots:
                self._engaged_bots.append(spotlight_bot_id)

            self._spotlight_bot = spotlight_bot_id

            ## Store the users on all bots before transering all users to the spotlighted bot
            # self._bot_users_before_spotlight = copy.deepcopy({bot_id: self._robots[bot_id].get_users() for bot_id in self._robots})
            ## Add all user to the spotlight bot
            for user in self._known_users:
                self.add_user_to_robot(spotlight_bot_id, user)

            self.__set_bot_status(spotlight_bot_id, RobotStatus.PINNED)
            if self._bot_spotlight_changed_cb is not None:
                self._bot_spotlight_changed_cb(spotlight_bot_id, True)

            self._spotlight_cam = True

    def unspotlight_bot(self, spotlight_bot_id: int) -> None:
        if self._spotlight_bot == spotlight_bot_id:
            self.__set_bot_status(spotlight_bot_id, RobotStatus.STANDBY)
            self._spotlight_bot = -1
            #for bot_id in self._bot_users_before_spotlight:
                #users_on_bot = self._bot_users_before_spotlight[bot_id]
                ## print(f"Bot {bot_id} user {users_on_bot}")
                #for user in users_on_bot:
                    #self.add_user_to_robot(bot_id, user)
                    #print(f"Add user {user} to {bot_id}")
            if self._bot_spotlight_changed_cb is  not None:
                self._bot_spotlight_changed_cb(spotlight_bot_id, False)

            self._spotlight_cam = False

    def set_bot_spotlight_changed_cb(self, spotlight_cb: Callable[[int, bool], None]):
        self._bot_spotlight_changed_cb = spotlight_cb

    def get_pinned_robot(self) -> List[int]:
        return self._pinned_bots

    def pin_robot(self, robot_id: int) -> None:
        if robot_id not in self._pinned_bots:
            self._pinned_bots.append(robot_id)            
            self.__set_bot_status(robot_id=robot_id, status=RobotStatus.PINNED)

    def unpin_robot(self, robot_id: int) -> None:
        if robot_id in self._pinned_bots:
            self._pinned_bots.remove(robot_id)
            self.__set_bot_status(robot_id, RobotStatus.STANDBY)


    ## TODO: maybe only add a robot when there is sufficient evidence for its existence
    ## This should be checked in SensorManager or the main loop
    def add_robot(self, robot: Robot) -> bool:
        if self.check_new_robot(robot.get_id()):
            robot.set_reached_cb(self.__bot_reached_goal_cb)
            robot.set_engagement_changed_cb(self.__bot_engagement_changed_cb)
            self._robots[robot.get_id()] = robot
            self._disengaged_bots.append(robot.get_id())
            return True
        else:
            print(f"Error in RobotManager/add_robot, robot {robot.get_id()} already added.")
            return False

    def get_redistribution_due(self) -> bool:
        return self._redistribution_due

    def redistribute_disengaged_bots(self):
        disengaged_bots = self.get_all_disengaged_robot()
        if len(disengaged_bots) > 0:
            disengaged_bots_pos = np.array([bot.get_pos() for bot in disengaged_bots])
            redist_pos = self._redist.redistribute(len(disengaged_bots))
            assign_inds = self._redist.assign(disengaged_bots_pos, redist_pos)

            for i in range(0, len(disengaged_bots)):
                bot_goal = redist_pos[assign_inds[i]]
                ## Setting the goal will not guarantee the bots move along the edge of the table
                self.set_robot_goal(disengaged_bots_pos[i].get_id(), bot_goal[0], bot_goal[1], -np.pi / 2)
        self._redistribution_due = False

    def register_robot_addr(self, robot_id: int, robot_ip: str) -> None:
        self._robot_addr[robot_id] = robot_ip

    ## Do we ever remove a robot?
    def remove_robot(self, marker_id: int) -> bool:
        return True

    def get_all_robots(self) -> List[Robot]:
        return list(self._robots.values())

    def get_all_engaged_robot(self) -> List[Robot]:
        #return list(filter(lambda r: r.check_engaged(), self._robots.values()))
        return [self._robots[engaged_id] for engaged_id in self._engaged_bots if not self.check_new_robot(engaged_id)]

    def get_all_disengaged_robot(self) -> List[Robot]:
        #return list(filter(lambda r: not r.check_engaged(), self._robots.values()))
        return [self._robots[disengaged_id] for disengaged_id in self._disengaged_bots if not self.check_new_robot(disengaged_id)]

    def get_all_unmanaged_robot(self) -> List[Robot]:
        return self.get_all_disengaged_robot() + self.get_all_engaged_robot()

    def get_all_unoccupied_robots(self) -> List[Robot]:
        return [r for r in self.get_all_unmanaged_robot() if len(r.get_users()) == 0]

    def get_robot_v(self, robot_id: int) ->  float:
        return self._robots[robot_id].get_v()


    @staticmethod
    def __raised_up(prev_z: float, z: float, ground_z: float) -> bool:
        # print(f"Testing pick up {prev_z} and {z}")
        return prev_z != 0 and prev_z < 1.3 * ground_z and z > 1.3 * ground_z 

    @staticmethod
    def __put_down(prev_z: float, z: float, ground_z: float) -> bool:
        # print(f"Testing put down {prev_z} and {z}")
        return prev_z > 1.2 * ground_z and z < 1.2 * ground_z 

    @staticmethod
    def __in_air(z: float, ground_z: float) -> bool:
        return z > 2 * ground_z

    @staticmethod
    def __on_ground(z: float, ground_z: float) -> bool:
        return z < 1.2 * ground_z

    def update_robot_state(self, robot_id: int, x: float, y: float, z: float, h: float, ts: float) -> None:
        if not self.check_new_robot(robot_id):
            self._robots[robot_id].update_robot_state(x, y, h, ts, False)
            self._robots[robot_id].update_pos_z(z)
            #if RobotManager.__raised_up(prev_z, z, self._robot_ground_z):
            # if RobotManager.__in_air(z, self._robot_ground_z):
                ## The robot has been picked up
                # print(f"High z value {z}")
                # self.__on_robot_raised_up(robot_id)
            #elif RobotManager.__put_down(prev_z, z, self._robot_ground_z):
            # elif RobotManager.__on_ground(z, self._robot_ground_z):
                # self.__on_robot_put_down(robot_id)
        
    def guess_robot_state(self, robot_id: int, ts: float) -> None:
        if not self.check_new_robot(robot_id):
            self._robots[robot_id].update_robot_state(-1, -1, -1, ts, True)

    def __on_robot_raised_up(self, robot_id: int) -> None:
        ## Change robot status
        if not self.check_new_robot(robot_id):
            in_air = self._robots[robot_id].pick_up_bot()
            if in_air:
                print(Fore.RED + f"Picked up {robot_id}" + Fore.RESET)
                if self.bot_is_autonav(robot_id) or self.bot_is_rc(robot_id):
                    self.__set_bot_status(robot_id, RobotStatus.STANDBY)
                ## Remove the bot from engaged or disengaged set
                if robot_id in self._disengaged_bots:
                    self._disengaged_bots.remove(robot_id)
                if robot_id in self._engaged_bots:
                    self._engaged_bots.remove(robot_id)

    def __on_robot_put_down(self, robot_id: int) -> None:
        if not self.check_new_robot(robot_id):
            bot = self._robots[robot_id]
            if bot.put_down_bot():
                print(Fore.RED + f"Put down {robot_id}" + Fore.RESET)
                if robot_id == self._retinue_bot:
                    ## No more retinue bot
                    self._retinue_bot = -1
                if robot_id not in self._engaged_bots:
                    self._engaged_bots.append(robot_id)
                if robot_id in self._disengaged_bots:
                    self._disengaged_bots.remove(robot_id)

    def add_user_to_robot(self, robot_id: int, username: str):
        if username not in self._known_users:
            self._known_users.append(username)
        if not self.check_new_robot(robot_id) and (robot_id in self._engaged_bots or robot_id in self._disengaged_bots):
            # remove user from what they previouly control
            for r in self._robots.values():
                if r.get_id() != robot_id:
                    r.remove_user(username)
                    # self.send_led(r.get_id(), min(0.5, len(r.get_users()) / 12.0))
            # print(f"Adding user {username} to robot {robot_id}")
            self._robots[robot_id].add_user(username)

            # self.send_led(robot_id, min(0.5, len(self._robots[robot_id].get_users()) / 12.0))

            ## if the robot is disengaged, add it to the engaged list
            if robot_id in self._disengaged_bots:
                self._disengaged_bots.remove(robot_id)
            if robot_id not in self._engaged_bots:
                self._engaged_bots.append(robot_id)

    def user_has_control(self, robot_id: int, username: str):
        if not self.check_new_robot(robot_id):
            return self._robots[robot_id].get_owner() == username
        return False

    def which_user_control(self, username: str) -> Union[int, None]:
        for r in self._robots.values():
            if r.get_owner() == username:
                return r.get_id()

        return None

    def get_closest_valid_robot(self, pos: np.ndarray, username: str) -> Union[int, None]:
        ## Valid robots include all unoccupied robot plus the one that the user currently controls, if there's one
        user_controlling = self.which_user_control(username)
        user_controlling_robot = self._robots[user_controlling] if user_controlling is not None else None
        ## Do not use demonstrator or system managed robots
        valid_robots = [self._robots[bot_id] for bot_id in (self._engaged_bots + self._disengaged_bots) 
            if ((len(self._robots[bot_id].get_users()) == 0) and (not self._robots[bot_id].check_pinned())) ]
        if user_controlling_robot is not None:
            valid_robots = [user_controlling_robot] + valid_robots 
        ## TODO: use a* path length instead of Euclidean distance?
        if len(valid_robots) > 0:
            dist_pos = np.array([np.linalg.norm(pos - robot.get_pos()) for robot in valid_robots])
            closest_ind = np.argmin(dist_pos)
            return valid_robots[closest_ind].get_id()
        else:
            return None

    def bot_is_standby(self, robot_id) -> bool:
        if not self.check_new_robot(robot_id):
            return self._robots[robot_id].check_standby()
        return False

    def bot_is_rc(self, robot_id) -> bool:
        if not self.check_new_robot(robot_id):
            return self._robots[robot_id].check_rc()
        return False

    def bot_is_autonav(self, robot_id) -> bool:
        if not self.check_new_robot(robot_id):
            return self._robots[robot_id].check_autonav()
        return False

    def bot_is_pinned(self, robot_id) -> bool:
        if not self.check_new_robot(robot_id):
            return self._robots[robot_id].check_pinned()
        return False

    def __bot_reached_goal_cb(self, robot_id: int) -> None:
        self.__set_bot_status(robot_id=robot_id, status=RobotStatus.STANDBY)
        # self.__send_standby_status(robot_id=robot_id)

    def __bot_engagement_changed_cb(self, engagement_status: bool) -> None:
        self._redistribution_due = True

    def __set_bot_status(self, robot_id, status: RobotStatus) -> None:
        if not self.check_new_robot(robot_id):
            self._robots[robot_id].set_status(status)
            ## Sync status with bot
            if status == RobotStatus.STANDBY:
                self.__send_bot_status(robot_id=robot_id, status='standby')
            elif status == RobotStatus.PINNED:
                self.__send_bot_status(robot_id=robot_id, status='pinned')

    def choose_viable_goal_for_bot(self, robot_id: int, candidate_goals: np.ndarray) -> np.ndarray:
        if not self.check_new_robot(robot_id):
            bot = self._robots[robot_id]
            return bot.find_viable_target(candidate_goals)
        else:
            return np.array([])

    def set_robot_goal(self, robot_id, gx, gy, gh) -> None:
        if not self.check_new_robot(robot_id):
            bot = self._robots[robot_id]
            ## Keep the same heading if gh is None
            if not self._spotlight_loc:
                bot.set_target(gx, gy, gh if gh is not None else bot.get_heading())
            else:
                bot.just_go(gx, gy, gh)

    def rc_robot(self, robot_id: int, cmd: str) -> None:
        if (not self.check_new_robot(robot_id)) and (not self.bot_is_pinned(robot_id)):
            ## Set robot states
            if not self._robots[robot_id].check_rc():
                self._robots[robot_id].set_status(RobotStatus.RC)
            self.send_rc(robot_id, cmd)
    # def send_to_robot(self, robot_id, j_obj) -> bool:

    def send_vel_cmd(self, robot_id) -> None:
        if robot_id in self._robot_addr:
            ## Only send if the robot address have been registered
            v_track, theta_track = self._robots[robot_id].run_local_planner()
            # wheel_l, wheel_r = 0, 0
            vel_cmd = json.dumps({'id': float(robot_id), 'type': 'track', 'payload': [v_track, theta_track]})
            ## TODO: what are potential problems since now the id is a float?
            # print(f"Sending {v_track} and {theta_track} to robot {robot_id}")
            self._sock.sendto(vel_cmd.encode('utf-8'), (self._robot_addr[robot_id], self._robot_recv_port))
        else:
            #print(f"Error in RobotManager/send_vel_cmd, ip address is unknown for robot {robot_id}.")
            pass

    def send_led(self, robot_id: int, led_brightness_level: float) -> None:
        if robot_id in self._robot_addr:
            rc_msg = json.dumps({'type': 'led', 'payload': led_brightness_level})
            self._sock.sendto(rc_msg.encode('utf-8'), (self._robot_addr[robot_id], self._robot_recv_port))

    def send_rc(self, robot_id: int, cmd: str) -> None:
        if robot_id in self._robot_addr:
            rc_msg = json.dumps({'type': 'rc', 'payload': cmd})
            self._sock.sendto(rc_msg.encode('utf-8'), (self._robot_addr[robot_id], self._robot_recv_port))

    def send_curr_vel(self, robot_id) -> None:
        if robot_id in self._robot_addr:
            robot = self._robots[robot_id]
            curr_vel = json.dumps({'id': float(robot_id), 'type': 'vel', 'payload': [robot.get_v(), robot.get_omega()]})
            self._sock.sendto(curr_vel.encode('utf-8'), (self._robot_addr[robot_id], self._robot_recv_port))
        else:
            #print(f"Error in RobotManager/send_vel_cmd, ip address is unknown for robot {robot_id}.")
            pass

    def send_curr_v_theta(self, robot_id) -> None:
        if robot_id in self._robot_addr:
            robot = self._robots[robot_id]
            curr_vt = json.dumps({'id': float(robot_id), 'type': 'vtheta', 'payload': [robot.get_v(), robot.get_heading()]})
            self._sock.sendto(curr_vt.encode('utf-8'), (self._robot_addr[robot_id], self._robot_recv_port))
        else:
            #print(f"Error in RobotManager/send_vel_cmd, ip address is unknown for robot {robot_id}.")
            pass

    def __send_bot_status(self, robot_id: int, status: str) -> None:
        if robot_id in self._robot_addr:
            status_msg = json.dumps({'id': float(robot_id), 'type': 'status', 'payload': status})
            self._sock.sendto(status_msg.encode('utf-8'), (self._robot_addr[robot_id], self._robot_recv_port))

    def close(self) -> None:
        for bot in self._robots.values():
            bot.stop()
            