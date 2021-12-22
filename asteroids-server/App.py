from typing import List

from cv2 import FastFeatureDetector, waitKey
from astar import AStarPlanner
from Robot import Robot
from RobotManager import RobotManager
from ObstacleMap import ObstacleMap
from SensorManager import SensorManager
from interactions.CommandManager import CommandManager, MapCamera
from interactions.Tokens import Tokens, TokenType
from datetime import datetime
# import cv2
import time
from Person import Person
from RobotManagement.TableTrack import get_table_track
from Util import normalize_vec, get_rect_closest_side, project_to_rect_side, get_rect_opposite_side
from Study.DataLogger import DataLogger

import numpy as np

## For debugging only
from colorama import Fore

## Global configs
USE_CAMERAPERSON = False
USE_RETINUE = False
USE_LOG = True

## Tag numbers
WORLD_TAGS = [14, 13, 12]
#ROBOT_TAGS = [20, 22, 24, 25]
#PERSON_TAG = [21]
ROBOT_TAGS = [1, 5, 4, 6, 8]
PERSON_TAG = [10]
WORKSPACE_TAGS = [[28, 29], [30, 31]]
WORKSPACE_TAGS_FLAT = [28, 29, 30, 31]
SPOTLIGHT_TAG = 21
SPOTLIGHT_LOC_TAG = 22
PIN_TAGS = [23, 24, 26, 27]
TOKEN_TAGS = [SPOTLIGHT_TAG, SPOTLIGHT_LOC_TAG] + PIN_TAGS

## Update rates
LOCAL_PLANNER_RATE = 5 # local planner runs at 5Hz
GLOBAL_PLANNER_RATE = 0.2 # global planner runs every 5s, but will run at least once for every goal
MANAGED_BOTS_RATE = 1/5.0 # send commands to system-managed bots at x Hz
POSSESSED_BOTS_RATE = 1/2.0
LOGGING_RATE = 1.0
# RETINUE_BOT_RATE = 1/10.0

## Field
ASTAR_GRID_SIZE = 0.06
FIELD_PADDING = 0.06
FIELD_H = 1.0
FIELD_W = 1.52 - 2 * FIELD_PADDING
FIELD_ORIGIN = np.array([0, 0]) + np.array([FIELD_PADDING, FIELD_PADDING])
FIELD_CENTER = FIELD_ORIGIN + np.array([(FIELD_W - FIELD_PADDING) / 2, (FIELD_H - FIELD_PADDING) / 2])

## Data logging
dt_now = datetime.now()
INTERACTION_LOG_PATH = f"data/interaction-{dt_now.strftime('%m-%d-%H-%M-%S')}.csv"
MOTION_LOG_PATH = f"data/motion-{dt_now.strftime('%m-%d-%H-%M-%S')}.csv"
WORKSPACE_LOG_PATH = f"data/workspace-{dt_now.strftime('%m-%d-%H-%M-%S')}.csv"


def within_workspace(x: float, y: float, origin: List[float], w: float, h: float):
    return x < origin[0] + w and x > origin[0] and y < origin[1] + h and y > origin[0]

LOCATIONING_CAM_PARAM_PATH = 'c930-720p-calib.yaml'
MAP_CAM_PARAM_PATH = 'c922-720p-calib.json'

# Try to get a sensor update at target_fps
sensor_manager = SensorManager(LOCATIONING_CAM_PARAM_PATH, ROBOT_TAGS + PERSON_TAG, WORLD_TAGS, 
                            slowtags=WORKSPACE_TAGS_FLAT + TOKEN_TAGS, target_fps=50.0, camnum= 1) 
#save_res_counter = 0
#save_res_name = f"debugoutput/{dt_now.strftime('%H-%M-%S')}.mp4"
#res_vid = cv2.VideoWriter(save_res_name,cv2.VideoWriter_fourcc('M','J','P','G'), 30, (1920, 1080))
prev_local_plan_ts = -1
prev_global_update_ts = -1
prev_managed_update_ts = -1
retinue_update_count = 0
retinue_update_threshold = 2
prev_possessed_update_ts = -1
prev_log_ts = -1
# prev_retinue_update_ts = -1
# The table measures 1.52 by 0.76 m, leaving 0.06 m room on each side
# The discretization resolution must be smaller than the global planner resolution
obs_map = ObstacleMap(workspace_o = [0.0, 0.0], workspace_wh=[1.46, 1.0], workspace_resolution=0.05)
for tag_pair in WORKSPACE_TAGS:
    obs_map.add_static_obs(tag_pair[0], tag_pair[1])
# Global path planner, shared by all robots. The resolution used must be greater than 
astar_planner = AStarPlanner(resolution=ASTAR_GRID_SIZE, rr=0.06)
## Initialize the global planner with the workspace boundary
oxb, oyb = obs_map.get_discrete_static_obs()
astar_planner.update_obs(oxb, oyb)

track = get_table_track(origin=[0, 0], w=1.52, h=0.76, padding=0.2, numunit=40)
robot_manager = RobotManager(4399, disengaged_track=track)
debug_target_set = False

person = Person()
tokens = Tokens(spotlight_tag_id=SPOTLIGHT_TAG, spotlight_loc_tag_id=SPOTLIGHT_LOC_TAG, pin_tag_ids=PIN_TAGS)
prev_retinue_workspace_id = -1
spotlight_loc_found = False

## Data logging
logger = DataLogger(interaction_logs=INTERACTION_LOG_PATH,  robot_motion_logs=MOTION_LOG_PATH, 
                    workspace_logs=WORKSPACE_LOG_PATH) if USE_LOG else None

# Handle commands
map_cam = MapCamera(MAP_CAM_PARAM_PATH)
cmd_manager = CommandManager(robot_manager=robot_manager, mp_cam=map_cam, person=person, obstacles=obs_map, logger=logger)



while True:
    # if save_res_counter % 30 == 0:
    #     save_res_name = f"debugoutput/{save_res_counter}.jpg"
    # save_res_counter += 1
    try:
        success, res = sensor_manager.GetSensorReading(save_results=None)
        if success == 2:
            print("Cannot read camera frame")
            break
        elif success == 1:
            continue
        elif success == 0:
            #print(res, end='\r', flush=True)
            ts = time.time()
            local_plan_ready = (ts - prev_local_plan_ts) > (1 / LOCAL_PLANNER_RATE)
            should_global_map_update = (ts - prev_global_update_ts)  >  (1 / GLOBAL_PLANNER_RATE)
            should_managed_bot_update = (ts - prev_managed_update_ts) > (1 / MANAGED_BOTS_RATE)
            should_possessed_bot_update = (ts - prev_possessed_update_ts) > (1 / POSSESSED_BOTS_RATE)
            should_write_log = (ts - prev_log_ts) > (1 / LOGGING_RATE)
            # should_retinue_bot_update = (ts - prev_retinue_update_ts) > (1 / RETINUE_BOT_RATE)

            ## Tracking debug
            #tracking_total_frame += 1
            #if tracking_total_frame == tracking_reset:
                #print(Fore.CYAN + f"Tracking rate is {float(tracked_frame) / float(tracking_total_frame)}" + Fore.RESET)
                #tracking_total_frame = 0
                #tracked_frame = 0

            ## Update obstacles and add new robots
            obj_state_dict = {}
            spotlight_loc_found = False

            for obj_state in res:
                ## Update the states of all obstacles
                obj_id, obj_x, obj_y, obj_z, obj_heading = obj_state
                # print(f"Found tag {obj_id}")
                obj_state_dict[obj_id] = (obj_x, obj_y, obj_heading, obj_z)

                #if obj_id in ROBOT_TAGS:
                    ## TODO: decide if non-robot objects should also be added to local planning
                #    robot_obs_v = robot_manager.get_robot_v(obj_id)
                    #obs_map.add_or_update_obs(obj_id, obj_x, obj_y, 0.06, robot_obs_v)

                ## Add robots
                if robot_manager.check_new_robot(obj_id) and obj_id in ROBOT_TAGS:
                    robot = Robot(obj_id, obs_map, global_planner=astar_planner)
                    robot_manager.add_robot(robot)

                ## Update person info
                if obj_id in PERSON_TAG:
                    person.update_person_state([obj_x, obj_y], obj_heading)

                ## Update workspace 
                if obj_id in WORKSPACE_TAGS_FLAT:
                    obs_map.update_static_obs(obj_id, obj_x, obj_y)

                if obj_id in TOKEN_TAGS:
                    ## Update tag positions
                    tokens.set_token_pos(obj_id, obj_x, obj_y)
                    if obj_id == SPOTLIGHT_LOC_TAG:
                        spotlight_loc_found = True

            if not spotlight_loc_found:
                tokens.spotlight_loc_maybe_gone()

            should_spotlight_loc = tokens.check_spotlight_loc_token_presence()
            robot_manager.toggle_spotlight_loc(should_spotlight_loc)
            if should_spotlight_loc and should_possessed_bot_update:
                # print(f"Found spotlight loc token at {tokens.get_spotlight_loc_pos()}")
                for bot in robot_manager.get_all_unmanaged_robot():
                    curr_bot_pos = bot.get_pos()
                    lookat_spotlight_loc = normalize_vec(np.array(tokens.get_spotlight_loc_pos()) - curr_bot_pos)
                    lookat_spotlight_loc_angle = np.arctan2(lookat_spotlight_loc[1], lookat_spotlight_loc[0])
                    robot_manager.set_robot_goal(bot.get_id(), curr_bot_pos[0], curr_bot_pos[1], lookat_spotlight_loc_angle)

            if should_global_map_update:
                ox, oy = obs_map.get_discrete_static_obs()
                rox, roy = obs_map.get_discrete_static_robot_obs()
                # print(f"Number of static robot obstacles {len(rox)}")
                astar_planner.update_obs(ox + rox, oy + roy)
                cmd_manager.send_workspace_update()
                prev_global_update_ts = ts
                
            ## Go over existing robots
            for bot in robot_manager.get_all_robots():
                bot_id = bot.get_id()
                if bot_id in obj_state_dict.keys():
                    ## Update robot states if found
                    bot_pose = obj_state_dict[bot_id]
                    robot_manager.update_robot_state(bot_id, bot_pose[0], bot_pose[1], bot_pose[3], bot_pose[2], ts)

                    ## Turn off token effect if necessary
                    if robot_manager.check_under_token(bot_id):
                        if robot_manager.get_spotlight_bot() == bot_id:
                            robot_manager.unspotlight_bot(bot_id)
                            print(f"Make robot {bot_id} not spotlight anymore")
                        if bot_id in robot_manager.get_pinned_robot():
                            robot_manager.unpin_robot(bot_id)
                            print(f"Make robot {bot_id} not pinned anymore")

                    #robot_manager.send_curr_vel(bot_id)
                    if robot_manager.bot_is_autonav(robot_id=bot_id):
                        robot_manager.send_curr_v_theta(bot_id)
                    ## Update robot obstacles
                    obs_map.add_or_update_robot_obs(bot_id, bot_pose[0], bot_pose[1], robot_manager.get_robot_v(bot_id))
                elif not robot_manager.check_under_token(bot_id):
                    ## Bot not found, this could be due to lost tracking or robot removed (handle separately)
                    if bot.check_existence():
                        if robot_manager.bot_is_autonav(bot_id):
                            ## TODO: how often does it happens that the robot never reaches standby state?
                            robot_manager.guess_robot_state(bot_id, ts)
                    else:
                        ## Remove the bot?
                        ## Check if covered by command tokens?
                        last_known_pos = bot.get_pos().tolist()
                        token_id, token_type = tokens.search_for_tag(last_known_pos[0], last_known_pos[1])
                        print(f"{token_type} is found.")
                        if token_type == TokenType.SPOTLIGHT:
                            print(f"Make robot {bot_id} spotlight")
                            robot_manager.set_spotlight_bot(bot_id)
                        elif token_type == TokenType.PIN:
                            print(f"Pin robot {bot_id}")
                            robot_manager.pin_robot(robot_id=bot_id)

                ## Generate local plans (thus velocity commands) if it's the right time
                if local_plan_ready and robot_manager.bot_is_autonav(robot_id=bot_id):
                    robot_manager.send_vel_cmd(bot_id)
                    # pass

            if should_managed_bot_update and person.get_person_pos() is not None:
                # print(f"Managed robot update.")
                person_pos = np.array(person.get_person_pos())
                field_center =  FIELD_CENTER
                room = 0.1
                # person_fc_dir = normalize_vec(field_center - person_pos) 
                # intersections = intersect_line_rect((person_pos[0], person_pos[1]), 
                                                    # (field_center[0], field_center[1]), (FIELD_ORIGIN[0], FIELD_ORIGIN[1]), 
                                                    # FIELD_W, FIELD_H)
                if USE_RETINUE and obs_map.check_all_static_obs_initialized():
                    # retinue_bot_id = robot_manager.get_retinue_bot()
                    retinue_update_count += 1
                    # Update the retinue goal less frequently
                    if retinue_update_count >= retinue_update_threshold:
                        retinue_update_count = 0
                        unoccupied = robot_manager.get_all_unoccupied_robots()
                        unreachable = [not person.check_reachable(unoccu.get_pos()[0], unoccu.get_pos()[1]) and 
                                        (not robot_manager.bot_is_pinned(unoccu.get_id())) for unoccu in unoccupied]
                        if len(unoccupied) > 0 and any(unreachable):
                            unreachable_ind = unreachable.index(True)
                            unreachable_bot = unoccupied[unreachable_ind]
                            ## Find a target for this robot
                            candidate_positions = []
                            while len(candidate_positions) < 10:
                                candidate = person.sample_reachable_points(num_sample=1)
                                ## Check if on any obstacles
                                if (not obs_map.check_on_any_obs(candidate[0, 0], candidate[0, 1])) and \
                                    (within_workspace(candidate[0, 0], candidate[0, 1], FIELD_ORIGIN.tolist(), FIELD_W, FIELD_H)):
                                    candidate_positions.append(candidate.flatten())
                            retinue_pos = robot_manager.choose_viable_goal_for_bot(unreachable_bot.get_id(), np.array(candidate_positions))
                            if len(retinue_pos) > 0:
                                face_person_dir = normalize_vec(person_pos - retinue_pos)
                                robot_manager.set_robot_goal(unreachable_bot.get_id(), retinue_pos[0], retinue_pos[1], None)
                                print(Fore.YELLOW + f"Setting retinue robot {unreachable_bot.get_id()} goal {retinue_pos[0]} {retinue_pos[1]}" + Fore.RESET)
                            else:
                                print(Fore.YELLOW + f"Trying to set retinue goal for robot {unreachable_bot.get_id()} but cannot find a valid target")
                    # prev_retinue_workspace_id = relevant_workspace.get_id1()

                workspaces = obs_map.get_all_static_obs()
        
                if len(workspaces) > 0:
                    if USE_CAMERAPERSON:
                        camperson_bot_id = robot_manager.get_cameraperson_bot()
                        workspaces_to_person = list(map(lambda w: np.linalg.norm(np.array(w.get_center()) - person_pos), workspaces))
                        relevant_workspaces_ind = np.argmin(workspaces_to_person)
                        relevant_workspace = workspaces[relevant_workspaces_ind]
                        relevant_workspace_center = relevant_workspace.get_center()
                        #intersections_workspace_field = intersect_line_rect((relevant_workspace_center[0], relevant_workspace_center[1]), 
                        #                        (field_center[0], field_center[1]), (FIELD_ORIGIN[0], FIELD_ORIGIN[1]), 
                        #                        FIELD_W, FIELD_H)
                        ## Find out which side of the workspace the person is on 
                        person_side = get_rect_closest_side((person_pos[0], person_pos[1]), (FIELD_ORIGIN[0], FIELD_ORIGIN[1]), FIELD_W, FIELD_H)
                        project_to = get_rect_opposite_side(person_side)
                        cam_bot_pos = project_to_rect_side(relevant_workspace_center, (FIELD_ORIGIN[0], FIELD_ORIGIN[1]), FIELD_W, FIELD_H, project_to)

                        # if len(intersections_workspace_field) > 1:
                        if camperson_bot_id == -1:
                            found_id = robot_manager.recruit_bot(list(cam_bot_pos))
                            if found_id != -1:
                                robot_manager.set_cameraperson_bot(found_id)
                        else:
                            workspace_orientation = normalize_vec(np.array(cam_bot_pos) - np.array(relevant_workspace_center) )
                            # camperson_pos = intersections_workspace_field[1] - workspace_orientation * room
                            camperson_heading = -np.arctan2(workspace_orientation[1], workspace_orientation[0])
                            robot_manager.set_robot_goal(camperson_bot_id, cam_bot_pos[0], cam_bot_pos[1], camperson_heading)
                            print(Fore.GREEN + f"Setting camera robot goal {cam_bot_pos[0]} {cam_bot_pos[1]}" + Fore.RESET)
                prev_managed_update_ts = ts

            ## Log data if needed
            if USE_LOG and should_write_log and logger is not None:
                ## log all robots TODO: what about ones that are gone 
                bots_to_log = robot_manager.get_all_robots()
                for bot in bots_to_log:
                    bot_pos = bot.get_pos()
                    logger.write_motion_log(ts, bot.get_id(), bot_pos[0], bot_pos[1], bot.get_heading(), bot.get_users())

                ## log all workspaces
                workspaces = obs_map.get_all_static_obs()
                for w in workspaces:
                    x1, y1 = w.get_id1_pos()
                    x2, y2 = w.get_id2_pos()

                    logger.write_workspace_log(ts, w.get_id1(), x1, y1, x2, y2)

                prev_log_ts = ts

            if local_plan_ready:
                ## Send robot states to user interfaces
                cmd_manager.send_robot_update()

                ## Check if we should disengage robots. Engaging robots has been taken care of in CommandManager
                #for bot in robot_manager.get_all_engaged_robot():
                    #if bot.check_should_disengage(ts):
                        #bot.disengage()

                #if robot_manager.get_redistribution_due():
                    #robot_manager.redistribute_disengaged_bots()


                prev_local_plan_ts = ts
    except KeyboardInterrupt:
        ## clean things up
        sensor_manager.Close()
        robot_manager.close()
        cmd_manager.close()
        if logger is not None:
            logger.close()

sensor_manager.Close()
#res_vid.release()