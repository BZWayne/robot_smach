#!/usr/bin/env python

import rospy
#Define environment size parameter as a list [max_x, max_y] with x:[0, max_x) and y:[0, max_y)
PARAM_ENVIRONMENT_SIZE = 'config/environment_size'

#Define initial robot position parameter
PARAM_INITIAL_POSE = 'state/initial_pose'

#Define node for shared knowledge required for scenario
NODE_ROBOT_STATE = 'robot-state'

#Define server to get current robot pose
ROBOT_GET_POSE = 'state/get_pose'

#Define server to get current robot pose
ROBOT_SET_POSE = 'state/set_pose'

#Define server to get current robot battery level
ROBOT_GET_BATTERY_LEVEL = 'state/get_battery_level'

#Define server to set current robot battery level
ROBOT_SET_BATTERY_LEVEL = 'state/set_battery_level'

#Define server to set current robot base movement state
ROBOT_SET_BASE_MOVEMENT_STATE = 'state/set_base_movement_state'

#Define server to get current robot base movement state
ROBOT_GET_BASE_MOVEMENT_STATE = 'state/get_base_movement_state'

#Define finite state machine node
NODE_FINITE_STATE_MACHINE = 'finite_state_machine'

#Label each log with a producer tag
def tag_log(msg, producer_tag):
    return f'@{producer_tag}>> {msg}'
