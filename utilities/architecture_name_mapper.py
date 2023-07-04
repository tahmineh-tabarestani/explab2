#!/usr/bin/env python
import rospy

# The name of the parameter to define the environment size.
PARAM_ENVIRONMENT_SIZE = 'config/environment_size'

# The name of parameter to set the initial robot position.
PARAM_INITIAL_POSE = 'state/initial_pose'
# ---------------------------------------------------------

# The name of the node representing the shared knowledge required for this scenario.
# The name of the robot state node.
NODE_ROBOT_STATE = 'robot_state'

# The name of the finite state machine node.
NODE_FINITE_STATE_MACHINE = 'finite_state_machine'

# The name of the set object state node.
NODE_SET_OBJECT_STATE = 'set_object_state'

# -------------------------------------------------

# The name of the server to get the current robot pose.
SERVER_GET_POSE = 'state/get_pose'

# The name of the server to get the current robot battery level.
SERVER_GET_BATTERY_LEVEL = 'state/get_battery_level'

# The name of the server to set the current robot battery level. 
SERVER_SET_BATTERY_LEVEL = 'state/set_battery_level'


# The name of the server to set the current robot base movement state. 
SERVER_SET_BASE_MOVEMENT_STATE = 'state/set_base_movement_state'

# The name of the server to get the current robot base movement state. 
SERVER_GET_BASE_MOVEMENT_STATE = 'state/get_base_movement_state'


# -------------------------------------------------

# Function used to label each log with a producer tag.
def tag_log(msg, producer_tag):
    return '@%s>> %s' % (producer_tag, msg)
