#!/usr/bin/env python
"""
.. module:: set_object_state
    :platform: Unix
    :synopsis: The Set Object State python script in second_assignment package

.. moduleauthor:: Tahmineh Tabarestani <Tahmine.tbi@gmail.com>

Definition:

    Berifly this node has three functions that have been used inside the **smach.state**, the functions are:
        1. *battery_lvl:* Take an instance from battery level 
        2. *arm_movement:* Take an instance from robot manipulator movement
        3. *base_movement:* Take an instance from robot base movement

Services:
    /state/set_battery_level ---> uses SetBatteryLevel.srv

    /move_arm ---> uses move_arm service defined inside moveit pkg

    /state/set_base_movement_state ---> use SetBaseMovmentSate.srv

"""
import rospy
from second_assignment.srv import SetBatteryLevel, SetBaseMovementState
import sys
sys.path.append('/root/ros_ws/src/full_second_assignment/second_assignment/src')
#Import constant name defined to structure the architecture.
from utilities import architecture_name_mapper as anm
from std_srvs.srv import SetBool

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_SET_OBJECT_STATE

def battery_lvl(battery_level):
    """
    Service client function for ``/state/set_battery_level`` Update the current robot battery level
    stored in the ``robot_states`` node.

    Args:
        battery_level(int)
    """
    #Wait until a service becomes available.
    rospy.wait_for_service(anm.SERVER_SET_BATTERY_LEVEL)
    try:
        log_msg = f'Set current robot battery level to the `{anm.SERVER_SET_BATTERY_LEVEL}` node.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        #Calling service
        service = rospy.ServiceProxy(anm.SERVER_SET_BATTERY_LEVEL, SetBatteryLevel)
        service(battery_level)
    #In case service calling is fail
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current robot battery level: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

def arm_movement(arm_movement_state):
    """
        Service client function for ``/move_arm``. Updates the current robot arm movement state stored 
        in ``my_moveit`` node.

        Args:
            arm_movement_state(bool)
    """
    #Wait until a service becomes available.
    rospy.wait_for_service('/move_arm')
    try:
        log_msg = 'Set robot arm movement state to ' + str(arm_movement_state)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        #Calling service
        service = rospy.ServiceProxy('move_arm', SetBool)
        service(arm_movement_state)
    #In case service calling is fail
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current arm movement state: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

def base_movement(base_movement_state):
    """
        Service client function for ``/base_movement_state``. Updates the current robot base movement state stored 
        in ``robot_states`` node.

        Args:   
            base_movement_state(bool)
    """
    #Wait until a service becomes available.
    rospy.wait_for_service(anm.SERVER_SET_BASE_MOVEMENT_STATE)
    try:
        log_msg = 'Set robot base movement state to ' + str(base_movement_state)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        #Calling service
        service = rospy.ServiceProxy(anm.SERVER_SET_BASE_MOVEMENT_STATE, SetBaseMovementState)
        service(base_movement_state)  
    #In case service calling is fail
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current base movement state: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
