#!/usr/bin/env python
"""
.. module:: finite_state_machine
    :platform: Unix
    :synopsis: The Finite State Machine python script in second_assignment package

.. moduleauthor:: Tahmineh Tabarestani <Tahmine.tbi@gmail.com>
Definiton:

    Breifly, finite_state_machine node define 5 task-level architecture for state machine in order to make robot abile to:

        1. *SETUP_MAP:* Detecte marker ID's use ``my_moveit.cpp`` node to move manipulator and add detected room's id into the list of the rooms
        2. *GO_IN_ROOM:* Patrolling the robot in map autonomously using ``move_base`` package and Finding target room via ontology map and control battery level using helper script ``set_object_state.py``.
        3. *INSPECT_ROOM:* Udate ontology map using helper script ``build_ontology_map.py`` and inspect room by camera on manipulator
        4. *GO_TO_CHARGER:* In case battery level lower than thershold
        5. *WAIT_FOR_CHARGING:* After a while machine state changes to the *GO_IN_ROOM*.

Subscribes to:
    /image_id ---> to get rooms information 

Services:

    /room_info ---> uses RoomInformation.srv

    /state/get_pose ---> uses GetPose.srv

 SimpleActionClient:
    /move_base ---> uses MoveBaseAction
  
"""
import rospy
#Smach library for smach state machine
import smach
import smach_ros
import math
import actionlib
#Move base library
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import sys
sys.path.append('/root/ros_ws/src/full_second_assignment/second_assignment/src')
#Import constant name defined to structure the architecture.
from utilities import architecture_name_mapper as anm
#Helper script to update the ontologycal map
from scripts.build_ontology_map import Build_Ontology_Map
#Helper script to set object state in boolean variable
from scripts.set_object_state import arm_movement,base_movement,battery_lvl
from second_assignment.msg import Point
from second_assignment.srv import RoomInformation,GetPose
from std_msgs.msg import Int32
#Colorful msg in output
from colorama import Fore
# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_FINITE_STATE_MACHINE
LOOP_TIME = 1

#Define variabile 
OM = None
mutex = None
#Define empty list
rooms_id = []
rooms_name = []
rooms_center = []
rooms_connections = []
rooms_doors = []

def room_data(room_id):
    """
        room_data function call service **RoomInformation** to get information of each room, use *room_info*.
        Args: 
            room_id(int)
        Returns:
            resp(RoomInformationResponse)
    """
    #Wait until a service becomes available.
    rospy.wait_for_service('room_info')
    try:
        #Calling service
        srv = rospy.ServiceProxy('room_info', RoomInformation)
        resp = srv(room_id)
        return resp 
    #In case service calling is fail
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def room_position(room):
    """
    Use previous function ``room_data`` to find out center of each room use *room_pose* *x* and *y* position.

    Args:
        room(string)
    Returns:
        room_pose(Point)
    """
    global rooms_name
    global rooms_center
    room_pose = Point()
    room_index = rooms_name.index(room)
    room_pose.x = rooms_center[room_index][0]
    room_pose.y = rooms_center[room_index][1]
    return room_pose

def marker_cb(data):
    """
        Marker CallBack function subscribe to ``image_id`` topic. Each time an image ID is obtained 
        through *robot_camera/image_box* it checks if the ID detected already exists or is significant and
        saves it in the following form:
            1. Save value of ``room_id`` detected (the value must be between 11 and 17 otherwise it is an error)
            2. Save name of room detected (i.e. #E, #R1, #R2, #R3, #R4, #C1, #C2)
            3. Save numer of room detected (there are max 7 IDs)
            4. Save ``room_data`` in ``room_info`` list to find out the connections between room, the door assigned to each room,
                and last visited time for each room
        Data about each room updates in the helper scripts build_ontological_map.py
        Args:
            data(int32)
    """
    global rooms_id
    global rooms_name
    global rooms_center
    global rooms_connections
    global rooms_doors
    global room_info
    if data.data not in rooms_id and data.data > 10 and data.data < 18:
        rooms_id.append(data.data)
        log_msg = Fore.BLUE +'\nID detected: %d \x1b[0m' % (data.data)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        log_msg = Fore.LIGHTBLUE_EX + '\nNumber of IDs founded: %d \x1b[0m' % (len(rooms_id))
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        
        room_info = room_data(data.data)
        rooms_name.append(room_info.room)
        log_msg = Fore.CYAN + '\nRoom '+ room_info.room + ' detected and added into map\x1b[0m'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        OM.add_room(room_info.room)

        rooms_center.append([room_info.x, room_info.y])
        log_msg = Fore.LIGHTCYAN_EX + '\nCenter of room ' + room_info.room + ' is: [%f, %f]\x1b[0m' % (room_info.x, room_info.y)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        for i in range(len(room_info.connections)):
            rooms_connections.append(room_info.connections[i].connected_to)
            rooms_doors.append(room_info.connections[i].through_door)
            log_msg = Fore.YELLOW + '\nRoom ' + room_info.room + ' is connected to ' + room_info.connections[i].connected_to + ' through door' + room_info.connections[i].through_door + '\x1b[0m'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            OM.add_door(room_info.connections[i].through_door)
            OM.assign_doors_to_room(room_info.room, room_info.connections[i].through_door)

        OM.disjoint_individuals()
        OM.add_last_visit_time(room_info.room, str(room_info.visit_time))

def go_to(pose):
    """
        Use Simple Action Client to send goal in ``move_base`` node, and gets a target_pose as an argument and sends it as 
        **MoveBaseGoal.msg** to the action server.
        Args:
            pose(Point)
        Returns:
            result(MoveBaseResult.msg)
    """
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose.x
    goal.target_pose.pose.position.y = pose.y
    goal.target_pose.pose.orientation.w = 1.0
    client.wait_for_server()
    client.send_goal(goal)

def in_target(target_pose):
    """
    This function it's useful just to control if robot reach target postion. Use **GetPose** service to get current postion
    of robot and control in function of proximity threshold. In case robot reach target position state of this function
    change to *True*.
    Args:
        target_pose(Point)

    Returns:
        target_reached(Bool)
    """
    #Wait until a service becomes available.
    rospy.wait_for_service(anm.SERVER_GET_POSE)
    try:
        #Calling service
        service = rospy.ServiceProxy(anm.SERVER_GET_POSE, GetPose)
        response = service()
        pose = response.pose
        #Define proximity threshold for target point
        if math.sqrt((target_pose.x - pose.x)**2 + (target_pose.y - pose.y)**2) < 1:
            target_reached = True
        else:
            target_reached = False
        log_msg = 'target reached state: ' + str(target_reached)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        return target_reached
    #In case service calling is fail
    except rospy.ServiceException as e:
        log_msg = f'Server cannot get current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

class SETUP_MAP (smach.State):
    """
    SETUP_MAP is initial state of hierarchical state machines, this state starts to move robot manipulator use ``my_moveit.cpp`` node.
    In this node all the configuration needed to find the boxes with IDs in different position are defined. Also in this state
    use *arm_movement()* function defined in ``set_object_state.py`` node. This last one is useful to change the state of the manipulate *(True/False)*.
    When 7 ID's are detected the state machine goes in next state and manipulator change state to *False* and machine state changes to GO_IN_ROOM.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['rooms data are complete'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global rooms_id
        arm_movement(True)
        # Wait for stimulus from the other nodes of the architecture.
        while not rospy.is_shutdown():  
            mutex.acquire()
            try:
                if len(rooms_id) > 6:
                    return 'rooms data are complete'
            finally:
                mutex.release()
            rospy.sleep(LOOP_TIME)

class GO_IN_ROOM(smach.State):
    """
    GO_IN_ROOM state is useful for moving the robot between rooms and patrolling in map. This status uses the *update_ontology()*
    function defined in ``build_ontology_map.py`` node to fetch data related to *target_room* and *battery_state* of
    robot ,meanwhile updates the list of time visited rooms. The robot movement is controlled through *base_movement()* 
    function defined in ``set_object_state.py`` node. If robot reached target *base_movemet()* becomes *False* and machine state
    changes to INSPECT_ROOM. During patrolling, in case battery level of the robot is lower than thershold the *target_room* is cancel
    and machine state changes to GO_TO_CHARGER.

    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['in target', 'low battery'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global OM
        now = rospy.get_rostime()
        [target_room, battery_low] = OM.update_ontology(now)
        target_position = room_position(target_room)
        base_movement(True)
        go_to(target_position)
        # Wait for stimulus from the other nodes of the architecture.
        while not rospy.is_shutdown():  
            mutex.acquire()
            try:
                now = rospy.get_rostime()
                [target_room, battery_low] = OM.update_ontology(now)
                if battery_low:
                    return 'low battery'
                else:
                    target_reached = in_target(target_position)
                    if target_reached:
                        base_movement(False)
                        return 'in target'
            finally:
                mutex.release()
            rospy.sleep(LOOP_TIME)

class INSPECT_ROOM(smach.State):
    """
    This state starts working when robot reached center of *target_room*. The *arm_movement()* becomes *True* to detect the room.
    After complete inspection, state machine changes again to GO_IN_ROOM.
    N.B: The *base_movment()* is *False* in case robot base start to move, this cause due to high frequency of the base control.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['inspected room'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        # Wait for stimulus from the other nodes of the architecture.
        while not rospy.is_shutdown():  
            mutex.acquire()
            try:
                arm_movement(True)
                return 'inspected room'
            finally:
                mutex.release()

class GO_TO_CHARGER(smach.State):
    """
    This state is triggered as a result of the message received from GO_IN_ROOM state when battery is low.
    In this state *target_position* becomes room *#E* and *update_ontology()* function updated and room #E
    add as a first room in **URGENT CLASS**. if room #E is reached state machine changes to WAIT_FOR_CHARGING.
    
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['in target'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global OM
        now = rospy.get_rostime()
        OM.update_ontology(now)
        target_position = room_position('E')
        base_movement(True)
        go_to(target_position)
        # Wait for stimulus from the other nodes of the architecture.
        while not rospy.is_shutdown():  
            mutex.acquire()
            try:
                now = rospy.get_rostime()
                OM.update_ontology(now)
                target_reached = in_target(target_position)
                if target_reached:
                    base_movement(False)
                    return 'in target'              

            finally:
                mutex.release()
            rospy.sleep(LOOP_TIME)

class WAIT_FOR_CHARGING(smach.State):
    """
    When room #E is reached by robot, robot wait 5 sec to charge battery and update onology map and state machine 
    changes to GO_IN_ROOM and start again patrolling.
    In this state uses *battery_lvl()* function defined in ``set_object_state.py`` node to set battery value of the robot.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['charging its over'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global OM
        # Wait for stimulus from the other nodes of the architecture.
        while not rospy.is_shutdown():  
            mutex.acquire()
            try:
                now = rospy.get_rostime()
                OM.update_ontology(now)
                #Delay 
                rospy.sleep(5)
                # Set battery level after charging to 1500
                battery_lvl(1500)
                return 'charging its over'

            finally:
                mutex.release()
def main():
    """
    The main function to run finite state machine, add state to each contatiner, create and start the introspection server for visualization **smach_ros**, 
    initialises the node and takes an instance of ``Build_Ontology_map`` class in the time instance now, defines the subscriner to the ``/image_id`` topic.
    """
    # Initialise this node.
    global OM
    global mutex

    rospy.init_node(anm.NODE_FINITE_STATE_MACHINE, log_level=rospy.INFO)
    now = rospy.get_rostime()
    OM = Build_Ontology_Map(LOG_TAG, now)
    
    # Get or create a new mutex.
    if mutex is None:
        mutex = Lock()
    else:
        mutex = mutex

    # Subscribe image id to get rooms information
    rospy.Subscriber('/image_id', Int32, marker_cb)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])
    sm.userdata.sm_counter = 0



    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('SETUP_MAP', SETUP_MAP(), transitions={'rooms data are complete':'GO_IN_ROOM'})
        smach.StateMachine.add('GO_IN_ROOM', GO_IN_ROOM(), transitions={'low battery':'GO_TO_CHARGER','in target':'INSPECT_ROOM'})
        smach.StateMachine.add('GO_TO_CHARGER', GO_TO_CHARGER(), transitions={'in target':'WAIT_FOR_CHARGING'})
        smach.StateMachine.add('INSPECT_ROOM', INSPECT_ROOM(), transitions={'inspected room':'GO_IN_ROOM'})
        smach.StateMachine.add('WAIT_FOR_CHARGING', WAIT_FOR_CHARGING(), transitions={'charging its over':'GO_IN_ROOM'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/START')
    sis.start()

    # Execute the state machine
    sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
    
