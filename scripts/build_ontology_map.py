#!/usr/bin/env python
"""
.. module:: build_ontology_map.py
    :platform: Unix
    :synopsis: The Build Ontology Map script in second_assignment package

.. moduleauthor::Tahmineh Tabarestani  <Tahmine.tbi@gmail.com>

Definition:

    Defines the Ontologycal map using the ``armor_client.py`` helper script methods and the
    ontology file ``ontologycal_map.owl``.

Services:
    /state/get_battery_level  ---> uses GetBatteryLevel.srv
    
    /state/get_pose ---> uses GetPose.srv

"""

import rospy
from os.path import dirname, realpath
import sys
sys.path.append('/root/ros_ws/src/full_second_assignment/second_assignment/src')
#Import constant name defined to structure the architecture.
from utilities import architecture_name_mapper as anm
from second_assignment.srv import GetPose, GetBatteryLevel
from second_assignment.msg import Point
#Helper script from armor package
from utilities.armor_client import ArmorClient
#Colorful msg in output
from colorama import Fore

class Build_Ontology_Map:
    """
        Class for building the ontologycal map. Each time this class is called by ``finite_state_machine.py``
        ontology map loaded by using helper script ``armor_client.py`` in **armor package** and it make me able 
        to use different method available inside *armor pkg*. In case, if each method from ``armor`` it's used,
        it's necessary to  syncs the reasoner (PELLET).
    """
    def __init__(self, log_tag, init_time):
        self.log_tag = log_tag
        self.init_time = init_time
        #Build topological map
        self.path = dirname(realpath(__file__))
        self.path = self.path + "/../utilities/ontology/"
        self.client = ArmorClient("ontology", "ontology_reference")
        #Load resoner
        self.client.utils.load_ref_from_file(self.path + "ontological_map.owl", " ", False, "PELLET", False, False)
        self.client.utils.set_log_to_terminal(True)
        self.add_robot()

    def add_room(self, room):
        """
        Add indiviuals (ROOM) to class in ontologycal map --> using helper script ``armor_client.py``
        in **armor package** and syncs the reasoner (PELLET)

        Args:
            room(string)
        """
        self.client.manipulation.add_ind_to_class(room, "ROOM")
        self.client.utils.sync_buffered_reasoner()

    def add_door(self, door):
        """
        Add indiviuals (DOOR) to class in ontologycal map --> using helper script ``armor_client.py``
        in **armor package** and syncs the reasoner (PELLET)

        Args:
            door(string)
        """
        self.client.manipulation.add_ind_to_class(door, "DOOR")
        self.client.utils.sync_buffered_reasoner()  

    def disjoint_individuals(self):
        """
        Disjoint each indiviual (ROOM, DOOR) in different class in ontologycal map --> using helper script ``armor_client.py``
        in **armor package** and syncs the reasoner (PELLET)
        """
        self.client.manipulation.disj_inds_of_class("ROOM")
        self.client.manipulation.disj_inds_of_class("DOOR")
        self.client.utils.sync_buffered_reasoner()

    def assign_doors_to_room(self, room, doors):
        """
        Assign at least one DOOR to each ROOM --> using helper script ``armor_client.py``
        in **armor package** and syncs the reasoner (PELLET)

        Args:
            room(string)
            doors[](string)
        """
        for i in range(len(doors)):
            self.client.manipulation.add_objectprop_to_ind("hasDoor", room, doors[i])
        self.client.utils.sync_buffered_reasoner()

    def add_last_visit_time(self, room, visit_time):
        """
        Add time visited method for each ROOM in order to make URGENT list of the ROOM
        which are not visited for a long time --> using helper script ``armor_client.py``
        in **armor package** and syncs the reasoner (PELLET)

        Args: 
            room(string)
            visit_time(string)
        """
        self.client.manipulation.add_dataprop_to_ind("visitedAt", room, "Int", visit_time)
        self.client.utils.sync_buffered_reasoner()

    def add_robot(self):
        """
        Function to add robot in extra palce in ROOM E (where there are marker) for first time and 
        and sets its initial time instance and battery level and defines its urgency and battery threshold
        --> using helper script ``armor_client.py`` in **armor package** and syncs the reasoner (PELLET)
        """ 
        self.client.manipulation.add_ind_to_class("Robot", "ROBOT") 
        self.client.manipulation.add_dataprop_to_ind("now", "Robot", "Int", str(self.init_time.secs))
        self.client.utils.sync_buffered_reasoner()  
        self.client.manipulation.add_objectprop_to_ind("isIn", "Robot", self.get_location())
        self.client.utils.sync_buffered_reasoner()  
        self.client.manipulation.add_dataprop_to_ind("visitedAt", self.get_location(), "Int", str(self.init_time.secs))
        self.client.utils.sync_buffered_reasoner() 
        self.client.manipulation.add_dataprop_to_ind("batteryLvl", "Robot", "Int", str(self.get_battery_level()))
        self.client.utils.sync_buffered_reasoner()  
        self.client.manipulation.add_dataprop_to_ind("urgencyThreshold", "Robot", "Int", "7")
        self.client.utils.sync_buffered_reasoner()  
        self.client.manipulation.add_dataprop_to_ind("batteryThreshold", "Robot", "Int", "300")
        self.client.utils.sync_buffered_reasoner()       

    def cut_dataprop(self, data_prop):
        """ 
        Very simple function that cut the data prop from a string received  from armor.

        Args:
            data_prop(string)
        """
        start = 0
        end = data_prop.rfind('^') - 2
        data_prop = data_prop[(start+1) : end]
        return data_prop

    def cut_dataprop_list(self, data_prop_list):
        """ 
        Very simple function that cut the data prop from a list of strings received  from armor.
        
        Args:
            data_prop_list(string[])
        """
        for i in range(len(data_prop_list)):
            data_prop_list[i] = self.cut_dataprop(data_prop_list[i])
        return data_prop_list
        
    def get_battery_level(self):
        """
        Get the current battery level of the robot from ``robot_states.py`` node, using 
        **/state/get_battery_level** service.

        Returns:
            battery_level(int)
        """
        # Eventually, wait for the server to be initialised.
        rospy.wait_for_service(anm.SERVER_GET_BATTERY_LEVEL)
        try:
            # Call the service and get a response with the current robot battery level.
            service = rospy.ServiceProxy(anm.SERVER_GET_BATTERY_LEVEL, GetBatteryLevel)
            response = service()
            battery_level = response.battery_level

            return battery_level
        #In case service calling is fail
        except rospy.ServiceException as e:
            log_msg = f'Server cannot get current robot battery level: {e}'
            rospy.logerr(anm.tag_log(log_msg, self.log_tag))

    def get_pose(self):
        """
        Get current position of the robot from ``robot_states.py`` node, using 
        **/state/get_pose** service.

        Returns:
            pose(Point)
        """
        #Wait until a service becomes available.
        rospy.wait_for_service(anm.SERVER_GET_POSE)
        try:
            # Call the service and get a response with the current robot position.
            service = rospy.ServiceProxy(anm.SERVER_GET_POSE, GetPose)
            response = service()
            pose = response.pose

            return pose
        #In case service calling is fail
        except rospy.ServiceException as e:
            log_msg = f'Server cannot get current robot position: {e}'
            rospy.logerr(anm.tag_log(log_msg, self.log_tag))
  
    def get_location(self):
        """
        Uses get_pose() function defined in this script to find out current location of the robot is 
        corrispondent to which room. This function uses proximity threshold condition for define area
        of each room.

        Returns:
            is_in(string)
        """
        pose = Point()
        pose = self.get_pose()

        if pose.y >= 5.5:
            is_in = "E"   
        elif pose.x <= -3.75 and pose.y >= -0.75 and pose.y < 5.5:
            is_in = "R1" 
        elif pose.x <= -3.75 and pose.y < -0.75:
            is_in = "R2"
        elif pose.x > -3.75 and pose.x <= 1.25 and pose.y < 5.5:
            is_in = "C1"
        elif pose.x > 1.25 and pose.x <= 6.25 and pose.y < 5.5:
            is_in = "C2"
        elif pose.x > 6.25 and pose.y >= -0.75 and pose.y < 5.5:
            is_in = "R3"
        elif pose.x > 6.25 and pose.y < -0.75:
            is_in = "R4"
        return is_in

    def update_ontology(self, now):
        """
        This function it's mainly used in state machine in ``finite_state_machine.py``.
        Briefly this function is useful to:
            1. Update robot time instance
            2. Update battery level
            3. Update robot location
            4. Update last visited time
            5. Detect urgent locations
            6. Define priority for target room

        Args:
            now(float32)

        Returns:
            target_room(string)
            battery_low(bool)
        """
        #Update robot time instance
        prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("now", "Robot"))[0]
        self.client.manipulation.replace_dataprop_b2_ind("now", "Robot", "Int", str(now.secs), prev_time)
        self.client.utils.sync_buffered_reasoner()
        #Update battery level
        prev_battery_level = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]
        battery_level = str(self.get_battery_level())
        self.client.manipulation.replace_dataprop_b2_ind("batteryLvl", "Robot", "Int", battery_level, prev_battery_level)
        self.client.utils.sync_buffered_reasoner()
        #Update robot location
        prev_loc = self.client.query.objectprop_b2_ind("isIn", "Robot")[0]
        loc = self.get_location()
        self.client.manipulation.replace_objectprop_b2_ind("isIn", "Robot", loc, prev_loc)
        self.client.utils.sync_buffered_reasoner()
        #Update last visited time
        prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", loc))[0]
        self.client.manipulation.replace_dataprop_b2_ind("visitedAt", loc, "Int", str(now.secs), prev_time)
        self.client.utils.sync_buffered_reasoner()
        # Detect target room
        visitedAt_E = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "E"))[0]
        visitedAt_R1 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R1"))[0]
        visitedAt_R2 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R2"))[0]
        visitedAt_R3 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R3"))[0]
        visitedAt_R4 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "R4"))[0]
        visitedAt_C1 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "C1"))[0]
        visitedAt_C2 = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", "C2"))[0]
        visitedAt_dict = {visitedAt_R1: "R1", visitedAt_R2: "R2", visitedAt_R3: "R3", visitedAt_R4: "R4", visitedAt_C1: "C1", visitedAt_C2: "C2", visitedAt_E: "E"}
        visitedAt_dict = dict(sorted(visitedAt_dict.items()))
        room_list = list(visitedAt_dict.values())
        target_room = room_list[0]
        #Detect urgent locations
        urgency_threshold = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("urgencyThreshold", "Robot"))[0]
        if now.secs - int(visitedAt_E) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("E", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("E", "URGENT")
        if now.secs - int(visitedAt_R1) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("R1", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("R1", "URGENT")
        if now.secs - int(visitedAt_R2) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("R2", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("R2", "URGENT")
        if now.secs - int(visitedAt_R3) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("R3", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("R3", "URGENT")
        if now.secs - int(visitedAt_R4) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("R4", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("R4", "URGENT")
        if now.secs - int(visitedAt_C1) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("C1", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("C1", "URGENT")
        if now.secs - int(visitedAt_C2) > int(urgency_threshold):
            self.client.manipulation.add_ind_to_class("C2", "URGENT")
        else:
            self.client.manipulation.remove_ind_from_class("C2", "URGENT")
        self.client.utils.sync_buffered_reasoner()
        urgent_rooms = self.client.query.ind_b2_class("URGENT")

        # Log updated information
        log_msg = Fore.GREEN + 'Ontologycal map Updated...\x1b[0m'
        rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        log_msg = Fore.CYAN + 'Battery level: ' + self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0] + '\x1b[0m'
        rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        log_msg = Fore.YELLOW + 'Current location:' + loc + '\x1b[0m '
        rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        log_msg = Fore.LIGHTRED_EX + 'Urgent locations:\x1b[0m ' 
        rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        for i in range(0,len(urgent_rooms)):
            log_msg = Fore.LIGHTRED_EX + urgent_rooms[i] + '\x1b[0m '
            rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        
        # Define priority for target room
        battery_threshold = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryThreshold", "Robot"))[0]
        battery_lvl = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]
        if int(battery_lvl) > int(battery_threshold):
            battery_low = False
            log_msg = Fore.LIGHTGREEN_EX + 'Target room:' + target_room + '\x1b[0m '
            rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        else:
            battery_low = True
            log_msg = Fore.RED + 'Battery low, moving to charger\x1b[0m' 
            rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
            target_room = 'E'

        return [target_room, battery_low]
