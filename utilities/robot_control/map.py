#!/usr/bin/env python
"""
	.. module:: map
		:platform: Unix
		:synopsis: the map python script
	.. moduleauthor:: Bauyrzhan Zhakanov <bauyrzhan.zhakanov@gmail.com>

	Uses Service:
		/state/get_battery  
		/state/get_pose

	Map is established utilizing the methods from the 
	ArmorClient and the incomplete Ontology file, 
	topological_map.owl, which only holds class definitions.
"""

import rospy
from os.path import dirname, realpath
from robot_control import logs_mapper as anm
from assignment2.srv import GetPose, GetBatteryLevel
from assignment2.msg import Point
from armor_api.armor_client import ArmorClient

ontology_path_ = "/../../ontology/"
ontology_name_ = "topological_map_new.owl"


class MapOntology:
    """
        The Topological Map class is used for implementing the topological map. 
        When an instance of the class is created, it loads the topological_map.owl 
        ontology file using the loading method from the ArmorClient script. 
        The class then attempts to complete the ontology by adding rooms, doors,
        and robot individuals, defining their disjointness, pre-setting their 
        last visit times, and positioning the robot in room "E" through calling 
        relevant functions that utilize the methods from ArmorClient.
    """
    def __init__(self, log_tag, init_time):
    	print('0')
        self.log_tag = log_tag
        print('1')
        self.init_time = init_time
        print('2')
        print('3')
        self.path = dirname(realpath(__file__))
        self.path = self.path + ontology_path_
        self.client = ArmorClient("ontology", "ontology_reference")
        self.client.utils.load_ref_from_file(self.path + ontology_name_, " ", False, "PELLET", False, False)
        self.client.utils.set_log_to_terminal(True)
        self.add_robot()

    def add_room(self, room):
        """
        Adds room to the topological map using ``armor_client``
        """
        self.client.manipulation.add_ind_to_class(room, "ROOM")
        self.client.utils.sync_buffered_reasoner()

    def add_door(self, door):
        """
        Adds door to the topological map using ``armor_client``
        add individual to class method and finaly syncs the reasoner
        """
        self.client.manipulation.add_ind_to_class(door, "DOOR")
        self.client.utils.sync_buffered_reasoner()  

    def disjoint_individuals(self):
        """
        Disjoints every individual in each class using ``armor_client`` disjoint 
        individuals of class method and finally syncs the reasoner
        """
        self.client.manipulation.disj_inds_of_class("ROOM")
        self.client.manipulation.disj_inds_of_class("DOOR")
        self.client.utils.sync_buffered_reasoner()

    def assign_doors_to_room(self, room, doors):
        """
        Links the rooms to their respective doors through the ``armor_client`` add object to individual method 
        and syncs the reasoner after completion.

        Args:
            room (string): The name of the room.
            doors (list of strings): The names of the doors associated with the room.
        """
        for i in range(len(doors)):
            self.client.manipulation.add_objectprop_to_ind("hasDoor", room, doors[i])
        self.client.utils.sync_buffered_reasoner()

    def add_last_visit_time(self, room, visit_time):
        """
        Sets the last visit time for a specified room using the armor_client method to add data to an individual and syncs the reasoner.

        Args:

            room (string): The name of the room
            visit_time (string): The time of the last visit to the room
        """
        self.client.manipulation.add_dataprop_to_ind("visitedAt", room, "Int", visit_time)
        self.client.utils.sync_buffered_reasoner()

    def add_robot(self):
        """
        Initializes the robot's position in room "E", sets its starting time instance and battery level, 
        and establishes its urgency and battery threshold using the add data and object to individual 
        methods in the armor_client. The reasoner is then synced to reflect these updates.
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
        self.client.manipulation.add_dataprop_to_ind("batteryThreshold", "Robot", "Int", "400")
        self.client.utils.sync_buffered_reasoner()       

    def cut_dataprop(self, data_prop):
        """ 
        Extracts the object property from a string received from armor.
        Args:
            data_prop(string)
        """
        start = 0
        end = data_prop.rfind('^') - 2
        data_prop = data_prop[(start+1) : end]
        return data_prop

    def cut_dataprop_list(self, data_prop_list):
        """ 
        Extracts the object property from a list of strings received from armor.
        
        Args:
            obj_prop_list(string[]): A list of strings that contain object properties
        
        Returns:
            list[string]: A list of extracted object properties        
        Args:
            data_prop_list(string[])
        """
        for i in range(len(data_prop_list)):
            data_prop_list[i] = self.cut_dataprop(data_prop_list[i])
        return data_prop_list

    def cut_objprop(self, obj_prop):
        """ 
        Extracts the object property from a string received from armor.

        Args:
            obj_prop(string)
        """
        start = obj_prop.rfind('#') + 1
        end = obj_prop.rfind('#') + 2
        obj_prop = obj_prop[(start+1) : end]
        return obj_prop

    def cut_objprop_list(self, obj_prop_list):
        """ 
        Extracts the object properties from a list of strings received from armor.
    
        Args:
            obj_prop_list(list[str]): The list of strings to extract object properties from.
        
        Returns:
            list[str]: The list of extracted object properties.
            
            Args:
                obj_prop_list(string[])
        """
        for i in range(len(obj_prop_list)):
            obj_prop_list[i] = self.cut_objprop(obj_prop_list[i])
        return obj_prop_list

    def get_battery_level(self):
        """
        Retrieve the current robot battery level by the ``state/battery_level`` server of the 
        ``robot-state`` node.

        Returns:
            battery_level(int)
        """
        rospy.wait_for_service(anm.ROBOT_GET_BATTERY_LEVEL)
        try:
            service = rospy.ServiceProxy(anm.ROBOT_GET_BATTERY_LEVEL, GetBatteryLevel)
            response = service()
            battery_level = response.battery_level
            return battery_level
        except rospy.ServiceException as e:
            log_msg = f'Robot cannot get current robot battery level: {e}'
            rospy.logerr(anm.tag_log(log_msg, self.log_tag))

    def get_pose(self):
        """
        Retrieve the current robot pose by calling the 'state/get_pose' service of the 
            'robot-state' node.
        Returns:
            pose (Point)
        """
        rospy.wait_for_service(anm.ROBOT_GET_POSE)
        try:
            service = rospy.ServiceProxy(anm.ROBOT_GET_POSE, GetPose)
            response = service()
            pose = response.pose
            return pose
        except rospy.ServiceException as e:
            log_msg = f'Robot cannot get current robot position: {e}'
            rospy.logerr(anm.tag_log(log_msg, self.log_tag))
  
    def get_location(self):
        """
        Determines the current location of the robot by calling the `get_pose()` function 
        and comparing the resulting pose to the defined room coordinates.
        Returns:
            location (string): The name of the room the robot is in.
        """
        pose = Point()
        pose = self.get_pose()

        if pose.y >= 5.5:
            location = "E"   
        elif pose.x <= -3.75 and pose.y >= -0.75 and pose.y < 5.5:
            location = "R1" 
        elif pose.x <= -3.75 and pose.y < -0.75:
            location = "R2"
        elif pose.x > -3.75 and pose.x <= 1.25 and pose.y < 5.5:
            location = "C1"
        elif pose.x > 1.25 and pose.x <= 6.25 and pose.y < 5.5:
            location = "C2"
        elif pose.x > 6.25 and pose.y >= -0.75 and pose.y < 5.5:
            location = "R3"
        elif pose.x > 6.25 and pose.y < -0.75:
            location = "R4"
        return location

    def update_ontology(self, now):
        """
        The function in the finite_state_machine node takes the current time instance as an argument. 
        It determines the robot's current location using the determine_location() function and the 
        robot's current battery level using the get_robot_battery_level() function. It updates this 
        information in the ontology. The function then determines the room that has not been visited
        for the longest time and sets it as the target room. If the last visit times require it, the 
        function updates the list of urgent rooms in the ontology. Finally, it returns the target room 
        and whether the battery level is low. If the battery level is sufficient, the target room will 
        be the room with the longest gap in visits. If the battery level is too low, the target room 
        will be room "E".

        Args:

            now (float32)

        Returns:

            target_room (string)
            battery_low (bool)
        """
        # Update robot time instance
        prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("now", "Robot"))[0]
        self.client.manipulation.replace_dataprop_b2_ind("now", "Robot", "Int", str(now.secs), prev_time)
        self.client.utils.sync_buffered_reasoner()
        # Update battery level
        prev_battery_level = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]
        battery_level = str(self.get_battery_level())
        self.client.manipulation.replace_dataprop_b2_ind("batteryLvl", "Robot", "Int", battery_level, prev_battery_level)
        self.client.utils.sync_buffered_reasoner()
        # Update robot location
        prev_loc = self.cut_objprop_list(self.client.query.objectprop_b2_ind("isIn", "Robot"))[0]
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
        # Detect urgent locations
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
        log_msg = 'Ontology Updated...'
        rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        log_msg = 'battery level: ' + self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]
        rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        log_msg = 'current location: ' + loc
        rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        log_msg = 'urgent locations: ' 
        rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        for i in range(0,len(urgent_rooms)):
            log_msg = urgent_rooms[i]
            rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        
        # Define priority for target room
        battery_threshold = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryThreshold", "Robot"))[0]
        battery_lvl = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]
        if int(battery_lvl) > int(battery_threshold):
            battery_low = False
            log_msg = 'next target room: ' + target_room
            rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
        else:
            battery_low = True
            log_msg = 'battery low, moving to charger' 
            rospy.loginfo(anm.tag_log(log_msg, self.log_tag))
            target_room = 'E'

        return [target_room, battery_low]

