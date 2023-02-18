#!/usr/bin/env python
"""
.. module:: map
    :platform: Unix
    :synopsis: the map python script
.. moduleauthor:: Bauyrzhan Zhakanov <bauyrzhan.zhakanov@gmail.com>

Map is established utilizing the methods from the ArmorClient and the incomplete Ontology file, created via 		Protege topological_map_new.owl, which only holds class definitions.
"""

import rospy
from os.path import dirname, realpath
import os.path 
from robot_control import logs_mapper as anm
from assignment2.srv import GetPose, GetBatteryLevel
from assignment2.msg import Point
from armor_api.armor_client import ArmorClient

ontology_path_ = "/ontology/"
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
        self.log_tag = log_tag
        self.init_time = init_time
        self.path = '/root/ros_ws/src/robot_smach/'
        self.path = self.path + ontology_path_
        self.client = ArmorClient("ontology", "ontoRef")
        self.client.utils.load_ref_from_file(self.path + ontology_name_, " ", True, "PELLET", False, False)
        self.client.utils.set_log_to_terminal(True)
        self.add_robot(init_time)
        #self.get_location(init_time)
        #self.modified_ontology(init_time)

    def add_room(self, room):
        """
        Adds room to the topological map using ``armor_client``
        """
        print('room is adding')
        self.client.manipulation.add_ind_to_class("E", "ROOM")
        self.client.manipulation.add_ind_to_class("C1", "ROOM")
        self.client.manipulation.add_ind_to_class("C2", "ROOM")
        self.client.manipulation.add_ind_to_class("R1", "ROOM")
        self.client.manipulation.add_ind_to_class("R2", "ROOM")
        self.client.manipulation.add_ind_to_class("R3", "ROOM")
        self.client.manipulation.add_ind_to_class("R4", "ROOM")

    def add_door(self, door):
        """
        Adds door to the topological map using ``armor_client``
        add individual to class method and finaly syncs the reasoner
        """
        print('door is adding')
        self.client.manipulation.add_ind_to_class("D1", "DOOR")
        self.client.manipulation.add_ind_to_class("D2", "DOOR")
        self.client.manipulation.add_ind_to_class("D3", "DOOR")
        self.client.manipulation.add_ind_to_class("D4", "DOOR")
        self.client.manipulation.add_ind_to_class("D5", "DOOR")
        self.client.manipulation.add_ind_to_class("D6", "DOOR")
        self.client.manipulation.add_ind_to_class("D7", "DOOR")

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
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "R1", "D1")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "R2", "D2")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "R3", "D3")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "R4", "D4")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "R4", "D4")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D6")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D7")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D1")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D2")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D5")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D7")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D3")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D4")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D5")
        self.client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D6")

    def add_last_visit_time(self, room, visit_time):
        """
        Sets the last visit time for a specified room using the armor_client method to add data to an individual and syncs the reasoner.

        Args:

            room (string): The name of the room
            visit_time (string): The time of the last visit to the room
        """
        print('last visit time')
        now = rospy.get_rostime()
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "R1", "Int", str(now.secs - 70))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "R2", "Int", str(now.secs - 50))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "R3", "Int", str(now.secs - 30))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "R4", "Int", str(now.secs - 20))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "E", "Int", str(now.secs))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "C1", "Int", str(now.secs - 60))
        self.client.manipulation.add_dataprop_to_ind("visitedAt", "C2", "Int", str(now.secs - 40))  

    def add_robot(self, now):
        """
        Initializes the robot's position in room "E", sets its starting time instance and battery level, 
        and establishes its urgency and battery threshold using the add data and object to individual 
        methods in the armor_client. The reasoner is then synced to reflect these updates.
        """ 
        print('robot is adding')
        self.client.manipulation.add_ind_to_class("Robot", "ROBOT")
        print('robot is 1')  
        self.client.manipulation.add_dataprop_to_ind("now", "Robot", "Int", str(now.secs))
        print('robot is 2')  
        self.client.manipulation.add_objectprop_to_ind("isIn", "Robot", 'E')
        print('robot is 3') 
        self.client.manipulation.add_dataprop_to_ind("batteryLvl", "Robot", "Int", '50')
        print('robot is 5')  
        self.client.manipulation.add_dataprop_to_ind("urgencyThreshold", "Robot", "Int", "7")
        print('robot is 6')  
        self.client.manipulation.add_dataprop_to_ind("batteryThreshold", "Robot", "Int", "400")
        print('robot is 7')  
        self.client.utils.sync_buffered_reasoner()
        print('robot is 8')      

    def charging():
        """Function to charge battery of robot. If battery level of robot is equal to 20 function stops."""
        prev_battery_level = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]
        if  int(prev_battery_level) + 10 < 20:
            battery_lvl = int(prev_battery_level) + 5
        else:
            battery_lvl = 30
        self.client.manipulation.replace_dataprop_b2_ind("batteryLvl", "Robot", "Int", str(battery_lvl), prev_battery_level)

    def using():
        """Function to consume battery of robot when robot move between ROOMs."""
        prev_battery_level = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]
        battery_lvl = int(prev_battery_level) - 1
        self.client.manipulation.replace_dataprop_b2_ind("batteryLvl", "Robot", "Int", str(battery_lvl), prev_battery_level)

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
        # GET BATTERY LEVEL.
        rospy.wait_for_service(anm.ROBOT_GET_BATTERY_LEVEL)
        try:
            #SERVICE
            service = rospy.ServiceProxy(anm.ROBOT_GET_BATTERY_LEVEL, GetBatteryLevel)
            response = service()
            battery_level = response.battery_level
            return battery_level
        except rospy.ServiceException as e:
            print('error in getting battery level')

    def get_pose(self):
        """
        Retrieve the current robot pose by calling the 'state/get_pose' service of the 
            'robot-state' node.
        Returns:
            pose (Point)
        """
        print('initialize pose')
        rospy.wait_for_service(anm.ROBOT_GET_POSE)
        print('robot_get_pose')
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
        Function to detect robot current location and replace new location with previous location.
        
        Return:
            location(string): current location of robot
        """
        global location
        print('get_location_1')
        position = Point()
        print(position)
        print('get_pose_2')
        now = rospy.get_rostime()

        if position.y >= 5.5:
            location = "E"
            
        elif position.x <= -3.75 and position.y >= -0.75 and position.y < 5.5:
            location = "R1"
            
        elif position.x <= -3.75 and position.y < -0.75:
            location = "R2"

        elif position.x > -3.75 and position.x <= 1.25 and position.y < 5.5:
            location = "C1"

        elif position.x > 1.25 and position.x <= 6.25 and position.y < 5.5:
            location = "C2"

        elif position.x > 6.25 and position.y >= -0.75 and position.y < 5.5:
            location = "R3"

        elif position.x > 6.25 and position.y < -0.75:
            location = "R4"

        return location
    

    def modified_ontology(self, now):
        """
        The function which is called in ``finite_state_machine`` node, it gets current time instance as
        an argument, it gets robot current location using ``get_location()`` function, it gets robot current battery
        level using ``get_battery_level()`` function, and updates them in the ontology. It sorts the last visit
        times and detects which room is the most behind and sets it as the target room. It detects the urgent rooms
        considering the last visit times and robot urgeny threshold and updates them in the ontology, finally, it 
        returns the target room as it is found.If the battery level is high enough otherwise it returns room "E" as
        target room, and battery level state.
        Args:
            now(float32)
        Returns:
            target_room(string)
            battery_low(bool)
        """
        # Update robot time instance
        prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("now", "Robot"))[0]
        self.client.manipulation.replace_dataprop_b2_ind("now", "Robot", "Int", str(now.secs), prev_time)

        # Update battery level
        prev_battery_level = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]
        battery_level = str(self.get_battery_level())
        self.client.manipulation.replace_dataprop_b2_ind("batteryLvl", "Robot", "Int", battery_level, prev_battery_level)

        # Update robot location
        prev_loc = self.cut_objprop_list(self.client.query.objectprop_b2_ind("isIn", "Robot"))[0]
        loc = self.get_location()
        self.client.manipulation.replace_objectprop_b2_ind("isIn", "Robot", loc, prev_loc)

        #Update last visited time
        prev_time = self.cut_dataprop_list(self.client.query.dataprop_b2_ind("visitedAt", loc))[0]
        self.client.manipulation.replace_dataprop_b2_ind("visitedAt", loc, "Int", str(now.secs), prev_time)

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
