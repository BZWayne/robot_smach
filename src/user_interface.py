#!/usr/bin/env python

## @package user_interface  
# 
# @brief User_interface node 
# @author Bauyrzhan Zhakanov bauyrzhan.zhakanov@gmail.com
# @version 1.0
# @date 29/12/2022
# 
# Publishers to:<BR> 
#   /map_smach  
#
# Description:  
# User_interface node continuously interacts with the user and enables access and 
# modification of the ontology through specific input commands. It communicates with the 
# armor server and sends appropriate commands. The node also regularly updates the battery 
# state by publishing the map state to the topic /battery_smach with 0 for an unloaded state 
# and 1 for a fully loaded state.
#
# @see state_machine
# @see battery

import rospy
from armor_api.armor_client import ArmorClient
from robot_smach.msg import Map

# the main arguments
rospy.init_node('user_interface', anonymous=True)
path = rospy.get_param("/path")
user_publisher = rospy.Publisher('map_state', Map, queue_size=2)
ontology = ArmorClient("example", "ontoRef")

def main_interface():

    state_of_the_map = 0
    user_publisher.publish(state_of_the_map)
    ontology.call('LOAD','FILE','',[path, 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])

    while True:
        print("Choose the type of operation: (type [m] for manipulation, type [q] for query, type [u] for utils)")
        type_of_operation = input("Choose the type of operation: ")

        while not type_of_operation == 'm' or type_of_operation == 'q' or type_of_operation == 'u':
          type_of_operation = input("Choose again the type of operation: ")

        reference_name = input("Name the operation: ")
        first_command = input("Enter the first command: ")
        second_command = input("Enter the second command: ")
        num_of_args = int(input("Enter the number of args: "))
        args = [input("Enter argument number %d: " % (i+1)) for i in range(num_of_args)]

        ontology.call(reference_name, first_command, second_command, args)

        if type_of_operation == "m":
            ontology.call('REASON','','',[''])
            ontology.call('SAVE','','',[path])
            map_status = input("Is map finished? (yes/no)")
            state_of_the_map = 1 if map_status == "yes" else 0
            user_publisher.publish(state_of_the_map)

if __name__ == '__main__':
    try:
        main_interface()
    except rospy.ROSInterruptException:
        pass

