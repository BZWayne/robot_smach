#!/usr/bin/env python

## @package state_machine  
# 
# @brief This is the state_machine node 
# @author Bauyrzhan Zhakanov bauyrzhan.zhakanov@gmail.com
# @version 1.0
# @date 29/12/2022
# 
# Subsctiber to:<BR> 
#   /battery_state    /map_state
#  
# Services used: <BR>
#    / ArmorClient
#
# Description:  
# It controlls how the robot changes its state based on the reasoning of the topological ontology, and the battery state of the robot.
# This node subscribes to two topics /battery_state and /map_state, and it calls the armor server for updating the loaded ontology.
#
# There are 4 states which are:<BR>
# FILING_MAP: The robots keeps waiting in room E untill the whole map is loaded, the robot can know if the map is fully loaded by 
# subscribing to the topic /map_state. When the map is fully loaded the robot go to MOVING_IN_CORRIDORS state trough the transition move.
#
# MOVING_IN_CORRIDORS: The robots keeps moving between C1 and C2 for infinit time, the robots keep cheking the state of the battery by
# subscribing to the topic /battery_state, if the battery is low the robot goes to CHARGING state trough the transition tired. if the battery
# is not low, the robot cheks if there is an urgent room by quering the individuals of the class urg. If there is an urgent room the robot 
# goes to VISITING_URGENT state through the transition urgent.
#
# VISITING_URGENT: First, the robots checks if the battery is low goes to CHARGING state, else it checkes it the urgent room is reacheable
# by quering the object properties of the robot canReach, if the room can be reached it visit it and return back to the MOVING_IN_CORRIDORS
# trough the transition visited. If the urgent room is not reacheable it goes to MOVING_IN_CORRIDORS state through the transition not_reached,
# in this case the robot will change the corridors and visit it again.
#
# CHARGING: The robot keeps checking the state of the battery, if it is full it goes to MOVING_IN_COORIDORS state the the transition charged,
# otherwise, it stays in the CHARGING state.
#
# @see battery
# @see user_interface

import roslib 
import rospy
import smach
import smach_ros
import time
from armor_api.armor_client import ArmorClient
from robot_smach.msg import Battery
from robot_smach.msg import Map

global timer,time_to_stay_in_location,timeR1,timeR2,timeR3,timeR4,timeC1,timeC2,timeE
global visit,to_visit,map_state,battery

map_state = 0
battery = 1
timer = 1665579740
to_visit = 'R'
visit = "<http://bnc/exp-rob-lab/2022-23#>"
time_to_stay_in_location = rospy.get_param("/time_to_stay_in_location")
timeR1=rospy.get_param("/visitedAt_R1")
timeR2=rospy.get_param("/visitedAt_R2")
timeR3=rospy.get_param("/visitedAt_R3")
timeR4=rospy.get_param("/visitedAt_R4")
timeC1=rospy.get_param("/visitedAt_C1")
timeC2=rospy.get_param("/visitedAt_C2")
timeE=rospy.get_param("/visitedAt_E")
path = rospy.get_param("/path")

def battery_callback(data):
        """! Callback function of /battery_state, used in all the states except **FILLING_MAP** to see if the battery is low and change the state
        to **charging**
        @param No parameters
        @return No returned value
        """
        global battery
        battery = data.battery_state
        
def map_state_callback(data):
        """! Callback function of /map_state, used in the state **FILLING_MAP** to see if the map is fully loaded and quit this state forever
        @param No parameters
        @return No returned value
        """
        global map_state
        map_state = data.map_state
        
class LoadMap(smach.State):
	
       def __init__(self):
           """! The filling_map class initializer, Set the list of outcomes 
           @param name  no parameters
           @return  no returned value
           """
           smach.State.__init__(self, outcomes=['loaded', 'not_loaded'])

       def execute(self, userdata):
           """! A method of the class filling_map, Load the ontology and subscribe to /map_state
           @param userdata
           @return  one of its outcomes
           """
           rospy.loginfo('Executing state LoadMap')
           rospy.Subscriber("map_state", Map, map_state_callback)
           ontology = ArmorClient("example", "ontoRef")

           if map_state == 1:
             ontology.call('LOAD', 'FILE', '', [path, 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
             time.sleep(2.0)
             return 'loaded'

           time.sleep(3.0)  
           return 'not_loaded'
           
class MovingInCorridors(smach.State):
	
       def __init__(self):
           """! The moving_in_corridors class initializer, Set the list of outcomes 
           @param name  no parameters
           @return  no returned value
           """           
           smach.State.__init__(self, outcomes=['move', 'tired', 'urgent'])
           ontology = ArmorClient("example", "ontoRef")
      
       def execute(self, userdata):
           """! A method of the class moving_in_corridors
           @param userdata
           @return  one of its outcomes
           """
           rospy.loginfo('Executing state MovingInCorridors')
           rospy.Subscriber("/battery_state", Battery, battery_callback)
           ontology = ArmorClient("example", "ontoRef")
           ontology.call('REASON', '', '', [''])
           current_location = ontology.call('QUERY', 'OBJECTPROP', 'IND', ['isIn', 'Robot1']).queried_objects[0]
           global timer,timeR1,timeR2,timeR3,timeR4,timeC1,timeC2,timeE,path

           if battery == 0:
             # print("low battery")
             return 'tired'
	    
           timer += time_to_stay_in_location
	    
           if current_location == "<http://bnc/exp-rob-lab/2022-23#E>":
             ontology.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'C1', 'E'])
             print("moves from E to C1")
             ontology.call('REPLACE', 'DATAPROP', 'IND', ['visitedAt', 'C1', 'Long', str(timer), str(timeC1)])
             timeC1 = timer
             
           elif current_location == "<http://bnc/exp-rob-lab/2022-23#C2>":
             ontology.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'C1', 'C2'])
             ontology.call('REPLACE', 'DATAPROP', 'IND', ['visitedAt', 'C1', 'Long', str(timer), str(timeC1)])
             print("moves from C2 to C1")
             timeC1 = timer

           elif current_location == "<http://bnc/exp-rob-lab/2022-23#C1>":
             ontology.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'C2', 'C1'])
             ontology.call('REPLACE', 'DATAPROP', 'IND', ['visitedAt', 'C2', 'Long', str(timer), str(timeC2)])  
             print("moves from C1 to C2")         
             timeC2 = timer

           elif current_location == "<http://bnc/exp-rob-lab/2022-23#R1>":
             ontology.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'C1', 'R1'])
             ontology.call('REPLACE', 'DATAPROP', 'IND', ['visitedAt', 'C1', 'Long', str(timer), str(timeC1)])
             print("moves from R1 to C1")
             timeC1 = timer

           elif current_location == "<http://bnc/exp-rob-lab/2022-23#R2>":
             ontology.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'C1', 'R2'])
             ontology.call('REPLACE', 'DATAPROP', 'IND', ['visitedAt', 'C1', 'Long', str(timer), str(timeC1)])
             print("moves from R2 to C1")
             timeC1 = timer

           elif current_location == "<http://bnc/exp-rob-lab/2022-23#R3>":
             ontology.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'C2', 'R3'])
             ontology.call('REPLACE', 'DATAPROP', 'IND', ['visitedAt', 'C2', 'Long', str(timer), str(timeC2)])
             print("moves from R3 to C2")
             timeC2 = timer

           elif current_location == "<http://bnc/exp-rob-lab/2022-23#R4>":
             ontology.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'C2', 'R4']) 
             ontology.call('REPLACE', 'DATAPROP', 'IND', ['visitedAt', 'C2', 'Long', str(timer), str(timeC2)])
             print("moves from R4 to C2")  
             timeC2 = timer 
		
           ontology.call('REPLACE', 'DATAPROP', 'IND' ,['now', 'Robot1', 'long', str(timer), str(timer - 1)])
           ontology.call('REASON', '', '', [''])
           ontology.call('SAVE', '', '', [path])
	    
           urgent_rooms = ontology.call('QUERY', 'IND', 'CLASS', ['URGENT']).queried_objects
           print(urgent_rooms)
           accessible_rooms = ontology.call('QUERY', 'OBJECTPROP', 'IND', ['canReach', 'Robot1']).queried_objects
           print(accessible_rooms)
           global to_visit,visit
           time.sleep(2.0)
	    
           for room in urgent_rooms:
             if room in accessible_rooms:
               to_visit = room.split("#")[1]
               print(room)
               visit = room
               print("to_visit = " + to_visit)
               return 'urgent'
			
           return 'move'
             
class VisitUrgentRoom(smach.State): 
       def __init__(self):
           """! The visiting_urgent class initializer, Set the list of outcomes 
           @param name  no parameters
           @return  no returned value
           """                
           smach.State.__init__(self, outcomes=['visited', 'tired', 'not_reached'])

       def execute(self, userdata):
           """! A method of the class visiting_urgent
           @param userdata
           @return  one of its outcomes
           """
           rospy.loginfo('Executing state VisitUrgentRoom')
           ontology = ArmorClient("example", "ontoRef")
           ontology.call('REASON', '', '', [''])
           rospy.Subscriber("/battery_state", Battery, battery_callback)
           global timeR1, timeR2, timeR3, timeR4,timeC1,timeC2,timeE
           global to_visit,timer,visit,path

           if battery == 0:
             return 'tired'
	      
           reachable = ontology.call('QUERY', 'OBJECTPROP', 'IND', ['canReach', 'Robot1'])

           if len(reachable.queried_objects) > 0:
             for room in reachable.queried_objects:
               print(room)
               if room == visit:
                 print("Urgent room is reachable.")
                 time.sleep(1.0)

                 if to_visit == 'R1':
                   timer += time_to_stay_in_location
                   ontology.call('REPLACE', 'DATAPROP', 'IND', ['visitedAt', to_visit, 'Long', str(timer), str(timeR1)])
                   timeR1 = timer
                   ontology.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'R1', 'C1'])
                   ontology.call('REPLACE', 'DATAPROP', 'IND' , ['now', 'Robot1', 'long', str(timer), str(timer-1)])
                   print("Moving from C1 to R1.")
                   
                 elif to_visit =='R2':
                   timer=timer+time_to_stay_in_location
                   ontology.call('REPLACE','DATAPROP','IND',['visitedAt',to_visit,'Long',str(timer),str(timeR2)])
                   timeR2=timer
                   ontology.call('REPLACE','OBJECTPROP','IND',['isIn','Robot1','R2','C1'])
                   ontology.call('REPLACE','DATAPROP','IND' ,['now','Robot1','long',str(timer),str(timer-1)])
                   print("Moving from C1 to R2.")
                   
                 elif to_visit =='R3':
                   timer=timer+time_to_stay_in_location
                   ontology.call('REPLACE','DATAPROP','IND',['visitedAt',to_visit,'Long',str(timer),str(timeR3)])
                   timeR3=timer
                   ontology.call('REPLACE','OBJECTPROP','IND',['isIn','Robot1','R3','C2'])
                   ontology.call('REPLACE','DATAPROP','IND' ,['now','Robot1','long',str(timer),str(timer-1)])
                   print("Moving from C2 to R3")
				  
                 elif to_visit =='R4':
                   timer=timer+time_to_stay_in_location
                   ontology.call('REPLACE','DATAPROP','IND',['visitedAt',to_visit,'Long',str(timer),str(timeR4)])
                   timeR4=timer
                   ontology.call('REPLACE','OBJECTPROP','IND',['isIn','Robot1','R4','C2'])
                   ontology.call('REPLACE','DATAPROP','IND' ,['now','Robot1','long',str(timer),str(timer-1)])
                   print("Moving from C2 to R4")

                 ontology.call('REASON', '', '', [''])
                 ontology.call('SAVE', '', '', [path])
                 time.sleep(3.0)
                 return 'visited'

           print("The room is unreachable")
           time.sleep(3.0)
           return 'not_reached'

# class ChargingState(smach.State):
#        def __init__(self):
#            """! The charging class initializer, Set the list of outcomes 
#            @param name  no parameters
#            @return  no returned value
#            """  
#            smach.State.__init__(self, outcomes=['charged'])
#            ontology = ArmorClient("example", "ontoRef")
  
#        def execute(self, userdata): 
#            """! A method of the class charging
#            @param userdata
#            @return  one of its outcomes
#            """ 
#            rospy.loginfo('Executing charging state')
#            ontology = ArmorClient("example", "ontoRef")
#            global timeC1,timeC2,timer,timeE

#            while battery == 0:
#              rospy.Subscriber("/battery_state", Battery, battery_callback)
#              time.sleep(1.0)
#              req = ontology.call('QUERY', 'OBJECTPROP', 'IND', ['isIn', 'Robot1'])
#              time.sleep(2.0)
      
#              if req.queried_objects[0] == "<http://bnc/exp-rob-lab/2022-23#C1>":
#                print("Robot is in C1 and needs to be charged")
#                timer += time_to_stay_in_location
#                ontology.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'E', 'C1'])
#                print("Robot moved to E to be charged")
#                ontology.call('REPLACE', 'DATAPROP', 'IND', ['visitedAt', 'E', 'Long', str(timer), str(timeE)])
#                timeE = timer
#                ontology.call('REPLACE', 'DATAPROP', 'IND', ['now', 'Robot1', 'long', str(timer), str(timer - 1)])
#                ontology.call('REASON', '', '', [''])
#                time.sleep(2.0)

#              elif req.queried_objects[0] == "<http://bnc/exp-rob-lab/2022-23#C2>":
#                print("Robot is in C2 and needs to be charged")
#                timer += time_to_stay_in_location 
#                ontology.call('REPLACE','OBJECTPROP','IND',['isIn','Robot1','E','C2'])
#                print("Robot moved to E to be charged")
#                ontology.call('REPLACE','DATAPROP','IND',['visitedAt','E','Long',str(timer),str(timeE)])
#                timeE=timer
#                ontology.call('REPLACE','DATAPROP','IND' ,['now','Robot1','long',str(timer),str(timer-1)])
#                ontology.call('REASON','','',[''])
#                time.sleep(2.0)

#              elif req.queried_objects[0] == "<http://bnc/exp-rob-lab/2022-23#R1>":
#                print("Robot is in R1 and needs to be charged")
#                timer += time_to_stay_in_location 
#                ontology.call('REPLACE','OBJECTPROP','IND',['isIn','Robot1','C1','R1'])
#                print("Robot moved to C1")
#                ontology.call('REPLACE','DATAPROP','IND',['visitedAt','C1','Long',str(timer),str(timeC1)])
#                timeC1=timer
#                ontology.call('REPLACE','DATAPROP','IND' ,['now','Robot1','long',str(timer),str(timer-1)])
#                print(timer)
#                ontology.call('REASON','','',[''])
#                time.sleep(2.0)

#              elif req.queried_objects[0] == "<http://bnc/exp-rob-lab/2022-23#R2>":
#                print("Robot is in R2 and needs to be charged")
#                timer += time_to_stay_in_location 
#                ontology.call('REPLACE','OBJECTPROP','IND',['isIn','Robot1','C1','R2'])
#                print("Robot moved to C1")
#                ontology.call('REPLACE','DATAPROP','IND',['visitedAt','C1','Long',str(timer),str(timeC1)])
#                timeC1=timer
#                ontology.call('REPLACE','DATAPROP','IND' ,['now','Robot1','long',str(timer),str(timer-1)])
#                ontology.call('REASON','','',[''])
#                time.sleep(2.0)

#              elif req.queried_objects[0] == "<http://bnc/exp-rob-lab/2022-23#R3>":
#                print("Robot is in R3 and needs to be charged")
#                timer += time_to_stay_in_location 
#                ontology.call('REPLACE','OBJECTPROP','IND',['isIn','Robot1','C2','R3'])
#                print("Robot moved to C2")
#                ontology.call('REPLACE','DATAPROP','IND',['visitedAt','C2','Long',str(timer),str(timeC2)])
#                timeC2=timer
#                ontology.call('REPLACE','DATAPROP','IND' ,['now','Robot1','long',str(timer),str(timer-1)])
#                print(timer)
#                ontology.call('REASON','','',[''])
#                time.sleep(2.0)

#              elif req.queried_objects[0] == "<http://bnc/exp-rob-lab/2022-23#R4>":
#                print("Robot is in R4 and needs to be charged")
#                timer += time_to_stay_in_location 
#                ontology.call('REPLACE','OBJECTPROP','IND',['isIn','Robot1','C2','R4'])
#                print("Robot moved to C2")
#                ontology.call('REPLACE','DATAPROP','IND',['visitedAt','C2','Long',str(timer),str(timeC2)])
#                timeC2=timer
#                ontology.call('REPLACE','DATAPROP','IND' ,['now','Robot1','long',str(timer),str(timer-1)])
#                print(timer)
#                ontology.call('REASON','','',[''])
#                time.sleep(2.0)

#            time.sleep(2.0)
#            print("Robot is charged")
#            return 'charged'

           
def main():
## The main function. Initialize the node, the client of armor server, smach state machine 
# @param no parameters
# @return no returned value  
       rospy.init_node('smach_example_state_machine')
       client = ArmorClient("example", "ontoRef")
       # Create a SMACH state machine
       sm = smach.StateMachine(outcomes=['container_interface'])
  
       # Open the container
       with sm:
           # Add states to the container
           smach.StateMachine.add('FILLING_MAP', LoadMap(), 
                                  transitions={'loaded':'MOVING_IN_CORRIDORS', 'not_loaded':'FILLING_MAP'})
           smach.StateMachine.add('MOVING_IN_CORRIDORS', MovingInCorridors(), 
                                  transitions={'move':'MOVING_IN_CORRIDORS', 'tired':'CHARGING', 'urgent':'VISITING_URGENT' })
           smach.StateMachine.add('VISITING_URGENT', VisitUrgentRoom(), 
                                  transitions={'visited':'MOVING_IN_CORRIDORS', 'tired':'CHARGING','not_reached':'MOVING_IN_CORRIDORS'})
          #  smach.StateMachine.add('CHARGING', ChargingState(), 
          #                         transitions={'charged':'MOVING_IN_CORRIDORS'})
       sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
       sis.start()
       # Execute SMACH plan
       outcome = sm.execute()
       
if __name__ == '__main__':
       main()
