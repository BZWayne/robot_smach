#!/usr/bin/env python

## @package battery  
# 
# @brief Battery Node 
# @author Bauyrzhan Zhakanov bauyrzhan.zhakanov@gmail.com
# @version 1.0
# @date 29/12/2022
# 
# Publishers to:<BR> 
#   /battery_smach  
#
# Description:  
# /battery_smach is a publisher topic that provides different state of the battery True (full) or False (low) in a specific duration
#
# @see state_machine
# @see user_interface

import rospy
from robot_smach.msg import Battery
import time

time_charge=rospy.get_param("/time_of_one_charge") ## /time_of_one_change is the duration of robot movement during full charge
time_charging=rospy.get_param("/time_of_charging") ## /tume_of_charging is the duration the robot to be charged

def batteryState():
        """! /battery_smach is activated when battery() is running as a main function. 
        @param No parameters
        @return No returned value
        """
        pub = rospy.Publisher('/battery_state', Battery, queue_size=10)
        rospy.init_node('battery', anonymous=True)
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
        
           battery_full = 1  # battery is full
           pub.publish(battery_full)
           print("Battery is fully charged")
           time.sleep (time_charge)
           
           battery_full = 0 # battery is low
           pub.publish(battery_full)
           print("Battery is low")
           time.sleep(time_charging)
   
if __name__ == '__main__':
       try:
           batteryState()
       except rospy.ROSInterruptException:
           pass
