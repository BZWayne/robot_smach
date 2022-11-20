#!/usr/bin/env python

"""
.. module:: robot_states
:platform: Unix
:synopsis: A script that states of robot
.. moduleauthor:: Bauyrzhan Zhakanov
A state machine module is used to show the states of the robot.

"""

import threading
import random
import rospy
from robot_smach import architecture_name_mapper as anm
from std_msgs.msg import Bool
from robot_smach.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse


LOG_TAG = anm.NODE_ROBOT_STATE


class RobotState:
"""
 The node manager class.
 This class defines two services to get and set the current 
 robot pose, and a publisher to notify that the battery is low.
"""

    def __init__(self):
        """ Initialise this node. """
        rospy.init_node(anm.NODE_ROBOT_STATE, log_level=rospy.INFO)
        self._pose = None
        self._battery_low = False
        self._randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE, True)
        if self._randomness:
            self._random_battery_time = rospy.get_param(anm.PARAM_BATTERY_TIME, [15.0, 40.0])
            log_msg = (f'Random-based battery low notification active: the battery change state (i.e., low/high) with a '
                       f'delay in the range of [{self._random_battery_time[0]}, {self._random_battery_time[1]}) seconds.')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        rospy.Service(anm.SERVER_GET_POSE, GetPose, self.get_pose)
        rospy.Service(anm.SERVER_SET_POSE, SetPose, self.set_pose)

        th = threading.Thread(target=self._is_battery_low)
        th.start()

        log_msg = (f'Initialise node `{anm.NODE_ROBOT_STATE}` with services `{anm.SERVER_GET_POSE}` and '
                   f'`{anm.SERVER_SET_POSE}`, and topic {anm.TOPIC_BATTERY_LOW}.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


    def set_pose(self, request):
    """
    The `robot/set_pose` service implementation.
    The `request` input parameter is the current robot pose to be set,
    as given by the client. This server returns an empty `response`.
    """
        if request.pose is not None:
            # Store the new current robot position.
            self._pose = request.pose
            # Log information.
            self._print_info(f'Set current robot position through `{anm.SERVER_SET_POSE}` '
                             f'as ({self._pose.x}, {self._pose.y}).')
        else:
            rospy.logerr(anm.tag_log('Cannot set an unspecified robot position', LOG_TAG))
        # Return an empty response.
        return SetPoseResponse()


    def get_pose(self, request):
    """
    The `robot/get_pose` service implementation.
    The `request` input parameter is given by the client as empty. Thus, it is not used.
    The `response` returned to the client contains the current robot pose.
    """

        if self._pose is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot position', LOG_TAG))
        else:
            log_msg = f'Get current robot position through `{anm.SERVER_GET_POSE}` as ({self._pose.x}, {self._pose.y})'
            self._print_info(log_msg)

        response = GetPoseResponse()
        response.pose = self._pose
        return response


    def _is_battery_low(self):
    """  
    Publish changes of battery levels. This method runs on a separate thread.
    """

        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        if self._randomness:

            self._random_battery_notifier(publisher)
        else:

            self._manual_battery_notifier(publisher)


    def _random_battery_notifier(self, publisher):
    """
    Publish when the battery change state (i.e., high/low) based on a random
    delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
    The message is published through the `publisher` input parameter and is a
    boolean value, i.e., `True`: battery low, `False`: battery high.
    """
        delay = 0  
        while not rospy.is_shutdown():

            publisher.publish(Bool(self._battery_low))

            if self._battery_low:
                log_msg = f'Robot got low battery after {delay} seconds.'
            else:
                log_msg = f'Robot got a fully charged battery after {delay} seconds.'
            self._print_info(log_msg)

            delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
            rospy.sleep(delay)

            self._battery_low = not self._battery_low


    def _manual_battery_notifier(self, publisher):
    """
    Allow keyboard interaction to emulate battery level changes.
    The message is published through the `publisher` input parameter and is a
    boolean value, i.e., `True`: battery low, `False`: battery high.
    """

        print('  # Type `Low` (`L`) to notify that the battery is low, `hiGh` (`G`) that it is fully charged.')
        print('  # Type `cnt+C` and `Enter` to quit.')

        publisher.publish(Bool(self._battery_low))

        while not rospy.is_shutdown():

            user_input = input(' > ')
            user_input = user_input.lower()

            error = False
            if user_input == 'low' or user_input == 'l':
                self._battery_low = True
                rospy.loginfo(anm.tag_log('Robot got low battery.', LOG_TAG))
            elif user_input == 'high' or user_input == 'g':
                self._battery_low = False
                rospy.loginfo(anm.tag_log('Robot got a fully charged battery.', LOG_TAG))
            else:

                print('*** USER INPUT ERROR! Try again:')
                error = True

            if not error:
                publisher.publish(Bool(self._battery_low))


    def _print_info(self, msg):
    """
    Print logging only when random testing is active.
    This is done to allow an intuitive usage of the keyboard-based interface.
    """
        if self._randomness:
            rospy.loginfo(anm.tag_log(msg, LOG_TAG))


if __name__ == "__main__":
    """ Instantiate the node manager class and wait. """
    RobotState()
    rospy.spin()

