#! /usr/bin/env python
"""
.. module:: controller
  :platform: Unix 
  :synopsis: Controller
.. moduleauthor:: Bauyrzhan Zhakanov bauyrzhan.zhakanov@gmail.com
A module represents the controller of the robot. The working principle of the created services would provide the waypoints, that further control the robot trajectory based on it.
Service:
  /motion/controller
"""

import random
import rospy
from robot_smach import architecture_name_mapper as anm
from actionlib import SimpleActionServer
from robot_smach.msg import ControlFeedback, ControlResult

# a tag for identifying logs producer
LOG_TAG = anm.NODE_CONTROLLER   

class ControllingAction(object):
    """A class is used to implement an action server to simulate motion controlling.
    Given a plan as a set of via points, it simulate the movements to reach each point with a random delay.
    """
    def __init__(self):
        # Get random-based parameters used by this server
        self._random_motion_time = rospy.get_param(anm.PARAM_CONTROLLER_TIME, [0.1, 2.0])
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
                                      robot_smach.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()

        log_msg = (f'`{anm.ACTION_CONTROLLER}` Action Server initialised. It will navigate trough the plan with a delay ' 
                   f'between each via point spanning in [{self._random_motion_time[0]}, {self._random_motion_time[1]}).')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


    def execute_callback(self, goal):
        """The callback is used to invoke when a client set a goal to the `controller` server.
        This function requires a list of via points (i.e., the plan), and it simulate a movement through each point with a delay spanning in ['self._random_motion_time[0]`, `self._random_motion_time[1]`).
        """

        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            rospy.logerr(anm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
            self._as.set_aborted()
            return


        feedback = ControlFeedback()
        rospy.loginfo(anm.tag_log('Server is controlling...', LOG_TAG))
        for point in goal.via_points:

            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Service has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()
                return

            delay = random.uniform(self._random_motion_time[0], self._random_motion_time[1])
            rospy.sleep(delay)
            feedback.reached_point = point
            self._as.publish_feedback(feedback)

            log_msg = f'Reaching point ({point.x}, {point.y}).'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(anm.tag_log('Motion control successes.', LOG_TAG))
        self._as.set_succeeded(result)
        return 

def main():
    """ A main function is used to initialise the node, its action server, and wait. 
    """
    rospy.init_node(anm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()

if __name__ == '__main__':
    main()
