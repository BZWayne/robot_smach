#! /usr/bin/env python

"""
       .. module:: controller
          :platform: Unix 
	  :synopsis: A script that controls robot 
	.. moduleauthor:: Bauyrzhan Zhakanov 
	A controller module is used to control architecture of the robot, based on python
  	
  	Service:
  	/motion/controller
"""

import random
import rospy

from robot_smach import architecture_name_mapper as anm

from actionlib import SimpleActionServer

from robot_smach.msg import ControlFeedback, ControlResult
from robot_smach.srv import SetPose
import robot_smach  

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_CONTROLLER


class ControllingAction(object):
    """An action server to simulate motion controlling. Given a plan as a set of via points, 		it simulate the movements to reach each point with a random delay. This server updates 	the current robot position stored in the `robot-state` node. """

    def __init__(self):
        self._random_motion_time = rospy.get_param(anm.PARAM_CONTROLLER_TIME, [0.1, 2.0])
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
                                      robot_smach.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()
        # Log information.
        log_msg = (f'`{anm.ACTION_CONTROLLER}` Action Server initialised. It will navigate trough the plan with a delay ' 
                   f'between each via point spanning in [{self._random_motion_time[0]}, {self._random_motion_time[1]}).')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    def execute_callback(self, goal):
	"""The callback invoked when a client set a goal to the `controller` server. This   function requires a list of via points (i.e., the plan), and it simulate a movement through each point with a delay spanning in ['self._random_motion_time[0]`, `self._random_motion_time[1]`). As soon as each via point is reached, the related robot position is updated in the `robot-state` node. Check if the provided plan is processable. If not, this service will be aborted."""
	
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            rospy.logerr(anm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
            self._as.set_aborted()
            return

        feedback = ControlFeedback()
        rospy.loginfo(anm.tag_log('Server is controlling...', LOG_TAG))
        for point in goal.via_points:

            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Service has been cancelled by the client!', LOG_TAG))

                self._as.set_preempted()
                return

            delay = random.uniform(self._random_motion_time[0], self._random_motion_time[1])
            rospy.sleep(delay)

            feedback.reached_point = point
            self._as.publish_feedback(feedback)

            _set_pose_client(point)

            log_msg = f'Reaching point ({point.x}, {point.y}).'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(anm.tag_log('Motion control successes.', LOG_TAG))
        self._as.set_succeeded(result)
        return  # Succeeded.


def _set_pose_client(pose):
	""" Update the current robot `pose` stored in the `robot-state` node. This method is performed for each point provided in the action's server feedback. Eventually, wait for the server to be initialised. """
    rospy.wait_for_service(anm.SERVER_SET_POSE)
    try:

        log_msg = f'Set current robot position to the `{anm.SERVER_SET_POSE}` node.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
        service(pose)  
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))


if __name__ == '__main__':
    """ Initialise the node, its action server, and wait.  """
    rospy.init_node(anm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()
