#! /usr/bin/env python

"""
.. module:: planner
:platform: Unix
:synopsis: A script that plan the trajectory of robot
.. moduleauthor:: Bauyrzhan Zhakanov
A planner module is used to plan an architecture of the robot.

```
Service:
/motion/planner

```

"""

import random
import rospy
from robot_smach import architecture_name_mapper as anm
from actionlib import SimpleActionServer
from robot_smach.msg import Point, PlanFeedback, PlanResult
from robot_smach.srv import GetPose
import robot_smach 


LOG_TAG = anm.NODE_PLANNER


class PlaningAction(object):

 """ An action server to simulate motion planning.
 Given a target position, it retrieve the current robot position from the 
`robot-state` node, and return a plan as a set of via points. """

    def __init__(self):
""" Get random-based parameters used by this server. Instantiate and start the action server based on the `SimpleActionServer` class."""
        self._random_plan_points = rospy.get_param(anm.PARAM_PLANNER_POINTS, [2, 8])
        self._random_plan_time = rospy.get_param(anm.PARAM_PLANNER_TIME, [0.1, 1])
        self._environment_size = rospy.get_param(anm.PARAM_ENVIRONMENT_SIZE)
        self._as = SimpleActionServer(anm.ACTION_PLANNER, 
                                      robot_smach.msg.PlanAction, 
                                      execute_cb=self.execute_callback, 
                                      auto_start=False)
        self._as.start()
        # Log information.
        log_msg = (f'`{anm.ACTION_PLANNER}` Action Server initialised. It will create random path with a number of point '
                   f'spanning in [{self._random_plan_points[0]}, {self._random_plan_points[1]}). Each point will be generated '
                   f'with a delay spanning in [{self._random_plan_time[0]}, {self._random_plan_time[1]}).')
      

    def execute_callback(self, goal):
    	"""
    The callback invoked when a client set a goal to the `planner` server.
    This function will return a list of random points (i.e., the plan) when the fist point
    is the current robot position (retrieved from the `robot-state` node), while the last 
    point is the `goal` position (given as input parameter). The plan will contain 
    a random number of other points, which spans in the range 
    [`self._random_plan_points[0]`, `self._random_plan_points[1]`). To simulate computation,
    each point is added to the plan with a random delay spanning in the range 
    [`self._random_plan_time[0]`, `self._random_plan_time[1]`).
    	"""
    	
        start_point = _get_pose_client()
        target_point = goal.target


        if start_point is None or target_point is None:
            log_msg = 'Cannot have `None` start point nor target_point. This service will be aborted!.'
            rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return
        if not(self._is_valid(start_point) and self._is_valid(target_point)):
            log_msg = (f'Start point ({start_point.x}, {start_point.y}) or target point ({target_point.x}, '
                       f'{target_point.y}) point out of the environment. This service will be aborted!.')
            rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

            self._as.set_aborted()
            return
        

        feedback = PlanFeedback()
        feedback.via_points = []
        feedback.via_points.append(start_point)

        self._as.publish_feedback(feedback)
        delay = random.uniform(self._random_plan_time[0], self._random_plan_time[1])
        rospy.sleep(delay)


        number_of_points = random.randint(self._random_plan_points[0], self._random_plan_points[1] + 1)
        log_msg = f'Server is planning {number_of_points + 1} points...'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


        for i in range(1, number_of_points):
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Server has been cancelled by the client!', LOG_TAG))

                self._as.set_preempted()  
                return

            new_point = Point()
            new_point.x = random.uniform(0, self._environment_size[0])
            new_point.y = random.uniform(0, self._environment_size[1])
            feedback.via_points.append(new_point)
            if i < number_of_points - 1:

                self._as.publish_feedback(feedback)

                delay = random.uniform(self._random_plan_time[0], self._random_plan_time[1])
                rospy.sleep(delay)
            else:

                feedback.via_points.append(target_point)


        result = PlanResult()
        result.via_points = feedback.via_points
        self._as.set_succeeded(result)
        log_msg = 'Motion plan succeeded with plan: '
        log_msg += ''.join('(' + str(point.x) + ', ' + str(point.y) + '), ' for point in result.via_points)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


    def _is_valid(self, point):
"""
    Check if the point is within the environment bounds, i.e.
    x: [0, `self._environment_size[0]`], and y: [0, `self._environment_size[1]`].
"""
        return 0.0 <= point.x <= self._environment_size[0] and 0.0 <= point.y <= self._environment_size[1]


def _get_pose_client():
"""
 Retrieve the current robot pose by the `state/get_pose` server of the `robot-state` node. Eventually, wait for the server to be initialised.
"""

    rospy.wait_for_service(anm.SERVER_GET_POSE)
    try:

        service = rospy.ServiceProxy(anm.SERVER_GET_POSE, GetPose)
        response = service()
        pose = response.pose

        log_msg = f'Retrieving current robot position from the `{anm.NODE_ROBOT_STATE}` node as: ({pose.x}, {pose.y}).'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        return pose
    except rospy.ServiceException as e:
        log_msg = f'Server cannot get current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))


if __name__ == '__main__':

    """ Initialise the node, its action server, and wait."""
    rospy.init_node(anm.NODE_PLANNER, log_level=rospy.INFO)
    server = PlaningAction()
    rospy.spin()
