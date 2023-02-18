#!/usr/bin/env python
"""
.. module:: robot_states
    :platform: Unix
    :synopsis: the robot_states python script 
    
.. moduleauthor:: Bauyrzhan Zhakanov <bauyrzhan.zhakanov@gmail.com>

Subscribes to :
    /odom
Uses Service:
    /state/get_pose
    /state/set_pose 

Robot states is the script where the robot condition will be identified such as battery level, base movement, odometry information,
its pose information. 
"""
import rospy
from robot_control import logs_mapper as anm
from robot_smach.msg import Point
from robot_smach.srv import GetPose, SetPose, SetPoseResponse, GetPoseResponse

### node fsm
node_robot = anm.NODE_ROBOT_STATE

class RobotState:
    """
    Initializes the "robot-states" node to provide crucial information about the robot's current 
    state, such as its pose, battery level, and base movement state.
    """

    def __init__(self):
        # Initialise robot position and node
        rospy.init_node(anm.NODE_ROBOT_STATE, log_level=rospy.INFO)
        self._pose = Point()
        
        # Define services.
        rospy.Service(anm.ROBOT_GET_POSE, GetPose, self.get_pose)

    def get_pose(self, request):
        """
        The implementation of the /state/get_pose service. The input parameter, request, 
        from the client is not used. The response returned to the client contains the current 
        robot pose. The argument request is of type GetPoseRequest. The returned value is of
        type GetPoseResponse.
        """
        if self._pose is None:
            rospy.logerr(anm.tag_log('Error', node_fsm))
        else:
            log_msg = f'{anm.ROBOT_GET_POSE}` are ({self._pose.x}, {self._pose.y})'
            self._print_info(log_msg)
        response = GetPoseResponse()
        response.pose = self._pose
        return response

if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()
