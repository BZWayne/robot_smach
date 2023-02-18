#!/usr/bin/env python
"""
.. module:: fsm
    :platform: Unix
    :synopsis: the fsm python script
.. moduleauthor:: Bauyrzhan Zhakanov <bauyrzhan.zhakanov@gmail.com>

The finite state machine's initial state involves building a semantic map of
the environment using image IDs of markers detected by the robot camera. 
This node updates the ontology using the map.py helper script 
and retrieves the target room based on the last visit times and the robot's 
battery state. In the next state, the robot moves to the target room, and if 
the battery level drops below a threshold, the robot goes to the charger to recharge.
"""
import rospy
import smach
import smach_ros
import math
import numpy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from assignment2.msg import RoomConnection, Point
from assignment2.srv import SetBatteryLevel, GetPose, SetBaseMovementState, RoomInformation
from robot_control import logs_mapper as anm
from map import * 
from std_msgs.msg import Int32
from std_srvs.srv import SetBool
from os.path import dirname, realpath
from threading import Lock 
import sys

# logs
node_fsm = anm.NODE_FINITE_STATE_MACHINE
node_time_for_log = 5

publisher_ = None
ontology = None
threading_lock = None
room_id = []
room_name = []
room_center = []
room_connection = []
door = []

def movingPose(data):
    """
    The function serves as an action client for the "move_base" node. It takes a pose as an argument 
    and sends it in the form of "MoveBaseGoal.msg" to the action server. The function returns the 
    result in the form of "MoveBaseResult.msg".
    """
    print('sending goal')
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = data.x
    goal.target_pose.pose.position.y = data.y
    goal.target_pose.pose.orientation.w = 1.0
    client.wait_for_server()
    client.send_goal(goal)
    
def getConfirmation(data):
    """
    The function serves if the robot has arrived at a specific point by calculating the Euclidean distance 
    between the robot's current pose and the target pose.
    """
    rospy.wait_for_service(anm.ROBOT_GET_POSE)
    try:
        service = rospy.ServiceProxy(anm.ROBOT_GET_POSE, GetPose)
        response = service()
        pose = response.pose
        if math.sqrt((data.x - pose.x)**2 + (data.y - pose.y)**2) < 1:
            reached = True
        else:
            reached = False
        log_msg = 'target reached state: ' + str(reached)
        rospy.loginfo(anm.tag_log(log_msg, node_fsm))
        return reached
    except rospy.ServiceException as e:
        log_msg = f'Server cannot get current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, node_fsm))

def getRoomLocation(data):
    """
    This function detects the center position of a specific room by using the room information.
    Args:
        room (string): the name of the room to detect the center position of
    Returns:
        room_pose (Point): the center position of the specified room
    """
    global room_name
    global room_center
    room_pose = Point()
    room_index = room_name.index(data)
    room_pose.x = room_center[room_index][0]
    room_pose.y = room_center[room_index][1]
    return room_pose

def battery_data(data):
    """
    Updates the battery level of the robot by using the /state/battery_data service.
    The current level of the robot's battery is stored in the robot-state node.
    """
    rospy.wait_for_service(anm.ROBOT_SET_BATTERY_LEVEL)
    try:
        log_msg = f'Set a battery level to the `{anm.ROBOT_SET_BATTERY_LEVEL}` node.'
        rospy.loginfo(anm.tag_log(log_msg, node_fsm))
        service = rospy.ServiceProxy(anm.ROBOT_SET_BATTERY_LEVEL, SetBatteryLevel)
        service(data)
    except rospy.ServiceException as e:
        log_msg = f'Cannot set current robot battery level: {e}'
        rospy.logerr(anm.tag_log(log_msg, node_fsm))

def base_data(data):
    """
    Service client function for /state/set_base_movement_state. Modifies the robot's base movement 
    state stored in the robot-states node.
    """
    rospy.wait_for_service(anm.ROBOT_SET_BASE_MOVEMENT_STATE)
    try:
        log_msg = 'Setting base movement state to ' + str(data)
        rospy.loginfo(anm.tag_log(log_msg, node_fsm))
        service = rospy.ServiceProxy(anm.ROBOT_SET_BASE_MOVEMENT_STATE, SetBaseMovementState)
        service(data)  
    except rospy.ServiceException as e:
        log_msg = f'Cannot set base movement: {e}'
        rospy.logerr(anm.tag_log(log_msg, node_fsm))

class LoadMap(smach.State):
    """
    The initial state is defined as building a semantic map for the robot. The arm of the robot is set to move along
    desired trajectories as defined in the robotik.cpp file. If sufficient room 
    IDs are detected, the state will exit and return the status of map_loaded.
    """
    def __init__(self):
        """
        Initialize the execution with map_loaded state
        """
        smach.State.__init__(self, outcomes=['map_loaded'])

    def execute(self, userdata):
        """
        Execute the LoadMap() state
        """
        global threading_lock
        global room_id

        ### execution
        print('robot moved')
        while not rospy.is_shutdown():
            threading_lock.acquire()
            try:
                if len(room_id) < 6:
                    print('check')
                    return 'map_loaded'
            except:
                print('not loaded')
            finally:
                threading_lock.release()
            rospy.sleep(node_time_for_log)

class MovingCorridor(smach.State):
    """
    The state where the robot moves to a desired location is defined in this code. 
    First, the function set_base_movement_state(base_movement_state) is used to enable battery 
    consumption. Then, the robot moves to the target room with the help of the function 
    movingPose(pose). As the robot moves to its destination, the ontology is updated with 
    the function update_ontology(now). When the robot reaches its target room, the state is 
    exited and the function returns reached. If the battery level drops below the threshold, 
    the function returns battery_low and the target room is cancelled.
    """
    def __init__(self):
        """
        Initialize the execution with reached or battery_low states
        """
        smach.State.__init__(self, outcomes=['reached', 'battery_low'])

    def execute(self, userdata):
        """
        Excecute MovintCorridor() state
        """
        global threading_lock
        global ontology

        ## parameters for ontology
        now = rospy.get_rostime()
        print('moving')
        target_room = ontology.modified_ontology(now)
        print('update_ontology')
        target_room_pose = getRoomLocation(target_room)
        print('get room location')
        base_data(True)
        print('...')
        movingPose(target_room_pose)
        print('target position pose')

        ## execution
        while not rospy.is_shutdown(): 
            threading_lock.acquire()
            try:
                now = rospy.get_rostime()
                next_target_room = ontology.modified_ontology(now)
                log_msg = 'target room: ' + target_room
                rospy.loginfo(anm.tag_log(log_msg, node_fsm))
                if battery_low:
                    return 'battery_low'
                else:
                    reached = getConfirmation(target_room_pose)
                    if reached:
                        base_data(False)
                        return 'reached'
            finally:
                threading_lock.release()
            rospy.sleep(node_time_for_log)

class DiscoverRoom(smach.State):
    """
    Defines the state where the robot has arrived at the target room and begins to explore it. 
    The robot arm's movement is enabled just like in the initial state
    function, and then it returns discovered.
    """

    def __init__(self):
        """
        Initialize the execution with discovered state
        """
        smach.State.__init__(self, outcomes=['discovered'])

    def execute(self, userdata):
        """
        Executes DiscoverRoom() state
        """
        global threading_lock

        ## execution
        while not rospy.is_shutdown(): 
            threading_lock.acquire()
            try:
                print('robot discover the room')
                sleep(10)
                return 'discovered'

            finally:
                threading_lock.release()

class MovingForBattery(smach.State):
    """
    The state when the battery is low and the robot needs to recharge is defined. 
    The function set_base_movement_state(movement_state) is used to enable battery 
    consumption, and the robot moves towards the charger using the movingPose(pose) 
    function. The ontology is updated during the movement to the charger using the 
    update_ontology(now) function until the robot reaches the charger."
    """
    def __init__(self):
        """
        Initialize the execution with reached state
        """
        smach.State.__init__(self, outcomes=['reached'])

    def execute(self, userdata):
        """
        Executes MovingForBattery() state
        """
        global threading_lock
        global ontology

        ## parameters for ontology
        now = rospy.get_rostime()
        ontology.modified_ontology(now)
        target_room_pose = getRoomLocation('E')
        base_data(True)
        movingPose(target_room_pose)

        ## execution
        while not rospy.is_shutdown():  
            threading_lock.acquire()
            try:
                now = rospy.get_rostime()
                ontology.modified_ontology(now)
                reached = getConfirmation(target_room_pose)
                if reached:
                    base_data(False)
                    return 'reached'              

            finally:
                threading_lock.release()
            rospy.sleep(node_time_for_log)

class Charging(smach.State):
    """
    The state is called Charging() and it activates the robot has reached the charging station and 
    recharges the battery. The battery level is updated using the set_battery_level(battery_level) 
    function after a certain period of time has elapsed. The state then transitions to charged.
    """
    def __init__(self):
        """
        Initialize the execution with charged state
        """
        smach.State.__init__(self, outcomes=['charged'])

    def execute(self, userdata):
        """
        Executes Charging() state
        """
        global threading_lock
        global ontology
        
        ### execution
        while not rospy.is_shutdown():  
            #threading_lock.acquire()
            try:
                now = rospy.get_rostime()
                ontology.modified_ontology()
                rospy.sleep(10)
                battery_data(1000)
                return 'charged'

            finally:
                threading_lock.release()

def main():
    """
    The main function for the finite_state_machine node initializes the node and takes an instance of the
    MapOntology class at the current time. It defines a subscriber for the /image_id topic and outlines the
    states and transitions for the finite state machine's topological map. Finally, it starts the finite state machine process.
    """
    # Initialise this node.
    global publisher_
    global threading_lock
    global ontology
    
    rospy.init_node(node_fsm, log_level=rospy.INFO)
    print('Loading ontology')
 
     #Build topological map
    now = rospy.get_rostime()
    ontology = MapOntology(node_fsm, now)

    # Get or create a new threading_lock.
    if threading_lock is None:
       threading_lock = Lock()
    else:
       threading_lock = threading_lock

    # Create a SMACH state machine
    robot_sm = smach.StateMachine(outcomes=[])
    robot_sm.userdata.sm_counter = 0

    # Open the container
    with robot_sm:
        # Add states to the container
        smach.StateMachine.add('Load_Map', LoadMap(), 
                                transitions={'map_loaded':'Moving_Corridor'})
        smach.StateMachine.add('Moving_Corridor', MovingCorridor(), 
                                transitions={'battery_low':'Moving_For_Battery', 'reached':'Discover_Room'})
        smach.StateMachine.add('Moving_For_Battery', MovingForBattery(), 
                                transitions={'reached':'Charging'})
        smach.StateMachine.add('Discover_Room', DiscoverRoom(), 
                                transitions={'discovered':'Moving_Corridor'})
        smach.StateMachine.add('Charging', Charging(), 
                                transitions={'charged':'Moving_Corridor'})

    # Create and start the introspection for visualization
    sis = smach_ros.IntrospectionServer('server_name', robot_sm, '/SM_ROOT')
    sis.start()

    # Execute the smach
    outcome = robot_sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
	main()
