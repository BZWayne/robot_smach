#!/usr/bin/env python

# Use randomness for testing purposes.
import random

# Imports relative to ROS and the SMACH library.
import rospy
import smach_ros
from smach import StateMachine, State

# Import constant names that define the architecture's structure.
from arch_skeleton import architecture_name_mapper as anm

# Import a class that decouples the interface of the Finite State Machine with
# the other  nodes of the architecture from the actual implementation of the
# Finite State Machine, which is available in this file.
from arch_skeleton.fsm_helper import InterfaceHelper

# Import used messages defined within the ROS architecture.
from arch_skeleton.msg import Gesture, Point, Speech, PlanAction, PlanGoal, ControlAction, ControlGoal


# The list of names that identify the states of the Finite State Machine.
STATE_RECHARGING = 'RECHARGING'  # The name of the state where the robot waits to recharging its battery.
STATE_NORMAL = 'NORMAL'  # An higher level state that include `PLAN_TO_RANDOM_POSE` and `GO_TO_RANDOM_POSE`.
STATE_PLAN_TO_RANDOM_POSE = 'PLAN_TO_RANDOM_POSE'  # Plan the via points to reach a new random position.
STATE_GO_TO_RANDOM_POSE = 'GO_TO_RANDOM_POSE'  # Follow the plan computed in the `PLAN_TO_RANDOM_POSE` state.
STATE_INTERACT = 'INTERACT'  # An higher level state that include `PLAN_TO_USER`, `GO_TO_USER`, `PLAN_TO_GESTURE`, `GO_TO_GESTURE`.
STATE_PLAN_TO_USER = 'PLAN_TO_USER'  # Plan the via points to reach the user.
STATE_GO_TO_USER = 'GO_TO_USER'  # Follow the plan computed in the `PLAN_TO_USER` state.
STATE_PLAN_TO_GESTURE = 'PLAN_TO_GESTURE'  # Plan the via points to reach the position pointed by the user.
STATE_GO_TO_GESTURE = 'GO_TO_GESTURE'  # Follow the plan computed in the `PLAN_TO_GESTURE` state.


# The list of names that identify the transitions of the Finite State Machine.
TRANS_RECHARGING = 'recharging'  # The transition for associating each state of the inner Finite State Machines to the `battery_low` transition.
TRANS_BATTERY_LOW = 'battery_low'  # The transition from the inner Finite State Machine associated with the `NORMAL` and `INTERACT` states toward the `RECHARGING` state.
TRAMS_RECHARGED = 'charged'  # The transition from the `RECHARGING` state to the inner Finite State Machine associated with the `NORMAL` state.
TRANS_CALLED = 'called'  # The transition from the `NORMAL` to the `INTERACT` state.
TRANS_GREETED = 'greeted'  # The transition from the `INTERACT` to the `NORMAL` state.
TRANS_REPEAT = 'repeat'  # The transition to repeat the `NORMAL` or `INTERACT` states, and the related inner Finite State Machine.
TRANS_START_INTERACTION = 'start_interaction'  # The transition for associating the inner Finite State Machine related to the `NORMAL` state with to the `called` transition.
TRANS_STOP_INTERACTION = 'stop_interaction'  # The transition for associating the inner Finite State Machine related to the `INTERACT` state with to the `greeted` transition.
TRANS_PLANNED_TO_RANDOM_POSE = 'planned_to_random_pose'  # The transition from the `PLAN_TO_RANDOM_POSE` state to the `GO_TO_RANDOM_POSE` state.
TRANS_WENT_RANDOM_POSE = 'went_random_pose'  # The transition associated with the `repeat` transition of the inner Finite Machine related to the `NORMAL` state.
TRANS_PLANNED_TO_USER = 'planned_to_user'  # The transition from the `PLAN_TO_USER` state to the `GO_TO_USER` state.
TRANS_WENT_TO_USER = 'went_to_user'  # The transition from the `GO_TO_USER` state to the `PLAN_TO_GESTURE` state.
TRANS_PLANNED_TO_GESTURE = 'planned_to_gesture'  # The transition from the `PLAN_TO_GESTURE` state to the `GO_TO_GESTURE` state.
TRANS_WENT_TO_GESTURE = 'went_to_gesture'  # The transition associated with the `repeat` transition of the inner Finite Machine related to the `INTERACT` state.


# The list of names that identify variables passed across the nodes of the Finite State Machine.
VAR_RANDOM_PLAN = 'random_plan'  # The planned via-points toward a random position. It is computed by the `PLAN_TO_RANDOM_POSE` state, and used by `GO_TO_RANDOM_POSE`.
VAR_TO_USE_PLAN = 'to_user_plan'  # The planned via-points toward the user's position. It is computed by the `PLAN_TO_USER` state, and used `GO_TO_USER`.
VAR_TO_GESTURE_PLAN = 'to_gesture_plan'  # The planned via-points toward the location pointed by the user. It is computed by the `PLAN_TO_GESTURE` state, and used by `GO_TO_GESTURE`.


# Sleeping time (in seconds) of the waiting thread to allow the computations
# for getting stimulus from the other components of the architecture.
LOOP_SLEEP_TIME = 0.3


# A tag for identifying logs producer.
LOG_TAG = anm.NODE_BEHAVIOUR


# The definition of the recharging state.
class Recharging(State):
    # Construct this class, i.e., initialise this state.
    def __init__(self, interface_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        State.__init__(self, outcomes=[TRAMS_RECHARGED])

    # Define the function performed each time a transition is such to enter in this state.
    # Note that the input parameter `userdata` is not used since no data is required from the other states.
    def execute(self, userdata):
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is no low anymore take the `charged` transition.
                if not self._helper.is_battery_low():
                    self._helper.reset_states()  # Reset the state variable related to the stimulus.
                    return TRAMS_RECHARGED
                # Note that if unexpected stimulus comes from the other nodes of the architecture through the
                # `self._helper` class, then this state will not take any transitions. This is equivalent to have a
                # loop-like transition in the behavioural UML diagram for all the other stimulus except `TRAMS_RECHARGED`.
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)


# The definition of the inner state of `NORMAL` to plan the via-points to reach a random position.
class PlanToRandomPose(State):
    # Construct this class, i.e., initialise this state.
    def __init__(self, interface_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        # Get the environment size from ROS parameters.
        self.environment_size = rospy.get_param(anm.PARAM_ENVIRONMENT_SIZE)
        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        # Also, set the `random_plan` variable, which will be eventually computed in the `execute` function and passed to the `GO_TO_RANDOM_POSE` state.
        State.__init__(self, outcomes=[TRANS_RECHARGING, TRANS_START_INTERACTION, TRANS_PLANNED_TO_RANDOM_POSE],
                       output_keys=[VAR_RANDOM_PLAN])

    # Define the function performed each time a transition is such to enter in this state.
    def execute(self, userdata):
        # Define a random point to be reached through some via-points to be planned.
        goal = PlanGoal()
        goal.target = Point(x=random.uniform(0, self.environment_size[0]),
                            y=random.uniform(0, self.environment_size[1]))
        # Invoke the planner action server.
        self._helper.planner_client.send_goal(goal)
        rospy.loginfo(anm.tag_log('Planning to go in a new random position...', LOG_TAG))
        # Wait for the action server computation and listen possible incoming stimulus.
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the planning action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority.
                    self._helper.planner_client.cancel_goals()
                    return TRANS_RECHARGING
                # If the user calls the robot, then cancel the planning action server and take the `start_interaction` transition.
                if self._helper.should_interaction_start():  # Lower priority.
                    self._helper.planner_client.cancel_goals()
                    return TRANS_START_INTERACTION
                # If the planner finishes its computation, then take the `planned_to_random_pose` transition.
                if self._helper.planner_client.is_done():
                    userdata.random_plan = self._helper.planner_client.get_results().via_points
                    return TRANS_PLANNED_TO_RANDOM_POSE
                # Note that if unexpected stimulus comes from the other nodes of the architecture through the
                # `self._helper` class, then this state will not take any transitions. This is equivalent to have a
                # loop-like transition in the behavioural UML diagram for all the other stimulus except
                # `TRANS_RECHARGING`, `TRANS_START_INTERACTION` and `TRANS_PLANNED_TO_RANDOM_POSE`.
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)


# The definition of the inner state of `NORMAL` concerning motion controlling to reach a random position.
class GoToRandomPose(State):
    # Construct this class, i.e., initialise this state.
    def __init__(self, interface_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        # Also, get the `random_plan` variable, which is computed by the `PLAN_TO_RANDOM_POSE` state.
        State.__init__(self, outcomes=[TRANS_WENT_RANDOM_POSE, TRANS_RECHARGING, TRANS_START_INTERACTION],
                       input_keys=[VAR_RANDOM_PLAN])

    # Define the function performed each time a transition is such to enter in this state.
    def execute(self, userdata):
        # Get the plan to a random position computed by the `PLAN_TO_RANDOM_POSE` state.
        plan = userdata.random_plan
        # Start the action server for moving the robot through the planned via-points.
        goal = ControlGoal(via_points=plan)
        self._helper.controller_client.send_goal(goal)
        rospy.loginfo(anm.tag_log('Following the plan to reach a random position...', LOG_TAG))
        # Wait for the action server computation and listen possible incoming stimulus.
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.controller_client.cancel_goals()
                    return TRANS_RECHARGING
                # If the user calls the robot, then cancel the control action server and take the `start_interaction` transition.
                if self._helper.should_interaction_start():  # Lower priority
                    self._helper.controller_client.cancel_goals()
                    return TRANS_START_INTERACTION
                # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if self._helper.controller_client.is_done():
                    return TRANS_WENT_RANDOM_POSE
                # Note that if unexpected stimulus comes from the other nodes of the architecture through the
                # `self._helper` class, then this state will not take any transitions. This is equivalent to have a
                # loop-like transition in the behavioural UML diagram for all the other stimulus except
                # `TRANS_RECHARGING`, `TRANS_START_INTERACTION` and `TRANS_WENT_RANDOM_POSE`.
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)


# The definition of the inner state of `INTERACT` to plan the via-points to reach the user.
class PlanToUser(State):
    # Construct this class, i.e., initialise this state.
    def __init__(self, interface_helper, user_pose):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        # Get the user position from the `main` function.
        self._user_pose = user_pose
        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        # Also, set the `to_user_plan` variable, which will be eventually computed in the `execute` function and passed to the `GO_TO_USER` state.
        State.__init__(self, outcomes=[TRANS_PLANNED_TO_USER, TRANS_RECHARGING, TRANS_STOP_INTERACTION],
                       output_keys=[VAR_TO_USE_PLAN])

    # Define the function performed each time a transition is such to enter in this state.
    def execute(self, userdata):
        # Define the planning goal as the user's position.
        goal = PlanGoal(target=self._user_pose)
        # Start the planner action server.
        self._helper.planner_client.send_goal(goal)
        rospy.loginfo(anm.tag_log('Planning to go toward the user...', LOG_TAG))
        # Wait for the action server computation and listen possible incoming stimulus.
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the planning action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.planner_client.cancel_goals()
                    return TRANS_RECHARGING
                # If the user greets the robot, then cancel the planning action server and take the `stop_interaction` transition.
                if self._helper.should_interaction_end():  # Lower priority
                    self._helper.planner_client.cancel_goals()
                    return TRANS_STOP_INTERACTION
                # If the planner finishes its computation, then take the `planned_to_user` transition.
                if self._helper.planner_client.is_done():
                    userdata.to_user_plan = self._helper.planner_client.get_results().via_points
                    return TRANS_PLANNED_TO_USER
                # Note that if unexpected stimulus comes from the other nodes of the architecture through the
                # `self._helper` class, then this state will not take any transitions. This is equivalent to have a
                # loop-like transition in the behavioural UML diagram for all the other stimulus except
                # `TRANS_RECHARGING`, `TRANS_STOP_INTERACTION` and `TRANS_PLANNED_TO_USER`.
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)


# The definition of the inner state of `INTERACT` to control the robot toward the via-points to reach the user.
class GoToUser(State):
    # Construct this class, i.e., initialise this state.
    def __init__(self, interface_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        # Also, get the `to_user_plan` variable, which is computed by the `PLAN_TO_USER` state.
        State.__init__(self, outcomes=[TRANS_WENT_TO_USER, TRANS_RECHARGING, TRANS_STOP_INTERACTION],
                       input_keys=[VAR_TO_USE_PLAN])

    # Define the function performed each time a transition is such to enter in this state.
    def execute(self, userdata):
        # Get the plan toward the user's position computed by the `PLAN_TO_USER` state.
        plan = userdata.to_user_plan
        # Start the action server for moving the robot through the planned via-points.
        goal = ControlGoal(via_points=plan)
        self._helper.controller_client.send_goal(goal)
        rospy.loginfo(anm.tag_log('Following the plan to reach the user...', LOG_TAG))
        # Wait for the action server computation and listen possible incoming stimulus.
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.controller_client.cancel_goals()
                    return TRANS_RECHARGING
                # If the user greets the robot, then cancel the control action server and take the `stop_interaction` transition.
                if self._helper.should_interaction_end():  # Lower priority
                    self._helper.controller_client.cancel_goals()
                    return TRANS_STOP_INTERACTION
                # If the controller finishes its computation, then take the `went_to_user` transition.
                if self._helper.controller_client.is_done():
                    return TRANS_WENT_TO_USER
                # Note that if unexpected stimulus comes from the other nodes of the architecture through the
                # `self._helper` class, then this state will not take any transitions. This is equivalent to have a
                # loop-like transition in the behavioural UML diagram for all the other stimulus except
                # `TRANS_RECHARGING`, `TRANS_STOP_INTERACTION` and `TRANS_WENT_TO_USER`.
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)


# The definition of the inner state of `INTERACT` to plan the via-points to reach the location pointed by the user.
class PlanToGesture(State):
    # Construct this class, i.e., initialise this state.
    def __init__(self, interface_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        # Also, set the `to_gesture_plan` variable, which will be eventually computed in the `execute` function and passed to the `GO_TO_GESTURE` state.
        State.__init__(self, outcomes=[TRANS_PLANNED_TO_GESTURE, TRANS_RECHARGING, TRANS_STOP_INTERACTION],
                       output_keys=[VAR_TO_GESTURE_PLAN])

    # Define the function performed each time a transition is such to enter in this state.
    def execute(self, userdata):
        # Reset the state of previous stimuli to assure that only the new stimulus (especially gestures) are considered.
        self._helper.planner_client.reset_client_states()
        # Wait for a user's gesture.
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the planner action server has not been already started.
                if not self._helper.planner_client.is_running():
                    # Get the last user's gesture.
                    gesture = self._helper.consume_gesture()
                    # If the users' gesture exists.
                    if gesture is not None:
                        # Start the planning action server for computing the via-points toward the pointed gesture.
                        goal = PlanGoal(target=gesture)
                        self._helper.planner_client.send_goal(goal)
                        rospy.loginfo(anm.tag_log('Planning to go toward the pointed location...', LOG_TAG))
                # If the battery is low, then cancel the planning action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.planner_client.cancel_goals()
                    return TRANS_RECHARGING
                # If the user greets the robot, then cancel the planning action server and take the `stop_interaction` transition.
                if self._helper.should_interaction_end():  # Lower priority
                    self._helper.planner_client.cancel_goals()
                    return TRANS_STOP_INTERACTION
                # If the planner finishes its computation, then take the `planned_to_gesture` transition.
                if self._helper.planner_client.is_done():
                    userdata.to_gesture_plan = self._helper.planner_client.get_results().via_points
                    return TRANS_PLANNED_TO_GESTURE
                # Note that if unexpected stimulus comes from the other nodes of the architecture through the
                # `self._helper` class, then this state will not take any transitions. This is equivalent to have a
                # loop-like transition in the behavioural UML diagram for all the other stimulus except
                # `TRANS_RECHARGING`, `TRANS_STOP_INTERACTION` and `TRANS_PLANNED_TO_GESTURE`.
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)


# The definition of the inner state of `INTERACT` to move toward the via-points for reaching the location pointed by the user.
class GoToGesture(State):
    # Construct this class, i.e., initialise this state.
    def __init__(self, interface_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        # Also, get the `to_gesture_plan` variable, which is computed by the `PLAN_TO_GESTURE` state.
        State.__init__(self, outcomes=[TRANS_WENT_TO_GESTURE, TRANS_RECHARGING, TRANS_STOP_INTERACTION],
                       input_keys=[VAR_TO_GESTURE_PLAN])

    # Define the function performed each time a transition is such to enter in this state.
    def execute(self, userdata):
        # Get the plan toward the position pointed by the user, as computed by the `PLAN_TO_GESTURE` state.
        plan = userdata.to_gesture_plan
        # Start the action server for moving the robot through the planned via-points.
        goal = ControlGoal(via_points=plan)
        self._helper.controller_client.send_goal(goal)
        rospy.loginfo(anm.tag_log('Following the plan to reach the pointed location...', LOG_TAG))
        # Wait for the action server computation and listen possible incoming stimulus.
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.controller_client.cancel_goals()
                    return TRANS_RECHARGING
                # If the user greets the robot, then cancel the control action server and take the `stop_interaction` transition.
                if self._helper.should_interaction_end():  # Lower priority
                    self._helper.controller_client.cancel_goals()
                    return TRANS_STOP_INTERACTION
                # If the controller finishes its computation, then take the `went_to_gesture` transition, which is related to the `repeat` transition.
                if self._helper.controller_client.is_done():
                    return TRANS_WENT_TO_GESTURE
                # Note that if unexpected stimulus comes from the other nodes of the architecture through the
                # `self._helper` class, then this state will not take any transitions. This is equivalent to have a
                # loop-like transition in the behavioural UML diagram for all the other stimulus except
                # `TRANS_RECHARGING`, `TRANS_STOP_INTERACTION` and `TRANS_WENT_TO_GESTURE`.
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)


def main():
    # Initialise this ROS node.
    rospy.init_node(anm.NODE_BEHAVIOUR, log_level=rospy.INFO)
    # Initialise an helper class to manage the interfaces with the other nodes in the architectures, i.e., it manages external stimulus.
    helper = InterfaceHelper()

    # Get the initial robot pose from ROS parameters.
    robot_pose_param = rospy.get_param(anm.PARAM_INITIAL_POSE, [0, 0])
    # Initialise robot position in the `robot_state`, as required by the plan anc control action servers.
    helper.init_robot_pose(Point(x=robot_pose_param[0], y=robot_pose_param[1]))

    # Get the user's position from ROS parameters.
    user_pose_param = rospy.get_param(anm.PARAM_USER_POSE, [0, 0])
    user_pose = Point(x=user_pose_param[0], y=user_pose_param[1])

    # Define the structure of the Finite State Machine.
    sm_main = StateMachine([])
    with sm_main:
        # Define the higher level node for the normal behaviour, i.e., when the robot moves between random points.
        sm_normal = StateMachine(outcomes=[TRANS_REPEAT, TRANS_BATTERY_LOW, TRANS_CALLED])
        with sm_normal:
            # Define the inner state to plan the via-points toward a random position.
            StateMachine.add(STATE_PLAN_TO_RANDOM_POSE, PlanToRandomPose(helper),
                             transitions={TRANS_PLANNED_TO_RANDOM_POSE: STATE_GO_TO_RANDOM_POSE,
                                          TRANS_RECHARGING: TRANS_BATTERY_LOW,
                                          TRANS_START_INTERACTION: TRANS_CALLED})
            # Define the inner state to move to a random position.
            StateMachine.add(STATE_GO_TO_RANDOM_POSE, GoToRandomPose(helper),
                             transitions={TRANS_WENT_RANDOM_POSE: TRANS_REPEAT,
                                          TRANS_RECHARGING: TRANS_BATTERY_LOW,
                                          TRANS_START_INTERACTION: TRANS_CALLED})
        # Add the sub Finite State Machine to the main Finite State Machine concerning the `normal` behaviour.
        StateMachine.add(STATE_NORMAL, sm_normal,
                         transitions={TRANS_REPEAT: STATE_NORMAL,
                                      TRANS_BATTERY_LOW: STATE_RECHARGING,
                                      TRANS_CALLED: STATE_INTERACT})

        # Define the higher level node for the interaction behaviour, i.e., when the robot moves to the user first,
        # and then to the location pointed by the user.
        sm_interact = StateMachine(outcomes=[TRANS_REPEAT, TRANS_BATTERY_LOW, TRANS_GREETED])
        with sm_interact:
            # Define the inner state to plan the via-points toward the user.
            StateMachine.add(STATE_PLAN_TO_USER, PlanToUser(helper, user_pose),
                             transitions={TRANS_PLANNED_TO_USER: STATE_GO_TO_USER,
                                          TRANS_RECHARGING: TRANS_BATTERY_LOW,
                                          TRANS_STOP_INTERACTION: TRANS_GREETED})
            # Define the inner state to move toward the user.
            StateMachine.add(STATE_GO_TO_USER, GoToUser(helper),
                             transitions={TRANS_WENT_TO_USER: STATE_PLAN_TO_GESTURE,
                                          TRANS_RECHARGING: TRANS_BATTERY_LOW,
                                          TRANS_STOP_INTERACTION: TRANS_GREETED})
            # Define the inner state to plan the via-points toward the location pointed by the user.
            StateMachine.add(STATE_PLAN_TO_GESTURE, PlanToGesture(helper),
                             transitions={TRANS_PLANNED_TO_GESTURE: STATE_GO_TO_GESTURE,
                                          TRANS_RECHARGING: TRANS_BATTERY_LOW,
                                          TRANS_STOP_INTERACTION: TRANS_GREETED})
            # Define the inner state to move toward the location pointed by the user.
            StateMachine.add(STATE_GO_TO_GESTURE, GoToGesture(helper),
                             transitions={TRANS_WENT_TO_GESTURE: TRANS_REPEAT,
                                          TRANS_RECHARGING: TRANS_BATTERY_LOW,
                                          TRANS_STOP_INTERACTION: TRANS_GREETED})
        # Add the sub Finite State Machine to the main Finite State Machine concerning the `interaction` behaviour.
        StateMachine.add(STATE_INTERACT, sm_interact,
                         transitions={TRANS_REPEAT: STATE_INTERACT,
                                      TRANS_BATTERY_LOW: STATE_RECHARGING,
                                      TRANS_GREETED: STATE_NORMAL})

        # Define the node where the robot's will recharge itself.
        StateMachine.add(STATE_RECHARGING, Recharging(helper),
                         transitions={TRAMS_RECHARGED: STATE_NORMAL})

    # Create and start the introspection server for visualizing the finite state machine.
    sis = smach_ros.IntrospectionServer('sm_introspection', sm_main, '/SM_ROOT')
    sis.start()

    # Execute the state machine. Note that the `outcome` value of the main Finite State Machine is not used.
    outcome = sm_main.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


# The function that get executed at start time.
if __name__ == '__main__':
    main()  # Initialise and start the ROS node.

