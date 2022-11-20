# ROS Smach based exercise with ontology map 
## Experimental Robotics Laboratory course, University of Genoa.  
---

**Author**: *[Bauyrzhan Zhakanov](https://github.com/bzwayne)*, [s5218116@studenti.unige.it](s5218116@studenti.unige.it)

**Course Instructors**: *[Carmine Recchiuto](https://github.com/CarmineD8)* and *[Luca Buoncomapgni](https://github.com/buoncubi)*

## Introduction

This repository contains **ROS-based** software that simulates a behavioural architecture of mobile robot travelling through the rooms. 
The assignment is developed by using [ROS SMACH](http://wiki.ros.org/smach) state machine and builds an ontology with [armor_py_api](https://github.com/EmaroLab/armor_py_api/blob/master/scripts/armor_api/armor_manipulation_client.py) provided by Emaro Lab of University of Genoa, Italy. Everything briefly explained in **Scenario** section and in the documentation. The task is developed under the supervision of course intructors of Experimental Robotics at University of Genoa, Italy.


## Scenario

The scenario has a robot with the following behaviour:

1. Robot is in room **E** and loads a map
2. **If** map is loaded, **then** robot starts its movement in the corridors, **otherwise**, it stays inside the room E.
3. **If** the battery of the robot is full, **then** robot moves to the rooms, **otherwise**, it goes to the room E to charge.
4. After fully charging the battery, robot goes back to the rooms.

The scheme of the task is shown below. 

### Synchronization

An important aspect of this exercise is the synchronization between the robot and the user. In
particular, the user should never wait for the robot to complete an action, except when it is
recharging its battery. This implies that the Finite States Machine should never be blocked. In
other words, the Finite States Machine should process speech-based, gesture-based, and 
battery-based events as soon as they occur. Furthermore, we consider that the Finite States 
Machine does not allow for concurrent states.

## Project Structure

### Package List

This repository contains a ROS package named `arch_skeleton` that includes the following resources.
 - [CMakeList.txt](CMakeList.txt): File to configure this package.
 - [package.xml](package.xml): File to configure this package.
 - [setup.py](setup.py): File to `import` python modules from the `utilities` folder into the 
   files in the `script` folder. 
 - [launcher/](launcher/): Contains the configuration to launch this package.
    - [manual_sense.launch](launcher/manual_sense.launch): It launches this package allowing 
       for keyboard-based interface.
    - [random_sense.launch](launcher/random_sense.launch): It launches this package with 
      random-based stimulus.
 - [msg/](msg/): It contains the message exchanged through ROS topics.
    - [Gesture.msg](msg/Gesture.msg): It is the message representing detected pointing gestures.
    - [Speech.msg](msg/Speech.msg): It is the message representing speech-based commands.
    - [Point.msg](msg/Point.msg): It is the message representing a 2D point.
 - [srv/](srv/): It Contains the definition of each server used by this software.
    - [GetPose.srv](srv/GetPose.srv): It defines the request and response to get the current 
      robot position.
    - [SetPose.srv](srv/SetPose.srv): It defines the request and response to set the current 
      robot position.
 - [action/](action/): It contains the definition of each action server used by this software.
    - [Plan.action](action/Plan.action): It defines the goal, feedback and results concerning 
      motion planning.
    - [Control.action](action/Control.action): It defines the goal, feedback and results 
      concerning motion controlling.
 - [scripts/](scripts/): It contains the implementation of each software components.
    - [speech.py](scripts/speech.py): It is a dummy implementation of the speech-based 
      commands detection algorithm.
    - [gesture.py](scripts/gesture.py): It is a dummy implementation of the gesture-based
      commands detection algorithm.
    - [robot_state.py](scripts/robot_state.py): It implements the robot state including:
      current position, and battery level.
    - [planner.py](scripts/planner.py): It is a dummy implementation of a motion planner.
    - [controller.py](scripts/controller.py): It is a dummy implementation of a motion 
      controller.
 - [utilities/arch_skeleton/](utilities/arch_skeleton/): It contains auxiliary python files, 
   which are exploited by the files in the `scripts` folder.
    - [architecture_name_mapper.py](scripts/architecture_name_mapper.py): It contains the name 
      of each *node*, *topic*, *server*, *actions* and *parameters* used in this architecture.
 - [diagrams/](diagrams/): It contains the diagrams shown below in this README file.

### Dependencies

The software exploits [roslaunch](http://wiki.ros.org/roslaunch) and 
[rospy](http://wiki.ros.org/rospy) for using python with ROS. Rospy allows defining ROS nodes, 
services and related messages.

Also, the software uses [actionlib](http://wiki.ros.org/actionlib/DetailedDescription) to define
action servers. In particular, this software is based on 
[SimpleActionServer](http://docs.ros.org/en/jade/api/actionlib/html/classactionlib_1_1simple__action__server_1_1SimpleActionServer.html#a2013e3b4a6a3cb0b77bb31403e26f137).
Thus, you should use the [SimpleActionClient](https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html)
to solve the exercise.

The Finite States Machine that you will implement based on the software components provided in 
this repository should be based on [SMACH](http://wiki.ros.org/smach). You can check the 
[tutorials](http://wiki.ros.org/smach/Tutorials) related to SMACH, for an overview of its 
functionalities. In addition, you can exploit the [smach_viewer](http://wiki.ros.org/smach_viewer)
node to visualize and debug the implemented Finite States Machine.

## Software Components

It follows the details of each software component implemented in this repository, which is available
in the `scripts/` folder.

### The `speech-eval` Node, its Message and Parameters

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/speech-eval.png" width="600">

The `speech-eval` node is a simple publisher that produces `Speech` messages in the 
`/sensor/speech` topic. Each generated message has two fields: a time `stamp` and a 
`command`. The latter is a string equal to `"Hello"`, when the interaction should start, or
`"Bye"` when the interaction should end. Such keywords can be configured through the 
`config/speech_commands` parameter (detailed below).

This node allows publishing messages from the keyboard or in a randomized manner, and this can 
be chosen with the `test/random_sense/active` parameter detailed below. When random messages are
published, the `test/random_sense/speech_time` parameter is used to delay the generated 
commands, which might not always be consistent for accounting perception errors (e.g., the command 
`"???"` is sometimes published).

To observe the behaviour of the `speech-eval` node you can run the following commands.
```bash
roscore
# Open a new terminal.
rosparam set config/speech_commands '["Hello", "Bye"]'
rosrun arch_skeleton speech.py 
# Open a new terminal
rostopic echo /sensor/speech
```
With `rosparam` you might also set the `test/random_sense/active` and  
`test/random_sense/speech_time` parameters (detailed below) to see how messages are differently
published.

### The `gesture-eval` Node, its Message and Parameters

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/gesture-eval.png" width="600">

The `gesture-eval` node is a simple publisher that produces `Gesture` messages in the 
`sensor/gesture` topic. Each generated message has two fields: a time `stamp` and a `coordinate`.
The latter is of type `Point` (defined in the `msg/` folder), which has two `float` sub-fields, 
i.e., `x` and `y`.

This node allows publishing messages from the keyboard or in a randomized manner, and this can be
chosen with the `test/random_sense/active` parameter detailed below. When random messages are
published, the `test/random_sense/gesture_time` parameter is used to delay the generated 
messages, which encode a `coordinate` with random `x` and `y` based on the 
`config/environment_size` parameter detailed below. To simulate possible perception error, this 
node might generate `coordinates` that are out of the environment.


To observe the behaviour of the `gesture-eval` node you can run the following commands.
```bash
roscore
# Open a new terminal.
rosparam set config/environment_size '[10,10]'
rosrun arch_skeleton gesture.py 
# Open a new terminal
rostopic echo /sensor/gesture 
```
With `rosparam` you might also set the `test/random_sense/active` and  
`test/random_sense/gesture_time` parameters (detailed below) to see how messages are differently 
published.

### The `robot-state` Node, its Messages and Parameters

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/robot-state.png" width="900">

The `robot-state` is a node that encodes the knowledge shared among the other components, and it 
implements two services (i.e., `state/set_pose` and `state/get_pose`) and a publisher (i.e., 
`state/battery_low`). 

The services allow setting and getting the current robot position, which is shared between the 
`planner` and the `controller` as detailed below. In particular, the `state/set_pose` requires a 
`Point` to be set and returns nothing, while the `state/get_pose` requires nothing and return
a `Point` encoding the robot pose. 

Note that a client should set the initial robot position when the architecture startups. 

Also, note that, for more general architectures, the robot pose might be published in a topic, 
instead of being provided through a server. This is because many components might require the 
current robot pose, which might change frequently. However, this example does not consider such a case.

Moreover, the `robot-state` also implements a publisher of `Boolean` messages into the `state/
battery_low` topic. This message is published when the batter changes state. We consider two 
possible states: low battery (i.e., `True` is published) and recharged (i.e., `False` is 
published).

The battery-related publisher allows publishing messages from the keyboard or in a randomized 
manner, and this can be chosen with the `test/random_sense/active` parameter detailed below. 
When random messages are published, the `test/random_sense/battery_time` parameter is used to 
delay the published messages.

To observe the behaviour of the `robot-state` node you can run the following commands.
```bash
roscore
# Open a new terminal.
rosrun arch_skeleton robot-state.py 
# Open a new terminal
rostopic echo /state/battery_low 
# Open a new terminal 
rosservice call /state/set_pose "pose: { x: 1.11,  y: 2.22}"
rosservice call /state/get_pose "{}" 
```
With `rosparam` you might also set the `test/random_sense/active` and  
`test/random_sense/battery_time` parameters (detailed below) to see how messages are 
differently published.

### The `planner` Node, its Message and Parameters

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/planner.png" width="900">

The `planner` node implements an action server named `motion/planner`. This is done by the 
means of the `SimpleActionServer` class based on the `Plan` action message. This action server 
requires the `state/get_pose/` service of the `robot-state` node, and a `target` point given as goal.

Given the current and target points, this component returns a plan as a list of `via_points`, 
which are randomly generated for simplicity. The number of `via_points` can be set with the 
`test/random_plan_points` parameter addressed below. Moreover, each `via_point` is provided 
after a delay to simulate computation, which can be tuned through the `test/random_plan_time` 
parameter. When a new `via_points` is generated, the updated plan is provided as `feedback`. When
all the `via_points` have been generated, the plan is provided as `results`.

While the picture above shows the actual implementation of the action server, you should not 
interact with it through the shown topics directly. Instead, you should use a 
[SimpleActionClient](https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html), 
for instance, as:
```python
import actionlib
from arch_skeleton.msg import PlanAction, PlanGoal
...
# Initialize the client and, eventually, wait for the server.
client = actionlib.SimpleActionClient('motion/planner', PlanAction)
client.wait_for_server()
...
def feedback_callback(feedback):
    # Do something when feedback is provided.
    pass  
...
def done_callback(status, results):
    # Do something when results are provided.
    pass  
...
# Send a new `goal`, which is a message of type `PlanGoal`.
client.send_goal(goal, done_cb = done_callback, feedback_cb = feedback_callback)
...
# Get the action server state.
client.get_state()
...
# Cancel all goals (or the current goal only, i.e., `client.cancel_goal()`).
client.cancel_all_goals()
```

To observe the behaviour of the `planner` you can run the following commands.
``` bash
roscore
# Open a new terminal.
rosrun arch_skeleton robot_states.py
# Open a new terminal.
rosservice call /state/set_pose "pose: { x: 0.11,  y: 0.22}"
rosparam set config/environment_size '[10,10]'
rosrun arch_skeleton planner.py
# Open a new terminal.
rosrun actionlib_tools axclient.py /motion/planner
```
Then, a GUI should appear. Set the goal you want to reach and hit the send button. Eventually, you
can cancel the goal as well. Also, you can change the `test/random_plan_points` and 
`test/random_plan_time` parameters (detailed below) to tune the behaviour of the planner.

The last command of the above fragment of code requires the `actionlib-tools` package, which can
be installed done by typing:
```bash
sudo apt update
sudo apt install ros-noetic-actionlib-tools
```


### The `controller` Node, its Message and Parameters

<img src="https://github.com/buoncubi/arch_skeleton/blob/main/diagrams/controller.png" width="900">

The `controller` node implements an action server named `motion/controller`. This is done by 
the means of the `SimpleActionServer` class based on the `Control` action message. This action 
server requires the `state/set_pose/` service of the `robot-state` node and a plan given as a 
list of `via_points` by the `planner`.

Given the plan and the current robot position, this component iterates for each planned 
`via_point` and waits to simulate the time spent moving the robot to that location. The 
waiting time can be tuned through the `test/random_motion_time` parameter detailed below. Each 
time a `via_point` is reached the `state/set_pose` service is invoked, and a `feedback` is 
provided. When the last `via_point` is reached, the action service provides a result by 
propagating the current robot position, which has been already updated through the 
`state/set_pose` service.

Similarly to the `planner` above, instead of using the raw topics, you can rely on a 
`SimpleActionClient`, which should be instantiated as:
```python
client = actionlib.SimpleActionClient('motion/controller', ControlAction)
```
This client would accept goals of type `ControlGoal`.

To observe the behaviour of the `controller` you can run the following commands.
``` bash
roscore
# Open a new terminal.
rosrun arch_skeleton robot_states.py
# Open a new terminal.
rosservice call /state/set_pose "pose: { x: 0.11,  y: 0.22}"
#rosparam set config/environment_size '[10,10]'
rosrun arch_skeleton controller.py
# Open a new terminal.
rosrun actionlib_tools axclient.py /motion/controller
```
Then, the same GUI seen for the `planner` should appear. In this case, you can test goals 
formatted as:
```yaml
via_points: 
  - 
    x: 0.109999999404
    y: 0.219999998808
  - 
    x: 3.61638021469
    y: 5.05489301682
  - 
    x: 0.292526483536
    y: 6.59786701202
  - 
    x: 4.33828830719
    y: 7.73262834549
  - 
    x: 6.0
    y: 6.0
```
You can also change the `test/random_motion_time` parameter (detailed below) to tune
the behaviour of the controller.

## Launching the Software

This software has been based on ROS Noetic, and it has been developed with this Docker-based
[environment](https://hub.docker.com/repository/docker/carms84/exproblab), which already 
provides the required dependencies listed above. 

### Installation

Follow these steps to install the software.
 - Clone this repository inside your ROS workspace (which should be sourced in your `.bashrc`).
 - Run `chmod +x <file_name>` for each file inside the `scripts` folder.
 - Run `catkin_make` from the root of your ROS workspace.
 - Install `xterm` by entering the command `sudo apt install -y xterm`.

### Launchers

Use the following command to launch the software with a keyboard-base interface for speech, 
gesture and battery level.
```bash
roslaunch arch_skeleton manual_sense.launch
```

Use the following command to launch the software with randomized stimulus, 
i.e., speech, gesture and battery level.
```bash
roslaunch arch_skeleton random_sense.launch
```

Note that the architecture launched with these commands does nothing, except generate stimulus, 
i.e., battery level, speech and gesture commands. In order to invoke the motion planner 
and controller, you need to implement the Finite States Machine as described below.

Check the `roslouch` outcome to get the path where logs are stored. usually, it is `~/.ros/log/`.
That folder should also contain a link to the `latest` produced log.

### ROS Parameters

This software requires the following ROS parameters.
 
 - `config/environment_size`: It represents the environment boundaries as a list of two float
   numbers, i.e., `[x_max, y_max]`. The environment will have the `x`-th coordinate spanning
   in the interval `[0, x_max)`, while the `y`-th coordinate in `[0, y_max)`.

 - `config/user_pose`: It represents the user's position as a list of two float numbers,
   i.e., `[x, y]`. This pose should be within the `environmet_size`.

 - `config/speech_commands`: It defines the keywords that the user can say to start and end
   the interaction. It must be a list made of two strings (e.g., `["Hello", "Bye"]`) that define
   the keyword to start and end the interaction, respectively.

 - `state/initial_pose`: It represents the initial robot pose as a list of two float numbers, 
   i.e., `[x, y]`. This pose should be within the `environmet_size`.

 - `test/random_plan_points`: It represents the number of via points in a plan, and it should be
   a list of two integer numbers `[min_n, max_n]`. A random value within such an interval will be
   chosen to simulate plans of different lengths.

 - `test/random_plan_time`: It represents the time required to compute the next via point of the 
   plan, and it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. 
   A random value within such an interval will be chosen to simulate the time required to 
   compute the next via points.

 - `test/random_motion_time`: It represents the time required to reach the next via point, and 
   it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. A random
   value within such an interval will be chosen to simulate the time required to reach the next 
   via points. 

 - `test/random_sense/active`: It is a boolean value that activates (i.e., `True`) or 
   deactivates (`False`) the random-based generation of stimulus (i.e., speech, gesture and 
   battery level). If this parameter is `True`, then the three parameters below are also 
   required.  If it is `False`, then the three parameters below are not used.
 

In addition, the `random_sense.launch` also requires the following three parameters. This 
occurs because `test/random_sense/active` has been set to `True`.

 - `test/random_sense/gesture_time`: It indicates the time passed within two randomly generated 
   pointing gestures. It should be a list of two float numbers, i.e., `[min_time, max_time]` in 
   seconds, and the time passed between gestures will be a random value within such an interval.

 - `test/random_sense/speech_time`: It indicates the time passed within two randomly generated
   commands based on speech. It should be a list of two float numbers, i.e., 
   `[min_time, max_time]` in seconds, and the time passed between speech-based commands will be 
   a random value within such an interval.

 - `test/random_sense/battery_time`: It indicates the time passed within battery state changes 
   (i.e., low/high). It should be a list of two float numbers, i.e., `[min_time, max_time]` in 
   seconds, and the time passed between changes in battery levels will be a random value within 
   such an interval.

---

## The exercise

Develop a Finite States Machine based on the SMACH library that implements the behaviour of the 
robot. Use only the software components provided in this repository to develop such a Finite 
States Machine.

Debug your implementation with the `manual_sense.launch` configuration. Then, test it in a log 
term running through the `random_sense.launch` configuration. 

Optionally, write a script that automatically checks if an anomalous behaviour occurs while 
using the `random_sense.launch` configuration.

### The Finite States Machine

The Finite States Machine to be developed should implement the scenario introduced at the 
beginning of this README file. 

In addition, the Finite States Machine should have the following functionalities.
 - It sets, in the `robot-state` node, the initial robot pose given through the 
   `state/initial_pose` parameter.
 - It subscribes to the `sensor/speech` topic to get speech-based commands.
 - It subscribes to the `sensor/gesture` topic to get pointing gestures.
 - It subscribes to the `state/battery_low` topic to get the battery state.
 - It processes each speech, gesture, and battery message as soon as they are provided.
 - It uses the `planner` action server and cancels it if necessary.
 - It uses the `controller` action server and cancels it if necessary.

Note that, in Python, ROS subscribes run on separate threads. Thus, you 
need to use `mutexes` to assure data consistency across concurrent threads.

---
