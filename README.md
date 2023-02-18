# ROS Smach based exercise with ontology map 
## Assignment I, Experimental Robotics Laboratory course, University of Genoa.  
---

**Author**: *[Bauyrzhan Zhakanov](https://github.com/bzwayne)*, [s5218116@studenti.unige.it](s5218116@studenti.unige.it)

**Course Instructors**: *[Carmine Recchiuto](https://github.com/CarmineD8)* and *[Luca Buoncomapgni](https://github.com/buoncubi)*

## Introduction

This repository contains **ROS-based** software that simulates a behavioural architecture of mobile robot travelling through the rooms. 
The assignment is developed by using [ROS SMACH](http://wiki.ros.org/smach) state machine and builds an ontology with [armor_py_api](https://github.com/EmaroLab/armor_py_api/blob/master/scripts/armor_api/armor_manipulation_client.py) provided by Emaro Lab of University of Genoa, Italy. An important aspect of this exercise is the synchronization between the robot and the user. The user should never wait for the robot to complete an action, except when it is recharging its battery.  Everything briefly explained in **Scenario** section and in the documentation. The task is developed under the supervision of course intructors of Experimental Robotics at University of Genoa, Italy.


## Scenario

### Environment

<img src="https://github.com/BZWayne/robotPatrol/blob/main/images/enviroment_ontology.png" align="left" width="250px"/>
                The scenario has a robot with the following behaviour:

                1. Robot is in room **E** and loads a map
                2. **If** map is loaded, **then** robot starts its movement in the corridors, 
                    **otherwise**, it stays inside the room E.
                3. **If** the battery of the robot is full, **then** robot moves to the rooms, 
                   **otherwise**, it goes to the room E to charge.
                4. After fully charging the battery, robot goes back to the rooms.
 <br clear="left"/>

## Project Structure

### Package List

This repository contains a ROS package named `arch_skeleton` that includes the following resources.
 - [CMakeList.txt](CMakeList.txt): File to configure this package.
 - [package.xml](package.xml): File to configure this package.
 - [setup.py](setup.py): File to `import` python modules from the `utilities` folder into the 
   files in the `script` folder.
 - [run.sh] (bash): It launches eveyrthing
 - [launcher/](launcher/): Contains the configuration to launch this package.
    - [solution.launch](launch/solution.launch): It launches this package allowing 
       for keyboard-based interface.
    - [launcher.launch](launch/launcher.launch): It launches this package with 
      random-based stimulus.
 - [msg/](msg/): It contains the message exchanged through ROS topics.
    - [Point.msg](msg/Gesture.msg): It is the float x and y based values
    - [Point.msg](msg/Point.msg): It is the message representing a 2D point.
 - [srv/](srv/): It Contains the definition of each server used by this software.
    - [GetPose.srv](srv/GetPose.srv): It defines the request and response to get the current 
      robot position.
    - [SetPose.srv](srv/SetPose.srv): It defines the request and response to set the current 
      robot position.
 - [src/](src/): It contains the implementation of each software components.
    - [fsm.py](scripts/speech.py): It is finite state machine script
    - [map.py](scripts/gesture.py): It is a topological map creator script.
    - [robot_state.py](scripts/robot_state.py): It implements the robot state including:
      current position, and battery level.
 - [utilities/arch_skeleton/](utilities/arch_skeleton/): It contains auxiliary python files, 
   which are exploited by the files in the `scripts` folder.
    - [architecture_name_mapper.py](scripts/architecture_name_mapper.py): It contains the name 
      of each *node*, *topic*, *server*, *actions* and *parameters* used in this architecture.
 - [diagrams/](diagrams/): It contains the diagrams shown below in this README file.


## Software Components

#### 1.- Armor

[Armor](https://github.com/EmaroLab/armor) was used was used to load topological map and manipulation and query classes. The finite-state machine is built in ROS environment based on [SMACH](http://wiki.ros.org/smach/Tutorials) ros-package. 

#### 2.- [Protege](https://protege.stanford.edu/)

Protégé was utilized to construct the topological map and establish the logical component. Its plug-in architecture is flexible and can be customized to develop ontology-based applications of varying complexity. By combining the results obtained from Protégé with rule systems or other problem-solving tools, developers can create a diverse array of intelligent systems.

#### 3- Arch_skeleton:

The purpose of this repository is to provide a simulation of a behavioral architecture using ROS-based software. It serves a dual objective of offering examples of ROS software and providing guidelines for robotics software architecture bootstrapping.

#### 4- State Machine:

The [SMACH](http://wiki.ros.org/smach) state machine controls the robot's state based on the topological ontology map reasoning and the robot's battery status. The following are the robot's states:

    - Load Map
    - Moving in Corridors
    - Discover Room
    - Moving for Battery
    - Charging

Each state is explained below:
1. **LoadMap**: The initial state is defined as building a semantic map for the robot. The arm of the robot is set to move along desired trajectories as defined in the robotik.cpp file using the setArmMotion(data) function. If sufficient room IDs are detected, the state will exit and return the status of map_loaded.
    
2. **MovinginCorridors**: The state where the robot moves to a desired location is defined in this code. First, the function set_base_movement_state(base_movement_state) is used to enable battery consumption. Then, the robot moves to the target room with the help of the function 
movingPose(pose). As the robot moves to its destination, the ontology is updated with the function update_ontology(now). When the robot reaches its target room, the state is exited and the function returns reached. If the battery level drops below the threshold, the function returns battery_low and the target room is cancelled.

3. **Discover Room**: The sate defines the state where the robot has arrived at the target room and begins to explore it. The robot arm's movement is enabled just like in the initial state using the setArmMotion(data) function, and then it returns discovered.

4. **MovingforBattery**: The state when the battery is low and the robot needs to recharge is defined. The function set_base_movement_state(movement_state) is used to enable battery consumption, and the robot moves towards the charger using the movingPose(pose) function. The ontology is updated during the movement to the charger using the update_ontology(time) function until the robot reaches the charger."

5. **Charging**: The state is called Charging() and it activates the robot has reached the charging station and recharges the battery. The battery level is updated using the set_battery_level(battery_level) function after a certain period of time has elapsed. The state then transitions to charged.


* Install konsule:

```bashscript
$ sudo apt-get install konsole
```

* Install smach state for ROS if not installed:
```bashscript
$ sudo apt-get install ros-<distro>-executive-smach*
$ sudo apt-get install ros-<distro>-smach-viewer
```

* Import the repository to you ROS workspace:
```bashscript
$ cd /your_workspace/src/
$ git clone https://github.com/BZWayne/robot_smach.git
$ cd ..
$ cd catkin_make
```

### Running

```bashscript
$ roscd robot_smach/launch
$ bash run.sh
``` 

### Limitations of the System:

The robot has certain limitations that need to be considered. It can only recharge in room E and is only capable of charging up to 20% of its battery capacity. Additionally, the robot consumes battery power whenever it moves, and if it remains stationary in a room, the battery is not consumed. If the robot's battery level drops to 7%, the system considers room E as the first urgent case for the robot to visit.

### Possible Technical Improvements:

To enhance the system, several technical improvements can be implemented. For example, a world can be added to the system to help visualize the robot's behavior more effectively. Additionally, walls have not been considered, but the world and the /laser_scan topic can be used to create an obstacle avoidance node. Moreover, there is no control over the robot's speed, but the /cmd_vel topic can be utilized to generate velocity.

## Author and Teachers contacts 
* Author 
  * Bauyrzhan Zhakanov, *s5218116@studenti.unige.it*
* Teachers
  * Luca Buoncompagni luca.buoncompagni@edu.unige.it
  * Carmine Recchiuto carmine.recchiuto@dibris.unige.it

