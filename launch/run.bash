konsole -e roslaunch robot_smach launcher.launch &
sleep 20
konsole -e rosrun robot_smach fsm.py
