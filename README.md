# DrRobot-X80SVP
Framework for DrRobot X80

Add all the directories to your catkin_ws and run catkin_make
Then run these 3 commands in 3 different terminals.

ˋˋˋbash
1. start x80sv node
$ roslaunch drrobot_X80_player robot.launch

You should be able to control the robot by W A S D. Be careful not to control it with SHIFT because it moves fast and may cause damage to the robot.
ˋˋˋbash

$ rosrun drrobot_X80_player drrobot_X80_keyboard_teleop_node
ˋˋˋ


