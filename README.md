# DrRobot-X80SVP
Framework for DrRobot X80

Add all the directories to your catkin_ws and run catkin_make

starting x80sv node:

  | 1. x80sv_description
  
  | 2. joint_state_publisher
  
  | 3. drrobot_X80_player_node
  
  | 4. maker trajectory

  ```bash
  $ roslaunch drrobot_X80_player robot.launch
  ```

starting following node:

  | 1. robot
  
  | 2. teleop_joy
  
  | 3. move_base_topoint
  
  | 4. x80sv_config rviz
  
  | 5. move_base_topoint
  
  | 6. imu complementary_filter
  
  | 7. PDR calibration
  
  | 8. findpeak
  
  | 9. human_tracker_show
  
  | 10. skeleton_follower
  
  ```bash
  $ roslaunch drrobot_X80_player following.launch
  ```


You should be able to control the robot by W A S D. Be careful not to control it with SHIFT because it moves fast and may cause damage to the robot.

  ```bash
  $ rosrun drrobot_X80_player drrobot_X80_keyboard_teleop_node
  ˋˋˋ


