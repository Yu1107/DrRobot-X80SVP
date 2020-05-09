# DrRobot-X80SVP
Framework for DrRobot X80

Add all the directories to your catkin_ws and run catkin_make

**starting x80sv node:**
  - x80sv_description
  - joint_state_publisher
  - drrobot_X80_player_node
  - maker trajectory

  ```bash
  roslaunch drrobot_X80_player robot.launch
  ```

**starting following node:**

  - robot  
  - teleop_joy
  - move_base_topoint
  - x80sv_config rviz
  - move_base_topoint
  - imu complementary_filter
  - PDR calibration
  - findpeak
  - human_tracker_show
  - skeleton_follower

  ```bash
  roslaunch drrobot_X80_player following.launch
  ```


You should be able to control the robot by W A S D. Be careful not to control it with SHIFT because it moves fast and may cause damage to the robot.

  ```bash
  rosrun drrobot_X80_player drrobot_X80_keyboard_teleop_node
  ```


