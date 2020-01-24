# mbzirc_usrg

## How to start up the 4 wheel steering robot

### Bring up the robot
`roslaunch mbzirc_wheel_control 4_wheel_control.launch`

### Launch joystick launch file
`roslaunch mbzirc_wheel_control joy.launch`

### Turn on the odrives
```
rosservice call /back/odrive/calibrate_motors "{}" 
rosservice call /front/odrive/calibrate_motors "{}" 
```

## Publish
- DYNAMIXEL
  - `/dynamixel_state(dynamixel_workbench_msgs/DynamixelStateList)`: States of connected DYNAMIXEL
  - `/joint_states(sensor_msgs/JointState)`: Joint information about connected DYNAMIXEL
- Hub motor
  - `/front/left/raw_odom/velocity(std_msgs/Int32)`: Speeds of front left hub motor (unit: encoder counts/s, 1 rotation: 90counts)
  - `/front/right/raw_odom/velocity(std_msgs/Int32)`: Speeds of front right hub motor
  - `/back/left/raw_odom/velocity(std_msgs/Int32)`: Speeds of back left hub motor
  - `/back/right/raw_odom/velocity(std_msgs/Int32)`: Speeds of back right hub motor
  - `/front/left/raw_odom/encoder(std_msgs/Int32)`: Encoder counts of front left hub motor
  - `/front/right/raw_odom/encoder(std_msgs/Int32)`: Encoder counts of front right hub motor
  - `/back/left/raw_odom/encoder(std_msgs/Int32)`: Encoder counts of back left hub motor
  - `/back/right/raw_odom/encoder(std_msgs/Int32)`: Encoder counts of back right hub motor

## Pre-requisite
1. install universal_robot and ur_modern_driver packages following this tutorial http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial
2. install moveit package  following this tutorial http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html
3. `git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git`
4. `git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git`
5. `git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git`
6. `sudo apt install ros-kinetic-rgbd-launch`
7. Install Intel Realsense ROS package
8. Install rosserial for Arduino on ROS-Kinetic following this https://answers.ros.org/question/235620/how-to-install-rosserial-for-arduino-on-ros-kinetic/

## Installation
1. cd ~/catkin_ws/src
2. git clone https://github.com/KAIST-USRG/mbzirc_usrg.git
3. cd ~/catkin_ws && catkin_make

## To run ur5
1. Connect to UR5 arm using ur_modern_driver package

`roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.1.102 [reverse_port:=REVERSE_PORT]`

2. Launch the planning node

`roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch`

3. Launch the visualization in RVIZ (not necessary ? just to be safe)

`roslaunch ur5_moveit_config moveit_rviz.launch config:=true`

4. Launch the ur5_controller

`roslaunch mbzirc_usrg_challenge2 move_group_interface_tutorial.launch`

## To run brick pose estimation
`roslaunch mbzirc_brick_pose estimate_pose_2d.launch`

## To use magnetic control
1. find port name of adruino
`ls /dev/ttyA*`
2. If the portname is "ttyACM0" type in this command
`sudo chmod 777 /dev/ttyACM0`
3. Publish /magnet_on topic (Bool msg)
  true = magnet ON
  false = magnet OFF
