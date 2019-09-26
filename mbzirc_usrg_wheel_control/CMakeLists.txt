cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_usrg_wheel_control)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  dynamixel_workbench_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES mbzirc_usrg_wheel_control
  CATKIN_DEPENDS dynamixel_sdk dynamixel_workbench_msgs geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs
  #DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wheel_control_node src/wheel_control.cpp)

add_dependencies(wheel_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(wheel_control_node ${catkin_LIBRARIES})

