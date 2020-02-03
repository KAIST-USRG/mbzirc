cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)



catkin_package(
  INCLUDE_DIRS include
)


set( hearders
)

set( sources
  src/mbzirc_robot_node.cpp
)


#find_package(lcm REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
#  ${LCM_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/mbzirc_robot_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
#   lcm
)


