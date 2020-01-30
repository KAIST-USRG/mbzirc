cmake_minimum_required(VERSION 2.8.3)
project(mission_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  actionlib
  move_base_msgs
  message_generation
  geometry_msgs
  dynamixel_sdk
  dynamixel_workbench_msgs
)

add_definitions(-std=c++11)# -m64) #-Wall)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
add_service_files(
  FILES
  test.srv
  ugv_move.srv
  ur_move.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
   CATKIN_DEPENDS
   roscpp
   std_msgs
#  DEPENDS system_lib
   sensor_msgs
   geometry_msgs
   dynamixel_sdk
   dynamixel_workbench_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
	../include
)



add_executable(mission_manager_node src/mission_manager_v2.cpp)

target_link_libraries(
	mission_manager_node
        ${catkin_LIBRARIES}
)

add_dependencies(mission_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})





