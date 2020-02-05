cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_challenge2)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    message_generation
    mbzirc_msgs
)
find_package(Boost REQUIRED system filesystem date_time thread)

#add_service_files(
#   FILES
#   go_to_brick.srv
#   place_in_container.srv
#)


#set(THIS_PACKAGE_INCLUDE_DIRS
#  doc/interactivity/include
#)
# Generate messages in the 'msg' folder
add_message_files(
   FILES
   BoxInfo.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)




###########
## Build ##
###########

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

#add_subdirectory(doc/kinematics)
#add_subdirectory(doc/robot_model_and_robot_state)
#add_subdirectory(doc/planning)
#add_subdirectory(doc/planning_scene)
#add_subdirectory(doc/planning_scene_ros_api)
#add_subdirectory(doc/motion_planning_api)
#add_subdirectory(doc/motion_planning_pipeline)
#add_subdirectory(doc/visualizing_collisions)
#add_subdirectory(doc/move_group_interface)
#add_subdirectory(doc/move_group_python_interface)
#add_subdirectory(doc/state_display)
#add_subdirectory(doc/interactivity)
#add_subdirectory(doc/pick_place)
#add_subdirectory(doc/perception_pipeline)



add_executable(mbzirc_ur5_control src/mbzirc_ur5_control.cpp)
target_link_libraries(mbzirc_ur5_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS mbzirc_ur5_control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# add_dependencies(mbzirc_ur5_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
