cmake_minimum_required(VERSION 2.8)
project(mbzirc_birck_pose)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport sensor_msgs pcl_conversions pcl_ros std_msgs message_generation tf)

find_package(PCL REQUIRED QUIET COMPONENTS common)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV)
find_package(Boost)
find_package(Eigen3)

add_message_files(
  FILES
  block_pose.msg
  flag.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${Eigen_INCLUDE_DIRS})

# add the publisher example
add_executable(estimate_pose_2d src/estimate_pose_2d.cpp)
add_dependencies(estimate_pose_2d ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(estimate_pose_2d ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES})

#add_executable(rgbd_demo src/rgbd_demo.cpp)
#add_dependencies(rgbd_demo ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(rgbd_demo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES})
