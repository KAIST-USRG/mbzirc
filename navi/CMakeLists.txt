cmake_minimum_required(VERSION 2.8.3)
project(navi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  genmsg
  message_generation
  nav_msgs
  geometry_msgs
  tf
  tf2
  tf2_ros
)

add_message_files(
  FILES
  #rio_to_pc.msg
  test.msg
)

find_package(OpenCV REQUIRED)

generate_messages(DEPENDENCIES std_msgs nav_msgs geometry_msgs)

set( sources
  src/main.cpp
  src/c_ekf.cpp
  src/CoordinateConv.cpp
)

set( headers
  src/main.h
  src/common_def.h
  src/c_ekf.h
  src/CoordinateConv.h
)

catkin_package(
  LIBRARIES navi
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime nav_msgs geometry_msgs tf tf2 tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${headers} ${sources})
add_executable(angle src/angle.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node navi_generate_messages_cpp)

#controller
add_executable(controller_node src/control.cpp)
add_executable(controller_node_3 src/control_3.cpp)
add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(controller_node_3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_node ${catkin_LIBRARIES})
target_link_libraries(controller_node_3 ${catkin_LIBRARIES})





