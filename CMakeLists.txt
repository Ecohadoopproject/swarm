cmake_minimum_required(VERSION 3.0.2)
project(swarm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  laser_geometry
  sensor_msgs
  message_filters
)
add_message_files(
  FILES
  ErrorMarker.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS roscpp rospy
 DEPENDS system_lib
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


