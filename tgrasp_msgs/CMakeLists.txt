cmake_minimum_required(VERSION 2.8.3)

project(tgrasp_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files( DIRECTORY msg
  FILES
  PclData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs actionlib_msgs)
