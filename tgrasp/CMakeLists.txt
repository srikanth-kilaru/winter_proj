cmake_minimum_required(VERSION 2.8.3)
project(tgrasp)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  visualization_msgs
  trajectory_msgs
  tf
  tf_conversions
  sensor_msgs
  message_generation
  message_runtime
  nav_msgs
  roscpp
  rospy
  std_msgs
  tgrasp_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_generation sensor_msgs message_runtime nav_msgs roscpp rospy std_msgs tgrasp_msgs)


find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cluster_extractor src/cluster_extractor.cpp)

target_link_libraries(cluster_extractor
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
