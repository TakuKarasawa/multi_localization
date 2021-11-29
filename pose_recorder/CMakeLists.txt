cmake_minimum_required(VERSION 3.0.2)
project(pose_recorder)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  object_detector_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
