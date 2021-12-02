cmake_minimum_required(VERSION 3.0.2)
project(pose_recorder)

add_compile_options(-std=c++11 -O2 -g -Wall)

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

# pose_recorder
add_executable(pose_recorder_node src/pose_recorder.cpp)
target_link_libraries(pose_recorder_node ${catkin_LIBRARIES})