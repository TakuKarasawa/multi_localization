cmake_minimum_required(VERSION 3.0.2)
project(map_updater)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  object_detector_msgs
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    ${PCL_INCLUDE_DIR}
    ${Eigen3_INCLUDE_DIRS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIR}
    ${Eigen3_INCLUDE_DIRS}
)

# map update checker
add_executable(map_update_checker src/map_update_checker.cpp)
target_link_libraries(map_update_checker ${catkin_LIBRARIES})
