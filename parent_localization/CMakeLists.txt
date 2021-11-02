cmake_minimum_required(VERSION 3.0.2)
project(parent_localization)

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
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

include_directories(
    include
    ../mcl/include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ../mcl/src/mcl.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# parent mcl
add_executable(parent_mcl_node
    ../mcl/src/mcl.cpp
    src/parent_mcl_node.cpp
    src/parent_mcl.cpp
)
target_link_libraries(parent_mcl_node ${catkin_LIBRARIES} ${PROJECT_NAME})

# reposition_marker
add_executable(reposition_marker
    src/reposition_marker.cpp
)
target_link_libraries(reposition_marker ${catkin_LIBRARIES})

# set_marker
add_executable(set_marker src/set_marker.cpp)
target_link_libraries(set_marker ${catkin_LIBRARIES})