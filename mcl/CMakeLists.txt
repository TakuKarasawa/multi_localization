cmake_minimum_required(VERSION 3.0.2)
project(mcl)

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

find_package(PCL REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    ${PCL_INCLUDE_DIRS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/mcl.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


add_executable(mcl_node
    src/mcl_node.cpp
    src/mcl.cpp
)

add_executable(tf_broadcaster_node
    src/tf_broadcaster_node.cpp
    src/tf_broadcaster.cpp
)

add_dependencies(mcl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tf_broadcaster_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(mcl_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(tf_broadcaster_node ${catkin_LIBRARIES})