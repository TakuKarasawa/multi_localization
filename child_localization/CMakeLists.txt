cmake_minimum_required(VERSION 3.0.2)
project(child_localization)

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
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

# child ekf
add_executable(child_ekf_node
    src/child_ekf.cpp
    src/child_ekf_node.cpp
)
target_link_libraries(child_ekf_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen3_LIBRARIES})

# child mcl
add_executable(child_mcl_node
    src/child_mcl.cpp
    src/child_mcl_node.cpp
)
target_link_libraries(child_mcl_node ${catkin_LIBRARIES})

# tf
add_executable(tf_broadcaster_node
    src/tf_broadcaster.cpp
    src/tf_broadcaster_node.cpp
)
target_link_libraries(tf_broadcaster_node ${catkin_LIBRARIES})

# mcl
add_executable(mcl_node
    src/mcl.cpp
    src/mcl_node.cpp
)
target_link_libraries(mcl_node ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})