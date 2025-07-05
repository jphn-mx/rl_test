cmake_minimum_required(VERSION 3.0.2)
project(pcl1)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(obstacle_detect src/obstacle_detect.cpp)
add_executable(read_pcd1 src/read_pcd1.cpp)

target_link_libraries(obstacle_detect 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(read_pcd1 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS obstacle_detect read_pcd1
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


