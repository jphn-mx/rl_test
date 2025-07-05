cmake_minimum_required(VERSION 3.0.2)
project(ch1)

# 添加C++11支持
#add_compile_options(-std=c++11)

# 查找catkin的必需组件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  pcl_ros
  pcl_conversions
  geometry_msgs
)

# 添加消息文件
add_message_files(
  FILES
  person.msg

)

# 添加服务文件
add_service_files(
  FILES
  st.srv
)

# 生成消息和服务
generate_messages(
  DEPENDENCIES
  std_msgs
)

# catkin包配置
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime sensor_msgs pcl_ros pcl_conversions geometry_msgs
)

# 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 链接目录
link_directories(${PCL_LIBRARY_DIRS})

# 添加PCL的定义
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
add_executable(subc src/subc.cpp)
add_executable(pubc src/pubc.cpp)
add_executable(read_pcd src/read_pcd.cpp)
add_executable(read_plane src/read_plane.cpp)
add_executable(2d_plane_model src/2d_plane_model.cpp)
add_executable(tri_model src/tri_model.cpp)
add_executable(recognize src/recognize.cpp)
add_executable(pointcloud2 src/pointcloud2.cpp)
add_executable( pcd2pgm src/pcd2pgm.cpp)

#add_executable(reg src/reg.cpp)

# 链接库
target_link_libraries(subc ${catkin_LIBRARIES})
target_link_libraries(pubc ${catkin_LIBRARIES})
target_link_libraries(pointcloud2 ${catkin_LIBRARIES})
target_link_libraries(read_pcd ${catkin_LIBRARIES})
target_link_libraries(read_plane ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(2d_plane_model ${catkin_LIBRARIES})
target_link_libraries(tri_model ${catkin_LIBRARIES})
target_link_libraries(recognize ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcd2pgm ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#target_link_libraries(reg ${catkin_LIBRARIES} ${PCL_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/pubpy.py 
  scripts/subpy.py
  scripts/msg_pub.py
  scripts/msg_sub.py
  scripts/srv_srv.py
  scripts/srv_clt.py
  scripts/static_pub.py
  scripts/static_sub.py 
  scripts/dynamic_pub.py
  scripts/dynamic_sub.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


