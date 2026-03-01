cmake_minimum_required(VERSION 3.8)
project(pointcloud_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_executable(octo_explorer_node src/explorer_node.cpp)
ament_target_dependencies(octo_explorer_node
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  tf2)
target_link_libraries(octo_explorer_node ${PCL_LIBRARIES})
target_include_directories(octo_explorer_node PUBLIC ${PCL_INCLUDE_DIRS})

add_executable(prob_grid_node src/prob_grid_node.cpp)
ament_target_dependencies(prob_grid_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  tf2)
target_link_libraries(prob_grid_node ${PCL_LIBRARIES})
target_include_directories(prob_grid_node PUBLIC ${PCL_INCLUDE_DIRS})

add_executable(pointcloud_fusion_node src/pointcloud_fusion_node.cpp)
ament_target_dependencies(pointcloud_fusion_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  tf2)
target_link_libraries(pointcloud_fusion_node ${PCL_LIBRARIES})
target_include_directories(pointcloud_fusion_node PUBLIC ${PCL_INCLUDE_DIRS})

add_executable(map_saver_node src/map_saver_node.cpp)
ament_target_dependencies(map_saver_node
  rclcpp
  std_srvs)

install(TARGETS
  octo_explorer_node
  prob_grid_node
  pointcloud_fusion_node
  map_saver_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
