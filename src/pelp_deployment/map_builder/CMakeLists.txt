cmake_minimum_required(VERSION 3.8)
project(map_builder)

cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_ros REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

add_executable(map_builder_node
  src/map_builder.cpp
  src/map_builder_node.cpp
)

target_link_libraries(map_builder_node
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

ament_target_dependencies(map_builder_node
  sensor_msgs
  rclcpp
  nav_msgs
  pcl_conversions
  octomap
  octomap_ros
  octomap_msgs
)

install(TARGETS
  map_builder_node
  DESTINATION lib/${PROJECT_NAME}
)