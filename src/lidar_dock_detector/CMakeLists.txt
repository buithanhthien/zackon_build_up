cmake_minimum_required(VERSION 3.8)
project(lidar_dock_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(opennav_docking_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)

set(DOCK_DEPS
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  opennav_docking_core
  pluginlib
  angles
)

add_library(lidar_dock_detector SHARED
  src/lidar_intensity_dock.cpp
  src/blind_docking.cpp
)
target_include_directories(lidar_dock_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lidar_dock_detector ${DOCK_DEPS})
pluginlib_export_plugin_description_file(opennav_docking_core plugins.xml)

install(TARGETS lidar_dock_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(debug_dock_pose_node src/debug_dock_pose_node.cpp)
ament_target_dependencies(debug_dock_pose_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

add_executable(debug_dock_pose_node_v2 src/debug_dock_pose_node_v2.cpp)
ament_target_dependencies(debug_dock_pose_node_v2
  rclcpp
  sensor_msgs
  geometry_msgs
)

add_executable(debug_dock_pose_node_v3 src/debug_dock_pose_node_v3.cpp)
ament_target_dependencies(debug_dock_pose_node_v3
  rclcpp
  sensor_msgs
  geometry_msgs
)

add_executable(debug_dock_pose_node_v4 src/debug_dock_pose_node_v4.cpp)
ament_target_dependencies(debug_dock_pose_node_v4
  rclcpp
  sensor_msgs
  geometry_msgs
)

add_executable(rear_docking_controller_node src/rear_docking_controller_node.cpp)
ament_target_dependencies(rear_docking_controller_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

add_executable(custom_range_dock_node src/custom_range_dock.cpp src/custom_range_dock_main.cpp)
target_include_directories(custom_range_dock_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(custom_range_dock_node
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  nav2_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  debug_dock_pose_node
  debug_dock_pose_node_v2
  debug_dock_pose_node_v3
  debug_dock_pose_node_v4
  rear_docking_controller_node
  custom_range_dock_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
