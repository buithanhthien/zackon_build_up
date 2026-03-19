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
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── Standalone docking node ──────────────────────────────────────────────────
add_executable(docking_node
  src/docking_node.cpp
)
target_include_directories(docking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(docking_node
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ── Debug intensity node ─────────────────────────────────────────────────────
add_executable(debug_intensity_node
  src/debug_intensity_node.cpp
)
ament_target_dependencies(debug_intensity_node
  rclcpp
  sensor_msgs
)

install(TARGETS docking_node debug_intensity_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
