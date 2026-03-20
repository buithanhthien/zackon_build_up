cmake_minimum_required(VERSION 3.8)
project(lidar_dock_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
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
)
target_include_directories(lidar_dock_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lidar_dock_detector ${DOCK_DEPS})
pluginlib_export_plugin_description_file(opennav_docking_core plugins.xml)

add_executable(debug_intensity_node src/debug_intensity_node.cpp)
ament_target_dependencies(debug_intensity_node rclcpp sensor_msgs)

install(TARGETS lidar_dock_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS debug_intensity_node
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
