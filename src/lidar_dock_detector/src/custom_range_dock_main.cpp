#include "rclcpp/rclcpp.hpp"
#include "lidar_dock_detector/custom_range_dock.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_dock_detector::CustomRangeDock>();
  
  // Start docking once after 2 seconds
  rclcpp::TimerBase::SharedPtr timer;
  timer = node->create_wall_timer(
    std::chrono::seconds(2),
    [node, &timer]() {
      node->startDocking();
      if (timer) timer->cancel();
    });
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
