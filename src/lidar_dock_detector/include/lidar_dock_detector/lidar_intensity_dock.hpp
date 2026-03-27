#ifndef LIDAR_DOCK_DETECTOR__LIDAR_INTENSITY_DOCK_HPP_
#define LIDAR_DOCK_DETECTOR__LIDAR_INTENSITY_DOCK_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "opennav_docking_core/charging_dock.hpp"

namespace lidar_dock_detector
{

class LidarIntensityDock : public opennav_docking_core::ChargingDock
{
public:
  LidarIntensityDock() = default;
  ~LidarIntensityDock() override = default;

  // ── Lifecycle ──────────────────────────────
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf) override;

  void cleanup()    override;
  void activate()   override;
  void deactivate() override;

  // ── ChargingDock interface ─────────────────
  geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose & dock_pose,
    const std::string & frame) override;

  bool getRefinedPose(
    geometry_msgs::msg::PoseStamped & pose,
    std::string /*id*/) override;

  bool isDocked()           override;
  bool isCharging()         override;
  bool disableCharging()    override;
  bool hasStoppedCharging() override;

private:
  // ── Internal types ─────────────────────────
  struct Reflector {
    int    peak_idx;
    int    valley_l_idx;
    int    valley_r_idx;
    float  I_peak;
    float  I_valley;
    double L_peak;
    double beta;
    double x;
    double y;
  };

  // ── Detection helpers ──────────────────────
  void scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg);

  std::vector<Reflector> detectReflectors(
    const sensor_msgs::msg::LaserScan & scan) const;

  double computeInceptionAngle(
    double Li, double Lj, double theta_rad) const;

  bool computeDockPose(
    const Reflector & left_tape,
    const Reflector & right_tape,
    double tape_dist,
    double lrf_offset,
    geometry_msgs::msg::PoseStamped & pose_out) const;

  // ── Parameters ─────────────────────────────
  std::string name_;
  std::string scan_topic_;
  std::string base_frame_;

  double lrf_tilt_alpha_;
  double lrf_forward_offset_;

  double tape_distance_;
  double rubber_width_;
  double reflector_width_;

  float  i_peak_;
  float  i_valley_;
  int    valley_search_range_;

  double staging_x_offset_;
  double docking_threshold_;

  // ── ROS handles ────────────────────────────
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // ── State ──────────────────────────────────
  geometry_msgs::msg::PoseStamped last_detected_pose_;
  bool       dock_detected_{false};
  std::mutex pose_mutex_;
};

}  // namespace lidar_dock_detector

#endif  // LIDAR_DOCK_DETECTOR__LIDAR_INTENSITY_DOCK_HPP_