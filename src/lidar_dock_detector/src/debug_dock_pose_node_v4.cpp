#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class DebugDockPoseNode : public rclcpp::Node
{
public:
  DebugDockPoseNode()
  : Node("debug_dock_pose_node_v4")
  {
    scan_topic_          = "/scan_rear_lidar_filter";
    lrf_forward_offset_  = -0.35;
    tape_distance_       = 0.375;
    rubber_width_        = 0.32;
    reflector_width_     = 0.048;
    i_peak_              = 45.0f;
    i_valley_            = 24.0f;
    valley_search_range_ = 19;
    max_detect_range_    = 3.0;
    intensity_cluster_threshold_ = 45.0f;
    min_cluster_points_ = 8;
    max_cluster_range_span_ = 0.03;
    min_cluster_angle_width_ = 0.01;
    max_cluster_angle_width_ = 0.12;

    max_peak_diff_ = 4.0f;
    max_pose_jump_dist_ = 0.10; 
    max_pose_jump_yaw_rad_ = 15.0 * M_PI / 180.0;

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DebugDockPoseNode::scanCallback, this, std::placeholders::_1));

    dock_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/debug_dock_pose_lidar", rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(
      this->get_logger(),
      "[debug_dock_pose_node_v4] Listening to %s", scan_topic_.c_str());
  }

private:
  struct Reflector
  {
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

  struct DetectStats
  {
    int reject_peak = 0;
    int reject_local_max = 0;
    int reject_valley = 0;
    int reject_range = 0;
    int reject_beta = 0;
    int accepted = 0;

    float strongest_peak_left = 0.0f;
    float strongest_peak_right = 0.0f;
  };

  bool isMeasurementReliable(
    const DetectStats & stats,
    const Reflector & left,
    const Reflector & right) const
  {
    if (stats.strongest_peak_left <= 10.0f || stats.strongest_peak_right <= 10.0f) {
      RCLCPP_WARN(
        this->get_logger(),
        "[quality] Reject: one side peak is zero (left=%.1f right=%.1f)",
        stats.strongest_peak_left,
        stats.strongest_peak_right);
      return false;
    }

    if (std::abs(stats.strongest_peak_left - stats.strongest_peak_right) > max_peak_diff_) {
      RCLCPP_WARN(
        this->get_logger(),
        "[quality] Reject: peak difference too large (left=%.1f right=%.1f diff=%.1f)",
        stats.strongest_peak_left,
        stats.strongest_peak_right,
        std::abs(stats.strongest_peak_left - stats.strongest_peak_right));
      return false;
    }

    double dist = std::hypot(left.x - right.x, left.y - right.y);
    if (std::abs(dist - tape_distance_) > tape_distance_ * 0.2) {
      RCLCPP_WARN(
        this->get_logger(),
        "[quality] Reject: tape distance invalid (measured=%.3f expected=%.3f)",
        dist, tape_distance_);
      return false;
    }

    return true;
  }

  bool isPoseJumpReasonable(const geometry_msgs::msg::PoseStamped & new_pose) const
  {
    if (!has_last_valid_pose_) {
      return true;
    }

    double dx = new_pose.pose.position.x - last_valid_pose_.pose.position.x;
    double dy = new_pose.pose.position.y - last_valid_pose_.pose.position.y;
    double dist = std::hypot(dx, dy);

    double yaw_new = 2.0 * std::atan2(
      new_pose.pose.orientation.z,
      new_pose.pose.orientation.w);

    double yaw_old = 2.0 * std::atan2(
      last_valid_pose_.pose.orientation.z,
      last_valid_pose_.pose.orientation.w);

    double dyaw = yaw_new - yaw_old; // do lech cua pose hien tai va pose truoc do 
    while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

    if (dist > max_pose_jump_dist_) {
      RCLCPP_WARN(
        this->get_logger(),
        "[quality] Reject: pose jump too large in position (%.3f m)", dist);
      return false;
    }

    if (std::abs(dyaw) > max_pose_jump_yaw_rad_) {
      RCLCPP_WARN(
        this->get_logger(),
        "[quality] Reject: pose jump too large in yaw (%.3f rad)", dyaw);
      return false;
    }

    return true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    DetectStats stats;
    auto reflectors = detectReflectors(*msg, stats);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[debug_dock_pose_node_v4] reflectors.size() = %zu",
      reflectors.size());

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[DEBUG FILTER] accept=%d peak_rej=%d local_rej=%d valley_rej=%d range_rej=%d beta_rej=%d left_peak=%.1f right_peak=%.1f",
      stats.accepted,
      stats.reject_peak,
      stats.reject_local_max,
      stats.reject_valley,
      stats.reject_range,
      stats.reject_beta,
      stats.strongest_peak_left,
      stats.strongest_peak_right);

    for (size_t k = 0; k < reflectors.size(); ++k) {
      const auto & r = reflectors[k];
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[debug_dock_pose_node_v4] ref[%zu]: idx=%d x=%.3f y=%.3f Ipeak=%.1f Ivalley=%.1f beta=%.3f",
        k, r.peak_idx, r.x, r.y, r.I_peak, r.I_valley, r.beta);
    }

    geometry_msgs::msg::PoseStamped detected;
    detected.header = msg->header;

    bool found = false;

    if (reflectors.size() == 2u) {
      const Reflector & right =
        (reflectors[0].peak_idx < reflectors[1].peak_idx)
        ? reflectors[0] : reflectors[1];
      const Reflector & left =
        (reflectors[0].peak_idx < reflectors[1].peak_idx)
        ? reflectors[1] : reflectors[0];

      if (isMeasurementReliable(stats, left, right)) {
        geometry_msgs::msg::PoseStamped candidate;
        candidate.header = msg->header;

        if (computeDockPose(left, right, tape_distance_, lrf_forward_offset_, candidate)) {
          if (isPoseJumpReasonable(candidate)) {
            detected = candidate;
            found = true;

            last_valid_pose_ = candidate;
            has_last_valid_pose_ = true;
          } else {
            RCLCPP_WARN(
              this->get_logger(),
              "[dock] Keep last valid pose: candidate jump rejected");
          }
        }
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "[dock] Keep last valid pose: measurement unreliable");
      }
    }

    if (found) {
      double yaw = 2.0 * std::atan2(
        detected.pose.orientation.z,
        detected.pose.orientation.w);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[debug_dock_pose_node_v4] UPDATED DOCK POSE in frame [%s]: x=%.3f y=%.3f yaw=%.3f rad",
        detected.header.frame_id.c_str(),
        detected.pose.position.x,
        detected.pose.position.y,
        yaw);

      dock_pose_pub_->publish(detected);
    }
    else if (has_last_valid_pose_) {
      double yaw = 2.0 * std::atan2(
        last_valid_pose_.pose.orientation.z,
        last_valid_pose_.pose.orientation.w);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[debug_dock_pose_node_v4] HOLD LAST VALID DOCK POSE in frame [%s]: x=%.3f y=%.3f yaw=%.3f rad",
        last_valid_pose_.header.frame_id.c_str(),
        last_valid_pose_.pose.position.x,
        last_valid_pose_.pose.position.y,
        yaw);

      dock_pose_pub_->publish(last_valid_pose_);
    }
  }

  std::vector<Reflector> detectReflectors(
    const sensor_msgs::msg::LaserScan & scan,
    DetectStats & stats) const
  {
    std::vector<Reflector> result;

    const size_t N = scan.ranges.size();
    if (scan.intensities.size() != N) {
      return result;
    }

    int i = 0;

    while (i < static_cast<int>(N)) {
      while (i < static_cast<int>(N) &&
             scan.intensities[i] < intensity_cluster_threshold_) {
        stats.reject_peak++;
        i++;
      }

      if (i >= static_cast<int>(N)) {
        break;
      }

      int cluster_start = i;
      std::vector<int> indices;

      while (i < static_cast<int>(N) &&
             scan.intensities[i] >= intensity_cluster_threshold_) {
        indices.push_back(i);
        i++;
      }

      int cluster_end = i - 1;

      if (static_cast<int>(indices.size()) < min_cluster_points_) {
        stats.reject_local_max++;
        continue;
      }

      double sum_I = 0.0;
      double sum_idx_I = 0.0;
      double sum_range_I = 0.0;

      double min_range = 1e9;
      double max_range = -1e9;

      float strongest_I = 0.0f;

      for (int idx : indices) {
        float I = scan.intensities[idx];
        float r = scan.ranges[idx];

        if (!std::isfinite(r) ||
            r < scan.range_min ||
            r > scan.range_max) {
          stats.reject_range++;
          continue;
        }

        sum_I += I;
        sum_idx_I += static_cast<double>(idx) * I;
        sum_range_I += static_cast<double>(r) * I;

        min_range = std::min(min_range, static_cast<double>(r));
        max_range = std::max(max_range, static_cast<double>(r));

        strongest_I = std::max(strongest_I, I);
      }

      if (sum_I < 1e-6) {
        continue;
      }

      double range_span = max_range - min_range;

      if (range_span > max_cluster_range_span_) {
        stats.reject_range++;
        continue;
      }

      double angle_start =
        scan.angle_min + cluster_start * scan.angle_increment;

      double angle_end =
        scan.angle_min + cluster_end * scan.angle_increment;

      double cluster_angle_width =
        std::abs(angle_end - angle_start);

      if (cluster_angle_width < min_cluster_angle_width_ ||
          cluster_angle_width > max_cluster_angle_width_) {
        stats.reject_valley++;
        continue;
      }

      double center_idx = sum_idx_I / sum_I;
      double center_range = sum_range_I / sum_I;

      double rplidar_angle =
        scan.angle_min + center_idx * scan.angle_increment;

      double ros_angle = M_PI - rplidar_angle;

      while (ros_angle > M_PI) {
        ros_angle -= 2.0 * M_PI;
      }

      while (ros_angle < -M_PI) {
        ros_angle += 2.0 * M_PI;
      }

      double x = center_range * std::cos(ros_angle);
      double y = center_range * std::sin(ros_angle);

      if (y >= 0.0) {
        stats.strongest_peak_left =
          std::max(stats.strongest_peak_left, strongest_I);
      } else {
        stats.strongest_peak_right =
          std::max(stats.strongest_peak_right, strongest_I);
      }

      Reflector ref;
      ref.peak_idx = static_cast<int>(std::round(center_idx));
      ref.valley_l_idx = cluster_start;
      ref.valley_r_idx = cluster_end;
      ref.I_peak = strongest_I;
      ref.I_valley = 0.0f;
      ref.L_peak = center_range;
      ref.beta = 0.0;
      ref.x = x;
      ref.y = y;

      result.push_back(ref);
      stats.accepted++;
    }

    return result;
  }

  bool computeDockPose(
    const Reflector & left_tape,
    const Reflector & right_tape,
    double tape_dist,
    double dock_offset,
    geometry_msgs::msg::PoseStamped & pose_out) const
  {
    double dx = left_tape.x - right_tape.x;
    double dy = left_tape.y - right_tape.y;
    double measured_dist = std::hypot(dx, dy);

    if (std::abs(measured_dist - tape_dist) > tape_dist * 0.2) {
      RCLCPP_WARN(
        this->get_logger(),
        "[debug_dock_pose_node_v4] Pair rejected: measured_dist=%.3f expected=%.3f",
        measured_dist, tape_dist);
      return false;
    }

    double mx = 0.5 * (left_tape.x + right_tape.x);
    double my = 0.5 * (left_tape.y + right_tape.y);

    double tx = dx / measured_dist;
    double ty = dy / measured_dist;

    double n1x = -ty;
    double n1y =  tx;

    double n2x =  ty;
    double n2y = -tx;

    double vx = -mx;
    double vy = -my;

    double dot1 = n1x * vx + n1y * vy;
    double dot2 = n2x * vx + n2y * vy;

    double nx, ny;
    if (dot1 >= dot2) {
      nx = n1x;
      ny = n1y;
    } else {
      nx = n2x;
      ny = n2y;
    }
    double yaw = std::atan2(ny, nx);

    yaw += M_PI;

    while (yaw > M_PI) {
      yaw -= 2.0 * M_PI;
    }
    while (yaw < -M_PI) {
      yaw += 2.0 * M_PI;
    }

    double d = std::abs(dock_offset);

    double px = mx - d * nx;
    double py = my - d * ny;

    pose_out.pose.position.x = -px + 0.51;
    pose_out.pose.position.y = py;
    pose_out.pose.position.z = 0.0;

    pose_out.pose.orientation.x = 0.0;
    pose_out.pose.orientation.y = 0.0;
    pose_out.pose.orientation.z = std::sin(yaw / 2.0);
    pose_out.pose.orientation.w = std::cos(yaw / 2.0);

    return true;
  }

private:
  std::string scan_topic_;
  double lrf_forward_offset_;
  double tape_distance_;
  double rubber_width_;
  double reflector_width_;
  float i_peak_;
  float i_valley_;
  int valley_search_range_;
  double max_detect_range_;
  float intensity_cluster_threshold_;
  int min_cluster_points_;
  double max_cluster_range_span_;
  double min_cluster_angle_width_;
  double max_cluster_angle_width_;

  bool has_last_valid_pose_ = false;
  geometry_msgs::msg::PoseStamped last_valid_pose_;

  float max_peak_diff_;
  double max_pose_jump_dist_;
  double max_pose_jump_yaw_rad_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DebugDockPoseNode>());
  rclcpp::shutdown();
  return 0;
}
