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
  : Node("debug_dock_pose_node_v3")
  {
    // ===== Current parameters =====
    scan_topic_ = "/scan_rear_lidar_filter";
    tape_distance_ = 0.375;
    max_detect_range_ = 3.0;

    intensity_cluster_threshold_ = 45.0f;
    min_cluster_points_ = 8;
    max_cluster_range_span_ = 0.03;   // 3 cm
    min_cluster_angle_width_ = 0.01;  // rad
    max_cluster_angle_width_ = 0.12;  // rad

    use_low_pass_filter_ = true;
    alpha_pos_ = 0.2;
    alpha_yaw_ = 0.2;
    // ==============================

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DebugDockPoseNode::scanCallback, this, std::placeholders::_1));

    dock_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/debug_dock_pose_lidar", rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(
      this->get_logger(),
      "[debug_dock_pose_node_v3] Listening to %s", scan_topic_.c_str());
  }

private:
  struct Reflector
  {
    int peak_idx;
    int cluster_start_idx;
    int cluster_end_idx;
    float I_peak;
    float I_valley;
    double L_peak;
    double beta;
    double x;
    double y;
  };

  struct DetectStats
  {
    int reject_peak = 0;
    int reject_cluster_size = 0;
    int reject_angle_width = 0;
    int reject_range = 0;
    int accepted = 0;

    float strongest_peak_left = 0.0f;
    float strongest_peak_right = 0.0f;
  };

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    DetectStats stats;
    auto reflectors = detectReflectors(*msg, stats);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[debug_dock_pose_node_v3] reflectors.size() = %zu",
      reflectors.size());

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "[DEBUG FILTER] accept=%d peak_rej=%d cluster_rej=%d angle_rej=%d range_rej=%d left_peak=%.1f right_peak=%.1f",
      stats.accepted,
      stats.reject_peak,
      stats.reject_cluster_size,
      stats.reject_angle_width,
      stats.reject_range,
      stats.strongest_peak_left,
      stats.strongest_peak_right);

    for (size_t k = 0; k < reflectors.size(); ++k) {
      const auto & r = reflectors[k];
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[reflector %zu] idx=%d start=%d end=%d x=%.3f y=%.3f Ipeak=%.1f range=%.3f",
        k, r.peak_idx, r.cluster_start_idx, r.cluster_end_idx,
        r.x, r.y, r.I_peak, r.L_peak);
    }

    geometry_msgs::msg::PoseStamped detected;
    detected.header = msg->header;

    bool found = false;

    if (reflectors.size() == 2u) {
      // Use y to split left/right in lidar frame, not peak_idx
      const Reflector & left =
        (reflectors[0].y >= reflectors[1].y) ? reflectors[0] : reflectors[1];
      const Reflector & right =
        (reflectors[0].y >= reflectors[1].y) ? reflectors[1] : reflectors[0];

      found = computeDockPose(left, right, tape_distance_, detected);
    }

    if (found) {
      double raw_yaw = 2.0 * std::atan2(
        detected.pose.orientation.z,
        detected.pose.orientation.w);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[RAW DOCK POSE] frame=%s x=%.3f y=%.3f yaw=%.3f",
        detected.header.frame_id.c_str(),
        detected.pose.position.x,
        detected.pose.position.y,
        raw_yaw);

      if (use_low_pass_filter_) {
        applyLowPassFilter(detected);
      }

      double filt_yaw = 2.0 * std::atan2(
        detected.pose.orientation.z,
        detected.pose.orientation.w);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[FILTERED DOCK POSE] frame=%s x=%.3f y=%.3f yaw=%.3f",
        detected.header.frame_id.c_str(),
        detected.pose.position.x,
        detected.pose.position.y,
        filt_yaw);

      dock_pose_pub_->publish(detected);
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
      // Skip low-intensity beams
      while (i < static_cast<int>(N) &&
             scan.intensities[i] < intensity_cluster_threshold_) {
        stats.reject_peak++;
        i++;
      }

      if (i >= static_cast<int>(N)) {
        break;
      }

      // Start cluster
      int cluster_start = i;
      std::vector<int> indices;

      while (i < static_cast<int>(N) &&
             scan.intensities[i] >= intensity_cluster_threshold_) {
        indices.push_back(i);
        i++;
      }

      int cluster_end = i - 1;

      // Cluster size check
      if (static_cast<int>(indices.size()) < min_cluster_points_) {
        stats.reject_cluster_size++;
        continue;
      }

      // Range and intensity properties
      double min_range = 1e9;
      double max_range = -1e9;
      double sum_range = 0.0;
      int valid_range_count = 0;
      float strongest_I = 0.0f;

      for (int idx : indices) {
        const float I = scan.intensities[idx];
        const float r = scan.ranges[idx];

        strongest_I = std::max(strongest_I, I);

        if (!std::isfinite(r) ||
            r < scan.range_min ||
            r > scan.range_max) {
          continue;
        }

        if (static_cast<double>(r) > max_detect_range_) {
          continue;
        }

        min_range = std::min(min_range, static_cast<double>(r));
        max_range = std::max(max_range, static_cast<double>(r));
        sum_range += static_cast<double>(r);
        valid_range_count++;
      }

      if (valid_range_count == 0) {
        stats.reject_range++;
        continue;
      }

      const double range_span = max_range - min_range;
      if (range_span > max_cluster_range_span_) {
        stats.reject_range++;
        continue;
      }

      const double angle_start =
        scan.angle_min + cluster_start * scan.angle_increment;
      const double angle_end =
        scan.angle_min + cluster_end * scan.angle_increment;
      const double cluster_angle_width =
        std::abs(angle_end - angle_start);

      if (cluster_angle_width < min_cluster_angle_width_ ||
          cluster_angle_width > max_cluster_angle_width_) {
        stats.reject_angle_width++;
        continue;
      }

      // ===== Geometric center, not weighted intensity center =====
      const double center_idx = 0.5 * (cluster_start + cluster_end);
      const double center_range = sum_range / static_cast<double>(valid_range_count);

      // Convert to x,y in lidar frame
      double rplidar_angle =
        scan.angle_min + center_idx * scan.angle_increment;

      double ros_angle = M_PI - rplidar_angle;
      while (ros_angle > M_PI) {
        ros_angle -= 2.0 * M_PI;
      }
      while (ros_angle < -M_PI) {
        ros_angle += 2.0 * M_PI;
      }

      const double x = center_range * std::cos(ros_angle);
      const double y = center_range * std::sin(ros_angle);

      if (y >= 0.0) {
        stats.strongest_peak_left =
          std::max(stats.strongest_peak_left, strongest_I);
      } else {
        stats.strongest_peak_right =
          std::max(stats.strongest_peak_right, strongest_I);
      }

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "[cluster] start=%d end=%d size=%zu width=%.4f range_span=%.4f center_idx=%.2f center_range=%.3f x=%.3f y=%.3f Imax=%.1f",
        cluster_start,
        cluster_end,
        indices.size(),
        cluster_angle_width,
        range_span,
        center_idx,
        center_range,
        x,
        y,
        strongest_I);

      Reflector ref;
      ref.peak_idx = static_cast<int>(std::round(center_idx));
      ref.cluster_start_idx = cluster_start;
      ref.cluster_end_idx = cluster_end;
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
    geometry_msgs::msg::PoseStamped & pose_out) const
  {
    const double dx = left_tape.x - right_tape.x;
    const double dy = left_tape.y - right_tape.y;
    const double measured_dist = std::hypot(dx, dy);

    if (std::abs(measured_dist - tape_dist) > tape_dist * 0.2) {
      RCLCPP_WARN(
        this->get_logger(),
        "[debug_dock_pose_node_v3] Pair rejected: measured_dist=%.3f expected=%.3f",
        measured_dist, tape_dist);
      return false;
    }

    // Midpoint-only debug mode
    const double mx = 0.5 * (left_tape.x + right_tape.x);
    const double my = 0.5 * (left_tape.y + right_tape.y);

    // Tangent along tape line
    const double tx = dx / measured_dist;
    const double ty = dy / measured_dist;

    // Two normals
    const double n1x = -ty;
    const double n1y =  tx;
    const double n2x =  ty;
    const double n2y = -tx;

    // Choose normal pointing toward lidar origin
    const double vx = -mx;
    const double vy = -my;

    const double dot1 = n1x * vx + n1y * vy;
    const double dot2 = n2x * vx + n2y * vy;

    double nx, ny;
    if (dot1 >= dot2) {
      nx = n1x;
      ny = n1y;
    } else {
      nx = n2x;
      ny = n2y;
    }

    double yaw = std::atan2(ny, nx);
    while (yaw > M_PI) {
      yaw -= 2.0 * M_PI;
    }
    while (yaw < -M_PI) {
      yaw += 2.0 * M_PI;
    }

    // IMPORTANT: midpoint only, no offset, no x inversion
    pose_out.pose.position.x = mx;
    pose_out.pose.position.y = my;
    pose_out.pose.position.z = 0.0;

    pose_out.pose.orientation.x = 0.0;
    pose_out.pose.orientation.y = 0.0;
    pose_out.pose.orientation.z = std::sin(yaw / 2.0);
    pose_out.pose.orientation.w = std::cos(yaw / 2.0);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "[dock raw geom] LEFT=(%.3f, %.3f) RIGHT=(%.3f, %.3f) MID=(%.3f, %.3f) yaw=%.3f",
      left_tape.x, left_tape.y,
      right_tape.x, right_tape.y,
      mx, my, yaw);

    return true;
  }

  void applyLowPassFilter(geometry_msgs::msg::PoseStamped & pose)
  {
    double yaw = 2.0 * std::atan2(
      pose.pose.orientation.z,
      pose.pose.orientation.w);

    if (!has_filtered_pose_) {
      filt_x_ = pose.pose.position.x;
      filt_y_ = pose.pose.position.y;
      filt_yaw_ = yaw;
      has_filtered_pose_ = true;
    } else {
      filt_x_ = alpha_pos_ * pose.pose.position.x + (1.0 - alpha_pos_) * filt_x_;
      filt_y_ = alpha_pos_ * pose.pose.position.y + (1.0 - alpha_pos_) * filt_y_;

      double dyaw = yaw - filt_yaw_;
      while (dyaw > M_PI) {
        dyaw -= 2.0 * M_PI;
      }
      while (dyaw < -M_PI) {
        dyaw += 2.0 * M_PI;
      }

      filt_yaw_ += alpha_yaw_ * dyaw;

      while (filt_yaw_ > M_PI) {
        filt_yaw_ -= 2.0 * M_PI;
      }
      while (filt_yaw_ < -M_PI) {
        filt_yaw_ += 2.0 * M_PI;
      }
    }

    pose.pose.position.x = filt_x_;
    pose.pose.position.y = filt_y_;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(filt_yaw_ / 2.0);
    pose.pose.orientation.w = std::cos(filt_yaw_ / 2.0);
  }

private:
  std::string scan_topic_;

  double tape_distance_;
  double max_detect_range_;

  float intensity_cluster_threshold_;
  int min_cluster_points_;
  double max_cluster_range_span_;
  double min_cluster_angle_width_;
  double max_cluster_angle_width_;

  bool use_low_pass_filter_;
  double alpha_pos_;
  double alpha_yaw_;

  bool has_filtered_pose_ = false;
  double filt_x_ = 0.0;
  double filt_y_ = 0.0;
  double filt_yaw_ = 0.0;

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