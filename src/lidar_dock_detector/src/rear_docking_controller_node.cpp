#include <cmath>
#include <memory>
#include <string>
#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

class RearDockingControllerNode : public rclcpp::Node
{
public:
  RearDockingControllerNode()
  : Node("rear_docking_controller_node")
  {
    dock_pose_topic_ = declare_parameter<std::string>("dock_pose_topic", "/debug_dock_pose_lidar");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan_rear_lidar_filter");
    // cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_test");


    control_frequency_ = declare_parameter<double>("control_frequency", 30.0);

    // Gains
    k_x_ = declare_parameter<double>("k_x", 0.5);
    k_y_ = declare_parameter<double>("k_y", 1.5);
    k_yaw_ = declare_parameter<double>("k_yaw", 2.0);

    // Velocity limits
    v_reverse_max_ = declare_parameter<double>("v_reverse_max", 0.2);
    v_reverse_min_ = declare_parameter<double>("v_reverse_min", 0.1);
    // w_max_ = declare_parameter<double>("w_max", 0.6);
    w_max_ = declare_parameter<double>("w_max", 0.3);

    // Thresholds
    stop_distance_ = declare_parameter<double>("stop_distance", 0.2);
    slowdown_distance_ = declare_parameter<double>("slowdown_distance", 0.40);
    y_align_threshold_ = declare_parameter<double>("y_align_threshold", 0.10);
    yaw_align_threshold_ = declare_parameter<double>("yaw_align_threshold", 0.25);

    // Pose freshness
    dock_pose_timeout_ = declare_parameter<double>("dock_pose_timeout", 0.30);
    stale_pose_timeout_ = declare_parameter<double>("stale_pose_timeout", 1.0);
    stop_on_lost_pose_ = declare_parameter<bool>("stop_on_lost_pose", true);

    // Safety
    min_range_threshold_ = declare_parameter<double>("min_range_threshold", 0.20);

    // Optional odom freshness
    require_fresh_odom_ = declare_parameter<bool>("require_fresh_odom", false);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.50);

    dock_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      dock_pose_topic_, 10,
      std::bind(&RearDockingControllerNode::dockPoseCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      std::bind(&RearDockingControllerNode::odomCallback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&RearDockingControllerNode::scanCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_frequency_),
      std::bind(&RearDockingControllerNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "RearDockingControllerNode started");
    RCLCPP_INFO(get_logger(), "dock_pose_topic: %s", dock_pose_topic_.c_str());
    RCLCPP_INFO(get_logger(), "odom_topic: %s", odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "cmd_vel_topic: %s", cmd_vel_topic_.c_str());
    RCLCPP_INFO(get_logger(), "scan_topic: %s", scan_topic_.c_str());
  }

private:
  static double clamp(double v, double lo, double hi)
  { 
    // (-)
    return std::max(lo, std::min(v, hi));
  }

  static double normalizeAngle(double a)
  {
    while (a > M_PI) {
      a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
      a += 2.0 * M_PI;
    }
    return a;
  }

  void dockPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Check if pose is stale (unchanged)
    if (has_dock_pose_) {
      const double dx = msg->pose.position.x - last_dock_pose_.pose.position.x;
      const double dy = msg->pose.position.y - last_dock_pose_.pose.position.y;
      const double dz = msg->pose.orientation.z - last_dock_pose_.pose.orientation.z;
      const double dw = msg->pose.orientation.w - last_dock_pose_.pose.orientation.w;
      
      const double pose_diff = std::sqrt(dx*dx + dy*dy + dz*dz + dw*dw);
      
      if (pose_diff < 0.001) {  // pose unchanged
        const double unchanged_time = (now() - last_dock_pose_change_time_).seconds();
        if (unchanged_time > stale_pose_timeout_) {
          if (state_ != DockState::SEARCH) {
            state_ = DockState::SEARCH;
            search_start_time_ = now();
            RCLCPP_WARN(
              get_logger(),
              "[rear_docking_ctrl] Stale pose detected (unchanged for %.2fs) -> SEARCH MODE",
              unchanged_time);
          }
          return;
        }
      } else {
        last_dock_pose_change_time_ = now();
      }
    } else {
      last_dock_pose_change_time_ = now();
    }

    last_dock_pose_ = *msg;
    has_dock_pose_ = true;
    last_dock_pose_time_ = now();

    // if pose is reacquired while searching
    if (state_ == DockState::SEARCH) {
      state_ = DockState::TRACK;

      RCLCPP_INFO(
        get_logger(),
        "[rear_docking_ctrl] Dock pose reacquired -> TRACK MODE");
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
    has_odom_ = true;
    last_odom_time_ = now();
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_ = msg;
    has_scan_ = true;
  }

  double getMinRange() const
  {
    if (!has_scan_ || last_scan_->ranges.empty()) {
      return std::numeric_limits<double>::infinity();
    }

    double min_range = std::numeric_limits<double>::infinity();
    for (const auto& range : last_scan_->ranges) {
      if (std::isfinite(range) && range > 0.0) {
        min_range = std::min(min_range, static_cast<double>(range));
      }
    }
    return min_range;
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  void publishSearchMotion()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;

    const double t = (now() - search_start_time_).seconds();

    // alternate left-right every 1 second
    const int phase =
      static_cast<int>(t / search_switch_period_) % 2;

    if (phase == 0) {
      cmd.angular.z = search_w_;
    } else {
      cmd.angular.z = -search_w_;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 500,
      "[rear_docking_ctrl] SEARCH MODE wz=%.3f",
      cmd.angular.z);

    cmd_pub_->publish(cmd);
  }

  void controlLoop()
  {
    // ===============================
    // SAFETY CHECK: Min range
    // ===============================
    const double min_range = getMinRange();
    if (min_range < min_range_threshold_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[rear_docking_ctrl] SAFETY STOP: min_range=%.3fm < threshold=%.3fm",
        min_range, min_range_threshold_);
      publishStop();
      return;
    }

    // ===============================
    // SEARCH / TRACK state handling
    // ===============================
    if (!has_dock_pose_) {
      if (state_ != DockState::SEARCH) {
        state_ = DockState::SEARCH;
        search_start_time_ = now();
      }

      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[rear_docking_ctrl] No dock pose -> SEARCH MODE");

      publishSearchMotion();
      return;
    }

    const double pose_age = (now() - last_dock_pose_time_).seconds();
    if (pose_age > dock_pose_timeout_) {
      if (state_ != DockState::SEARCH) {
        state_ = DockState::SEARCH;
        search_start_time_ = now();

        RCLCPP_WARN(
          get_logger(),
          "[rear_docking_ctrl] Dock pose timeout -> SEARCH MODE");
      }

      publishSearchMotion();
      return;
    }

    // valid pose
    state_ = DockState::TRACK;

    if (require_fresh_odom_) {
      if (!has_odom_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[rear_docking_ctrl] No odom yet");
        publishStop();
        return;
      }

      const double odom_age = (now() - last_odom_time_).seconds();
      if (odom_age > odom_timeout_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[rear_docking_ctrl] Odom timeout: %.3f s", odom_age);
        publishStop();
        return;
      }
    }

    // +X points toward robot base.
    // Dock behind robot => x < 0 when measurement is correct.
    const double x = last_dock_pose_.pose.position.x;
    const double y = last_dock_pose_.pose.position.y;

    const double yaw = normalizeAngle(
      2.0 * std::atan2(
        last_dock_pose_.pose.orientation.z,
        last_dock_pose_.pose.orientation.w));

    // If dock suddenly appears in front of the rear lidar frame, stop.
    if (x >= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[rear_docking_ctrl] Invalid rear-docking pose: x=%.3f (expected negative)", x);
      publishStop();
      return;
    }

    // Stop condition: longitudinal error is small enough
    if (std::abs(x) <= stop_distance_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[rear_docking_ctrl] Docked: x=%.3f y=%.3f yaw=%.3f",
        x, y, yaw);
      publishStop();
      return;
    }

    // Reverse speed from longitudinal error
    double v_mag;
    if (std::abs(x) > slowdown_distance_) {
      v_mag = v_reverse_max_;
    } else {
      double ratio = clamp(std::abs(x) / slowdown_distance_, 0.0, 1.0);
      v_mag = v_reverse_min_ + (v_reverse_max_ - v_reverse_min_) * ratio;
    }

    // Angular correction
    double w_cmd = -k_y_ * y + -k_yaw_ * yaw;
    w_cmd = clamp(w_cmd, -w_max_, w_max_);

    // If lateral or yaw error is large, slow down reverse speed
    const bool badly_misaligned =
      (std::abs(y) > y_align_threshold_) ||
      (std::abs(yaw) > yaw_align_threshold_);

    if (badly_misaligned) {
      v_mag *= 0.4;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = -v_mag;   // reverse into dock
    cmd.angular.z = w_cmd;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 200,
      "[rear_docking_ctrl] pose=(x=%.3f y=%.3f yaw=%.3f) cmd=(vx=%.3f wz=%.3f)",
      x, y, yaw, cmd.linear.x, cmd.angular.z);

    cmd_pub_->publish(cmd);
  }

private:
  enum class DockState
  {
    TRACK,
    SEARCH
  };

  DockState state_ = DockState::SEARCH;

  // search behavior
  // double search_w_ = 0.15;             // rad/s
  double search_w_ = 0.3;             // rad/s
  double search_switch_period_ = 1.0;  // seconds
  rclcpp::Time search_start_time_{0, 0, RCL_ROS_TIME};

  std::string dock_pose_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string scan_topic_;

  double control_frequency_;

  double k_x_;
  double k_y_;
  double k_yaw_;

  double v_reverse_max_;
  double v_reverse_min_;
  double w_max_;

  double stop_distance_;
  double slowdown_distance_;
  double y_align_threshold_;
  double yaw_align_threshold_;

  double dock_pose_timeout_;
  double stale_pose_timeout_;
  bool stop_on_lost_pose_;

  double min_range_threshold_;

  bool require_fresh_odom_;
  double odom_timeout_;

  bool has_dock_pose_ = false;
  bool has_odom_ = false;
  bool has_scan_ = false;

  geometry_msgs::msg::PoseStamped last_dock_pose_;
  nav_msgs::msg::Odometry last_odom_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  rclcpp::Time last_dock_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_dock_pose_change_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RearDockingControllerNode>());
  rclcpp::shutdown();
  return 0;
}