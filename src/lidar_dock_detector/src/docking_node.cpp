#include "lidar_dock_detector/docking_node.hpp"

#include <cmath>
#include <algorithm>

namespace lidar_dock_detector
{

DockingNode::DockingNode(const rclcpp::NodeOptions & opts)
: Node("docking_node", opts)
{
  // Declare & get parameters
  declare_parameter("scan_topic",          "/scan_front_filter");
  declare_parameter("base_frame",          "base_link");
  declare_parameter("i_peak",              43.0);
  declare_parameter("i_valley",            29.0);
  declare_parameter("valley_search_range", 19);
  declare_parameter("max_detect_range",    3.0);
  declare_parameter("max_fail_count",      5);
  declare_parameter("lrf_forward_offset",  0.30);
  declare_parameter("tape_distance",       0.375);
  declare_parameter("rotate_speed",        0.3);
  declare_parameter("backup_speed",        0.05);
  declare_parameter("backup_distance",     0.30);
  declare_parameter("staging_pose_x",     -0.420);
  declare_parameter("staging_pose_y",      0.580);
  declare_parameter("staging_pose_yaw",    1.57);

  scan_topic_           = get_parameter("scan_topic").as_string();
  base_frame_           = get_parameter("base_frame").as_string();
  i_peak_               = get_parameter("i_peak").as_double();
  i_valley_             = get_parameter("i_valley").as_double();
  valley_search_range_  = get_parameter("valley_search_range").as_int();
  max_detect_range_     = get_parameter("max_detect_range").as_double();
  max_fail_count_       = get_parameter("max_fail_count").as_int();
  lrf_forward_offset_   = get_parameter("lrf_forward_offset").as_double();
  tape_distance_        = get_parameter("tape_distance").as_double();
  rotate_speed_         = get_parameter("rotate_speed").as_double();
  backup_speed_         = get_parameter("backup_speed").as_double();
  backup_distance_      = get_parameter("backup_distance").as_double();
  staging_pose_x_       = get_parameter("staging_pose_x").as_double();
  staging_pose_y_       = get_parameter("staging_pose_y").as_double();
  staging_pose_yaw_     = get_parameter("staging_pose_yaw").as_double();

  // Publishers / subscribers
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&DockingNode::scanCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&DockingNode::odomCallback, this, std::placeholders::_1));

  // Trigger service
  trigger_srv_ = create_service<std_srvs::srv::Trigger>(
    "/start_docking",
    std::bind(&DockingNode::triggerDocking, this,
              std::placeholders::_1, std::placeholders::_2));

  // Nav2 action client
  nav_client_ = rclcpp_action::create_client<NavToPose>(this, "/navigate_to_pose");

  // 20 Hz control loop
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&DockingNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "DockingNode ready. Call /start_docking to begin.");
}

// ── Callbacks ────────────────────────────────────────────────────────────────

void DockingNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(scan_mutex_);
  latest_scan_ = msg;
}

void DockingNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(odom_mutex_);
  latest_odom_ = msg;
}

void DockingNode::triggerDocking(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (state_ != State::IDLE) {
    res->success = false;
    res->message = "Already docking";
    return;
  }
  state_          = State::NAVIGATE_TO_STAGING;
  nav_goal_sent_  = false;
  nav_done_       = false;
  nav_success_    = false;
  fail_count_     = 0;
  retry_count_    = 0;
  rotate_started_ = false;
  backup_started_ = false;
  res->success    = true;
  res->message    = "Docking started";
  RCLCPP_INFO(get_logger(), "Docking triggered → NAVIGATE_TO_STAGING");
}

// ── Main control loop ─────────────────────────────────────────────────────────

void DockingNode::controlLoop()
{
  switch (state_) {
    case State::IDLE:             break;
    case State::NAVIGATE_TO_STAGING: handleNavigate(); break;
    case State::DETECT_DOCK:      handleDetect();   break;
    case State::ROTATE_180:       handleRotate();   break;
    case State::MOVE_BACKWARD:    handleBackup();   break;
    case State::DOCKED:
      publishZeroVel();
      break;
    case State::FAILED:
      publishZeroVel();
      break;
  }
}

// ── STATE: NAVIGATE_TO_STAGING ────────────────────────────────────────────────

void DockingNode::handleNavigate()
{
  if (!nav_goal_sent_) {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "Nav2 action server not available yet...");
      return;
    }

    auto goal = NavToPose::Goal();
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp    = now();
    goal.pose.pose.position.x = staging_pose_x_;
    goal.pose.pose.position.y = staging_pose_y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, staging_pose_yaw_);
    goal.pose.pose.orientation.x = q.x();
    goal.pose.pose.orientation.y = q.y();
    goal.pose.pose.orientation.z = q.z();
    goal.pose.pose.orientation.w = q.w();

    auto send_opts = rclcpp_action::Client<NavToPose>::SendGoalOptions();
    send_opts.result_callback = [this](const auto & result) {
      nav_done_ = true;
      nav_success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
      if (nav_success_) {
        RCLCPP_INFO(get_logger(), "Reached staging pose → DETECT_DOCK");
        state_ = State::DETECT_DOCK;
      } else {
        RCLCPP_ERROR(get_logger(), "Navigation failed → FAILED");
        state_ = State::FAILED;
      }
    };

    nav_client_->async_send_goal(goal, send_opts);
    nav_goal_sent_ = true;
    RCLCPP_INFO(get_logger(), "Sent Nav2 goal to staging pose (%.3f, %.3f, yaw=%.3f)",
      staging_pose_x_, staging_pose_y_, staging_pose_yaw_);
  }
  // Wait for result callback to fire
}

// ── STATE: DETECT_DOCK ────────────────────────────────────────────────────────

void DockingNode::handleDetect()
{
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  {
    std::lock_guard<std::mutex> lk(scan_mutex_);
    scan = latest_scan_;
  }

  if (!scan) { return; }

  auto reflectors = detectReflectors(*scan);

  if (reflectors.size() == 2u) {
    fail_count_ = 0;

    // Smaller beam index = RIGHT (y < 0), larger = LEFT (y > 0)
    const Reflector & right = (reflectors[0].peak_idx < reflectors[1].peak_idx)
                              ? reflectors[0] : reflectors[1];
    const Reflector & left  = (reflectors[0].peak_idx < reflectors[1].peak_idx)
                              ? reflectors[1] : reflectors[0];

    double cx = (left.x + right.x) / 2.0 + lrf_forward_offset_;
    double cy = (left.y + right.y) / 2.0;

    RCLCPP_INFO(get_logger(),
      "Dock detected at base_link (%.3f, %.3f) → ROTATE_180", cx, cy);

    rotate_started_ = false;
    state_ = State::ROTATE_180;
  } else {
    fail_count_++;
    if (fail_count_ >= max_fail_count_) {
      retry_count_++;
      fail_count_ = 0;
      if (retry_count_ >= 3) {
        RCLCPP_ERROR(get_logger(), "Dock not found after 3 retries → FAILED");
        state_ = State::FAILED;
      } else {
        RCLCPP_WARN(get_logger(), "Dock lost, retry %d/3", retry_count_);
      }
    }
  }
}

// ── STATE: ROTATE_180 ─────────────────────────────────────────────────────────

void DockingNode::handleRotate()
{
  nav_msgs::msg::Odometry::SharedPtr odom;
  {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    odom = latest_odom_;
  }
  if (!odom) { return; }

  // Extract yaw from quaternion directly (no tf2 conversion needed)
  const auto & q = odom->pose.pose.orientation;
  double current_yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  if (!rotate_started_) {
    rotate_start_yaw_ = current_yaw;
    rotate_started_   = true;
    RCLCPP_INFO(get_logger(), "Rotating 180° from yaw=%.3f", rotate_start_yaw_);
  }

  double delta = current_yaw - rotate_start_yaw_;
  // Normalize to [-π, π]
  while (delta >  M_PI) delta -= 2 * M_PI;
  while (delta < -M_PI) delta += 2 * M_PI;

  if (std::abs(std::abs(delta) - M_PI) < 0.05) {
    publishZeroVel();
    RCLCPP_INFO(get_logger(), "Rotation complete → MOVE_BACKWARD");
    backup_started_ = false;
    state_ = State::MOVE_BACKWARD;
    return;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = rotate_speed_;
  cmd_vel_pub_->publish(cmd);
}

// ── STATE: MOVE_BACKWARD ──────────────────────────────────────────────────────

void DockingNode::handleBackup()
{
  nav_msgs::msg::Odometry::SharedPtr odom;
  {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    odom = latest_odom_;
  }
  if (!odom) { return; }

  double cx = odom->pose.pose.position.x;
  double cy = odom->pose.pose.position.y;

  if (!backup_started_) {
    backup_start_x_ = cx;
    backup_start_y_ = cy;
    backup_started_ = true;
    RCLCPP_INFO(get_logger(), "Moving backward %.2fm", backup_distance_);
  }

  double dist = std::hypot(cx - backup_start_x_, cy - backup_start_y_);

  if (dist >= backup_distance_) {
    publishZeroVel();
    RCLCPP_INFO(get_logger(), "Docked! Traveled %.3fm → DOCKED", dist);
    state_ = State::DOCKED;
    return;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -backup_speed_;
  cmd_vel_pub_->publish(cmd);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

void DockingNode::publishZeroVel()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
}

// ── detectReflectors() ────────────────────────────────────────────────────────

std::vector<Reflector> DockingNode::detectReflectors(
  const sensor_msgs::msg::LaserScan & scan) const
{
  std::vector<Reflector> result;
  const int N = static_cast<int>(scan.ranges.size());

  if (static_cast<int>(scan.intensities.size()) != N) { return result; }

  const int margin = valley_search_range_ + 2;

  for (int i = margin; i < N - margin; ++i) {
    const float I_i = scan.intensities[i];
    if (I_i < static_cast<float>(i_peak_)) { continue; }

    // Range check
    const float r_i = scan.ranges[i];
    if (!std::isfinite(r_i) || r_i < scan.range_min || r_i > scan.range_max) { continue; }
    if (static_cast<double>(r_i) > max_detect_range_) { continue; }

    // Local maximum check
    bool is_max = true;
    for (int k = i - 1; k >= std::max(0, i - valley_search_range_); --k) {
      if (scan.intensities[k] > I_i) { is_max = false; break; }
    }
    if (!is_max) { continue; }
    for (int k = i + 1; k <= std::min(N - 1, i + valley_search_range_); ++k) {
      if (scan.intensities[k] > I_i) { is_max = false; break; }
    }
    if (!is_max) { continue; }

    // Find valley left
    float vl_val = scan.intensities[i - 1];
    for (int k = i - 2; k >= std::max(0, i - valley_search_range_); --k) {
      vl_val = std::min(vl_val, scan.intensities[k]);
    }

    // Find valley right
    float vr_val = scan.intensities[i + 1];
    for (int k = i + 2; k <= std::min(N - 1, i + valley_search_range_); ++k) {
      vr_val = std::min(vr_val, scan.intensities[k]);
    }

    // Valley threshold check (valleys must be BELOW i_valley_)
    if (vl_val >= static_cast<float>(i_valley_) ||
        vr_val >= static_cast<float>(i_valley_)) { continue; }

    // Convert RPLidar angle → ROS frame
    double rplidar_angle = scan.angle_min + i * scan.angle_increment;
    double ros_angle = M_PI - rplidar_angle;
    while (ros_angle >  M_PI) ros_angle -= 2 * M_PI;
    while (ros_angle < -M_PI) ros_angle += 2 * M_PI;

    Reflector ref;
    ref.peak_idx = i;
    ref.x = static_cast<double>(r_i) * std::cos(ros_angle);
    ref.y = static_cast<double>(r_i) * std::sin(ros_angle);

    result.push_back(ref);

    // Skip past right valley to avoid double-detect
    i += valley_search_range_;
  }

  return result;
}

}  // namespace lidar_dock_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_dock_detector::DockingNode>());
  rclcpp::shutdown();
  return 0;
}
