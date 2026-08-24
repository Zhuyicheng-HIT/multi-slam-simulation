#include "uf_safety_supervisor/obstacle_safety_core.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uf_interfaces/msg/obstacle_safety_state.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace uf_safety_supervisor
{
namespace
{

double stamp_s(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

Eigen::Quaterniond quaternion(const geometry_msgs::msg::Quaternion & value)
{
  return {value.w, value.x, value.y, value.z};
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
    std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
    quaternion(q).norm() > 1.0e-6;
}

}  // namespace

class RawObstacleSafetyMonitor : public rclcpp::Node
{
public:
  RawObstacleSafetyMonitor() : Node("raw_obstacle_safety_monitor")
  {
    ObstacleSafetyConfig config;
    config.body_front_m = declare_parameter<double>("body.front_m", config.body_front_m);
    config.body_half_width_m = declare_parameter<double>("body.half_width_m", config.body_half_width_m);
    config.body_half_height_m = declare_parameter<double>("body.half_height_m", config.body_half_height_m);
    config.lateral_margin_m = declare_parameter<double>("safety.lateral_margin_m", config.lateral_margin_m);
    config.vertical_margin_m = declare_parameter<double>("safety.vertical_margin_m", config.vertical_margin_m);
    config.safety_margin_m = declare_parameter<double>("safety.margin_m", config.safety_margin_m);
    config.caution_margin_m = declare_parameter<double>("safety.caution_margin_m", config.caution_margin_m);
    config.reaction_time_s = declare_parameter<double>("braking.reaction_time_s", config.reaction_time_s);
    config.maximum_deceleration_mps2 = declare_parameter<double>(
      "braking.maximum_deceleration_mps2", config.maximum_deceleration_mps2);
    config.brake_ttc_s = declare_parameter<double>("braking.brake_ttc_s", config.brake_ttc_s);
    config.caution_ttc_s = declare_parameter<double>("braking.caution_ttc_s", config.caution_ttc_s);
    config.minimum_valid_range_m = declare_parameter<double>("raw.minimum_range_m", config.minimum_valid_range_m);
    config.maximum_valid_range_m = declare_parameter<double>("raw.maximum_range_m", config.maximum_valid_range_m);
    core_ = std::make_unique<ObstacleSafetyCore>(config);

    raw_timeout_s_ = declare_parameter<double>("raw.timeout_s", 0.20);
    odom_timeout_s_ = declare_parameter<double>("motion.timeout_s", 0.25);
    future_tolerance_s_ = declare_parameter<double>("raw.future_tolerance_s", 0.02);
    candidate_timeout_s_ = declare_parameter<double>("candidate.timeout_s", 0.50);
    path_timeout_s_ = declare_parameter<double>("candidate.path_timeout_s", 0.25);
    minimum_points_ = static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<int>("raw.minimum_points", 20)));
    const auto xyz = declare_parameter<std::vector<double>>(
      "extrinsic.body_from_lidar_xyz", {0.0, 0.0, 0.0});
    const auto rpy = declare_parameter<std::vector<double>>(
      "extrinsic.body_from_lidar_rpy", {0.0, 0.0, 0.0});
    if (xyz.size() != 3U || rpy.size() != 3U) {
      throw std::invalid_argument("body_from_lidar extrinsic must contain three xyz and rpy values");
    }
    body_from_lidar_ = Eigen::Isometry3d::Identity();
    body_from_lidar_.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
    body_from_lidar_.linear() = (
      Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();

    const auto raw_topic = declare_parameter<std::string>("raw_lidar_topic", "/livox/lidar");
    const auto odom_topic = declare_parameter<std::string>("odometry_topic", "/fusion/unified/odom");
    const auto body_motion_topic = declare_parameter<std::string>(
      "body_motion_topic", "/mavros/local_position/odom");
    const auto target_topic = declare_parameter<std::string>(
      "candidate_setpoint_topic", "/autonomy/selected_candidate_pose");
    const auto path_topic = declare_parameter<std::string>("candidate_path_topic", "/autonomy/candidate_path");
    const auto relocalization_path_topic = declare_parameter<std::string>(
      "relocalization_candidate_path_topic", "/autonomy/relocalization_candidate_path");

    state_pub_ = create_publisher<uf_interfaces::msg::ObstacleSafetyState>(
      "/safety/raw_obstacle_state", rclcpp::QoS(10).reliable());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/safety/raw_obstacle_diagnostics", 10);
    raw_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      raw_topic, rclcpp::SensorDataQoS(),
      [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {on_raw(*message);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {on_odom(*message);});
    motion_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      body_motion_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        motion_odom_ = *message;
        motion_received_ = true;
      });
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      target_topic, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {target_ = *message;});
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic, 10, [this](nav_msgs::msg::Path::ConstSharedPtr message) {path_ = *message;});
    relocalization_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      relocalization_path_topic, 10,
      [this](nav_msgs::msg::Path::ConstSharedPtr message) {relocalization_path_ = *message;});
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {watchdog();});
  }

private:
  void on_odom(const nav_msgs::msg::Odometry & message)
  {
    odom_ = message;
    odom_received_ = true;
  }

  Eigen::Vector3d desired_direction_body(bool & localization_valid)
  {
    localization_valid = odom_received_ && finite_pose(odom_.pose.pose);
    Eigen::Vector3d velocity{
      motion_odom_.twist.twist.linear.x, motion_odom_.twist.twist.linear.y,
      motion_odom_.twist.twist.linear.z};
    if (!localization_valid) {return velocity;}
    const Eigen::Vector3d current{
      odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z};
    Eigen::Vector3d goal = current;
    const double now = get_clock()->now().seconds();
    const double path_stamp = stamp_s(path_.header.stamp);
    const double relocalization_path_stamp = stamp_s(relocalization_path_.header.stamp);
    const double target_stamp = stamp_s(target_.header.stamp);
    const bool path_fresh = path_stamp > 0.0 && now >= path_stamp &&
      now - path_stamp <= path_timeout_s_;
    const bool relocalization_path_fresh = relocalization_path_stamp > 0.0 &&
      now >= relocalization_path_stamp && now - relocalization_path_stamp <= path_timeout_s_;
    const bool target_fresh = target_stamp > 0.0 && now >= target_stamp &&
      now - target_stamp <= candidate_timeout_s_;
    if (relocalization_path_fresh && !relocalization_path_.poses.empty() &&
      finite_pose(relocalization_path_.poses.back().pose))
    {
      goal = {relocalization_path_.poses.back().pose.position.x,
        relocalization_path_.poses.back().pose.position.y,
        relocalization_path_.poses.back().pose.position.z};
    } else if (path_fresh && !path_.poses.empty() && finite_pose(path_.poses.front().pose)) {
      goal = {path_.poses.front().pose.position.x, path_.poses.front().pose.position.y,
        path_.poses.front().pose.position.z};
    } else if (target_fresh && finite_pose(target_.pose)) {
      goal = {target_.pose.position.x, target_.pose.position.y, target_.pose.position.z};
    } else {
      return velocity;
    }
    Eigen::Quaterniond world_from_body = quaternion(odom_.pose.pose.orientation).normalized();
    return world_from_body.conjugate() * (goal - current);
  }

  void on_raw(const livox_ros_driver2::msg::CustomMsg & message)
  {
    const double now = get_clock()->now().seconds();
    const double scan_stamp = stamp_s(message.header.stamp);
    bool timestamps_valid = std::isfinite(scan_stamp) && scan_stamp > 0.0 &&
      scan_stamp <= now + future_tolerance_s_ &&
      (last_raw_stamp_s_ <= 0.0 || scan_stamp > last_raw_stamp_s_);
    if (timestamps_valid) {last_raw_stamp_s_ = scan_stamp;}
    last_raw_arrival_s_ = now;

    ObstacleSafetyInput input;
    input.sensor_age_s = std::max(0.0, now - scan_stamp);
    input.timestamps_valid = timestamps_valid;
    input.raw_sensor_healthy = timestamps_valid && input.sensor_age_s <= raw_timeout_s_ &&
      message.points.size() >= minimum_points_;
    input.body_points.reserve(message.points.size());
    for (const auto & source : message.points) {
      const Eigen::Vector3d lidar(source.x, source.y, source.z);
      input.body_points.push_back(lidar.allFinite() ? body_from_lidar_ * lidar : lidar);
    }
    bool localization_valid = false;
    input.desired_direction_body = desired_direction_body(localization_valid);
    input.localization_valid = localization_valid;
    const double motion_stamp = motion_received_ ? stamp_s(motion_odom_.header.stamp) : 0.0;
    input.body_velocity = {
      motion_odom_.twist.twist.linear.x, motion_odom_.twist.twist.linear.y,
      motion_odom_.twist.twist.linear.z};
    input.motion_finite = input.body_velocity.allFinite() && motion_received_ &&
      motion_stamp > 0.0 && now >= motion_stamp && now - motion_stamp <= odom_timeout_s_;
    publish(core_->evaluate(input), scan_stamp, input.raw_sensor_healthy, input.sensor_age_s);
  }

  void watchdog()
  {
    const double now = get_clock()->now().seconds();
    if (last_raw_arrival_s_ <= 0.0 || now - last_raw_arrival_s_ > raw_timeout_s_) {
      ObstacleSafetyInput input;
      input.raw_sensor_healthy = false;
      input.sensor_age_s = last_raw_arrival_s_ <= 0.0 ?
        std::numeric_limits<double>::infinity() : now - last_raw_arrival_s_;
      publish(core_->evaluate(input), now, false, input.sensor_age_s);
    }
  }

  void publish(
    const ObstacleSafetyResult & result, const double stamp, const bool healthy,
    const double sensor_age_s)
  {
    uf_interfaces::msg::ObstacleSafetyState message;
    message.header.stamp = rclcpp::Time(static_cast<std::int64_t>(stamp * 1.0e9));
    message.header.frame_id = "base_link";
    message.state = static_cast<std::uint8_t>(result.state);
    message.raw_sensor_healthy = healthy;
    message.fail_closed = result.fail_closed;
    message.nearest_clearance_m = static_cast<float>(result.nearest_clearance_m);
    message.path_clearance_m = static_cast<float>(result.path_clearance_m);
    message.time_to_collision_s = static_cast<float>(result.time_to_collision_s);
    message.stopping_distance_m = static_cast<float>(result.stopping_distance_m);
    message.raw_sensor_age_s = static_cast<float>(sensor_age_s);
    message.reason = result.reason;
    state_pub_->publish(message);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header = message.header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "raw_obstacle_safety_monitor";
    status.hardware_id = "raw_mid360";
    status.level = result.state == ObstacleState::kClear ?
      diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = std::string(to_string(result.state)) + ":" + result.reason;
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::unique_ptr<ObstacleSafetyCore> core_;
  double raw_timeout_s_{0.2};
  double odom_timeout_s_{0.25};
  double future_tolerance_s_{0.02};
  double candidate_timeout_s_{0.50};
  double path_timeout_s_{0.25};
  std::size_t minimum_points_{20U};
  double last_raw_stamp_s_{0.0};
  double last_raw_arrival_s_{0.0};
  bool odom_received_{false};
  bool motion_received_{false};
  Eigen::Isometry3d body_from_lidar_{Eigen::Isometry3d::Identity()};
  nav_msgs::msg::Odometry odom_;
  nav_msgs::msg::Odometry motion_odom_;
  geometry_msgs::msg::PoseStamped target_;
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path relocalization_path_;
  rclcpp::Publisher<uf_interfaces::msg::ObstacleSafetyState>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr raw_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr motion_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr relocalization_path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uf_safety_supervisor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uf_safety_supervisor::RawObstacleSafetyMonitor>());
  rclcpp::shutdown();
  return 0;
}
