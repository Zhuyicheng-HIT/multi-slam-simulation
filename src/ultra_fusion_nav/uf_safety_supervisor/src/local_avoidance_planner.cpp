#include "uf_safety_supervisor/local_avoidance_core.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <uf_interfaces/msg/local_avoidance_status.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
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

std::int64_t voxel_key(const Eigen::Vector3d & point, const double resolution)
{
  const auto x = static_cast<std::int64_t>(std::floor(point.x() / resolution));
  const auto y = static_cast<std::int64_t>(std::floor(point.y() / resolution));
  return (x * 73856093LL) ^ (y * 19349663LL);
}

}  // namespace

class LocalAvoidancePlannerNode : public rclcpp::Node
{
public:
  LocalAvoidancePlannerNode() : Node("local_avoidance_planner")
  {
    LocalPlannerConfig config;
    config.resolution_m = declare_parameter<double>("grid.resolution_m", config.resolution_m);
    config.obstacle_inflation_m = declare_parameter<double>(
      "grid.obstacle_inflation_m", config.obstacle_inflation_m);
    config.verification_clearance_m = declare_parameter<double>(
      "grid.verification_clearance_m", config.verification_clearance_m);
    config.planning_horizon_m = declare_parameter<double>(
      "planner.horizon_m", config.planning_horizon_m);
    config.side_padding_m = declare_parameter<double>(
      "planner.side_padding_m", config.side_padding_m);
    config.goal_tolerance_m = declare_parameter<double>(
      "planner.goal_tolerance_m", config.goal_tolerance_m);
    config.waypoint_spacing_m = declare_parameter<double>(
      "planner.waypoint_spacing_m", config.waypoint_spacing_m);
    config.maximum_expansions = static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<int>("planner.maximum_expansions", 30000)));
    planner_ = std::make_unique<ConservativeLocalPlanner>(config);
    goal_tolerance_m_ = config.goal_tolerance_m;
    waypoint_reached_m_ = declare_parameter<double>("planner.waypoint_reached_m", 0.35);
    raw_timeout_s_ = declare_parameter<double>("raw.timeout_s", 0.20);
    odom_timeout_s_ = declare_parameter<double>("motion.timeout_s", 0.25);
    mission_timeout_s_ = declare_parameter<double>("mission.timeout_s", 0.50);
    future_tolerance_s_ = declare_parameter<double>("raw.future_tolerance_s", 0.02);
    planning_timeout_ms_ = declare_parameter<double>("planner.timeout_ms", 80.0);
    vertical_limit_m_ = declare_parameter<double>("grid.vertical_limit_m", 0.40);
    minimum_range_m_ = declare_parameter<double>("raw.minimum_range_m", 0.12);
    maximum_range_m_ = declare_parameter<double>("raw.maximum_range_m", 40.0);
    downsample_m_ = declare_parameter<double>("grid.raw_downsample_m", 0.20);
    minimum_points_ = static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<int>("raw.minimum_points", 20)));
    const auto xyz = declare_parameter<std::vector<double>>(
      "extrinsic.body_from_lidar_xyz", {0.0, 0.0, 0.0});
    const auto rpy = declare_parameter<std::vector<double>>(
      "extrinsic.body_from_lidar_rpy", {0.0, 0.0, 0.0});
    if (xyz.size() != 3U || rpy.size() != 3U) {
      throw std::invalid_argument("body_from_lidar extrinsic must contain xyz/rpy triples");
    }
    body_from_lidar_ = Eigen::Isometry3d::Identity();
    body_from_lidar_.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
    body_from_lidar_.linear() = (
      Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();

    const auto raw_topic = declare_parameter<std::string>("raw_lidar_topic", "/livox/lidar");
    const auto odom_topic = declare_parameter<std::string>(
      "odometry_topic", "/mavros/local_position/odom");
    const auto mission_topic = declare_parameter<std::string>(
      "mission_intent_topic", "/autonomy/intent/mission/pose");
    planner_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/autonomy/intent/planner/pose", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/autonomy/candidate_path", 10);
    status_pub_ = create_publisher<uf_interfaces::msg::LocalAvoidanceStatus>(
      "/safety/local_avoidance_status", 10);
    raw_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      raw_topic, rclcpp::SensorDataQoS(),
      [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {on_raw(*message);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        odom_ = *message;
        odom_arrival_s_ = get_clock()->now().seconds();
        odom_received_ = true;
      });
    mission_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      mission_topic, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        mission_ = *message;
        mission_arrival_s_ = get_clock()->now().seconds();
        mission_received_ = true;
      });
    localization_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/safety/localization_hold", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        localization_hold_ = message->data;
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
  }

private:
  void on_raw(const livox_ros_driver2::msg::CustomMsg & message)
  {
    const double now = get_clock()->now().seconds();
    const double stamp = stamp_s(message.header.stamp);
    const bool timestamp_valid = std::isfinite(stamp) && stamp > 0.0 &&
      stamp <= now + future_tolerance_s_ &&
      (last_raw_stamp_s_ <= 0.0 || stamp > last_raw_stamp_s_);
    if (!timestamp_valid) {
      raw_valid_ = false;
      raw_arrival_s_ = now;
      raw_reason_ = "raw_timestamp_invalid";
      return;
    }
    last_raw_stamp_s_ = stamp;
    raw_arrival_s_ = now;
    body_obstacles_.clear();
    std::unordered_set<std::int64_t> occupied;
    occupied.reserve(message.points.size());
    bool finite = true;
    for (const auto & source : message.points) {
      const Eigen::Vector3d lidar{source.x, source.y, source.z};
      if (!lidar.allFinite()) {finite = false; break;}
      const Eigen::Vector3d body = body_from_lidar_ * lidar;
      const double range = body.norm();
      if (range < minimum_range_m_ || range > maximum_range_m_ ||
        std::abs(body.z()) > vertical_limit_m_)
      {
        continue;
      }
      if (occupied.insert(voxel_key(body, downsample_m_)).second) {
        body_obstacles_.push_back(body);
      }
    }
    raw_valid_ = finite && message.points.size() >= minimum_points_;
    raw_reason_ = raw_valid_ ? "raw_local_map_fresh" : "raw_points_invalid_or_sparse";
  }

  bool current_pose(Eigen::Vector3d & position, Eigen::Quaterniond & orientation) const
  {
    if (!odom_received_ || !finite_pose(odom_.pose.pose)) {return false;}
    position = {odom_.pose.pose.position.x, odom_.pose.pose.position.y,
      odom_.pose.pose.position.z};
    orientation = quaternion(odom_.pose.pose.orientation).normalized();
    return position.allFinite() && orientation.coeffs().allFinite();
  }

  std::vector<Eigen::Vector3d> active_path_body(
    const Eigen::Vector3d & position, const Eigen::Quaterniond & world_from_body) const
  {
    std::vector<Eigen::Vector3d> result;
    result.reserve(active_world_path_.size() + 1U);
    result.push_back(Eigen::Vector3d::Zero());
    for (const auto & world : active_world_path_) {
      // Keep the ordered path topology intact during collision verification.
      // Removing every nearby point can skip a required corner and create an
      // unsafe shortcut to a later waypoint.  Reached points are consumed only
      // from the front, in publish_active().
      result.push_back(world_from_body.conjugate() * (world - position));
    }
    return result;
  }

  void publish_hold(const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation)
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "map";
    message.pose.position.x = position.x();
    message.pose.position.y = position.y();
    message.pose.position.z = position.z();
    message.pose.orientation.w = orientation.w();
    message.pose.orientation.x = orientation.x();
    message.pose.orientation.y = orientation.y();
    message.pose.orientation.z = orientation.z();
    planner_pub_->publish(message);
  }

  void publish_active(
    const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation)
  {
    while (!active_world_path_.empty() &&
      (active_world_path_.front() - position).norm() <= waypoint_reached_m_)
    {
      active_world_path_.erase(active_world_path_.begin());
    }
    if (active_world_path_.empty()) {return;}
    geometry_msgs::msg::PoseStamped command;
    command.header.stamp = get_clock()->now();
    command.header.frame_id = "map";
    command.pose.position.x = active_world_path_.front().x();
    command.pose.position.y = active_world_path_.front().y();
    command.pose.position.z = active_world_path_.front().z();
    command.pose.orientation.w = orientation.w();
    command.pose.orientation.x = orientation.x();
    command.pose.orientation.y = orientation.y();
    command.pose.orientation.z = orientation.z();
    planner_pub_->publish(command);

    nav_msgs::msg::Path path;
    path.header = command.header;
    for (const auto & waypoint : active_world_path_) {
      auto pose = command;
      pose.pose.position.x = waypoint.x();
      pose.pose.position.y = waypoint.y();
      pose.pose.position.z = waypoint.z();
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  void publish_verified_mission()
  {
    // Once a previous detour/hold has published a planner intent, silently
    // stopping publication leaves that old candidate stale in the arbiter and
    // correctly triggers fail-closed forever.  NAVIGATING means the direct
    // mission segment has just passed the same Raw-obstacle verification, so
    // keep the ordinary-priority planner candidate fresh without bypassing
    // either the arbiter or the independent obstacle veto.
    auto command = mission_;
    command.header.stamp = get_clock()->now();
    command.header.frame_id = "map";
    planner_pub_->publish(command);
  }

  void clear_candidate_path()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = get_clock()->now();
    path.header.frame_id = "map";
    path_pub_->publish(path);
  }

  void publish_status(const bool healthy, const bool fail_closed)
  {
    uf_interfaces::msg::LocalAvoidanceStatus message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "map";
    message.state = static_cast<std::uint8_t>(fsm_.state());
    message.healthy = healthy;
    message.fail_closed = fail_closed;
    message.replan_count = replan_count_;
    message.consecutive_replans = consecutive_replans_;
    message.path_points = static_cast<std::uint32_t>(active_world_path_.size());
    message.planning_latency_ms = static_cast<float>(last_planning_latency_ms_);
    message.trajectory_clearance_m = static_cast<float>(last_clearance_m_);
    message.reason = fsm_.reason() + ":" + last_detail_;
    status_pub_->publish(message);
  }

  void tick()
  {
    const double now = get_clock()->now().seconds();
    Eigen::Vector3d position;
    Eigen::Quaterniond world_from_body;
    const bool pose_valid = current_pose(position, world_from_body) &&
      now >= odom_arrival_s_ && now - odom_arrival_s_ <= odom_timeout_s_;
    const bool raw_fresh = raw_valid_ && now >= raw_arrival_s_ &&
      now - raw_arrival_s_ <= raw_timeout_s_;
    const bool mission_fresh = mission_received_ && finite_pose(mission_.pose) &&
      now >= mission_arrival_s_ && now - mission_arrival_s_ <= mission_timeout_s_;
    const bool localization_healthy = pose_valid && !localization_hold_;
    AvoidanceEvent event;
    event.raw_fresh = raw_fresh;
    event.localization_healthy = localization_healthy;
    event.mission_fresh = mission_fresh;
    if (!pose_valid || !mission_fresh) {
      fsm_.update(event);
      clear_candidate_path();
      publish_status(false, true);
      return;
    }

    const Eigen::Vector3d goal_world{mission_.pose.position.x, mission_.pose.position.y,
      mission_.pose.position.z};
    const Eigen::Vector3d goal_body = world_from_body.conjugate() * (goal_world - position);
    std::vector<Eigen::Vector3d> path_body;
    if (fsm_.state() == AvoidanceState::kResume && !active_world_path_.empty()) {
      path_body = active_path_body(position, world_from_body);
    } else {
      path_body = {Eigen::Vector3d::Zero(), goal_body};
    }
    double current_clearance = 0.0;
    const bool path_valid = raw_fresh && planner_->verify(
      path_body, body_obstacles_, &current_clearance);
    // The planner evaluates the same raw scan directly.  Do not feed the
    // monitor's previous-cycle BRAKE back into path search: that creates a
    // BRAKE -> no-path -> BRAKE latch.  The independent monitor still has
    // higher priority in the arbiter and can veto every published detour.
    event.path_blocked = !path_valid;
    event.goal_reached = (goal_world - position).norm() <= goal_tolerance_m_ ||
      (fsm_.state() == AvoidanceState::kResume && active_world_path_.empty());
    last_clearance_m_ = current_clearance;

    if (fsm_.state() == AvoidanceState::kReplan && raw_fresh && localization_healthy) {
      const auto started = std::chrono::steady_clock::now();
      const auto plan = planner_->plan(Eigen::Vector3d::Zero(), goal_body, body_obstacles_);
      last_planning_latency_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      event.plan_attempted = true;
      event.plan_valid = plan.success && plan.verified &&
        last_planning_latency_ms_ <= planning_timeout_ms_;
      last_detail_ = event.plan_valid ? plan.reason :
        (last_planning_latency_ms_ > planning_timeout_ms_ ? "planner_timeout" : plan.reason);
      last_clearance_m_ = plan.minimum_clearance_m;
      ++replan_count_;
      ++consecutive_replans_;
      active_world_path_.clear();
      if (event.plan_valid) {
        for (std::size_t index = 1U; index < plan.path.size(); ++index) {
          active_world_path_.push_back(position + world_from_body * plan.path[index]);
        }
      }
    } else if (fsm_.state() == AvoidanceState::kTrajectoryVerify) {
      const auto candidate_body = active_path_body(position, world_from_body);
      event.plan_valid = raw_fresh && planner_->verify(
        candidate_body, body_obstacles_, &last_clearance_m_);
      last_detail_ = event.plan_valid ? "raw_trajectory_reverified" :
        "raw_trajectory_conflict";
    } else {
      last_detail_ = raw_fresh ? raw_reason_ : "raw_map_stale";
    }

    const auto previous = fsm_.state();
    const auto state = fsm_.update(event);
    if (state == AvoidanceState::kNavigating) {
      active_world_path_.clear();
      consecutive_replans_ = 0U;
      clear_candidate_path();
      publish_verified_mission();
    } else if (state == AvoidanceState::kResume) {
      publish_active(position, world_from_body);
    } else {
      publish_hold(position, world_from_body);
      clear_candidate_path();
    }
    if (previous != state) {
      RCLCPP_INFO(get_logger(), "%s -> %s: %s", to_string(previous), to_string(state),
        fsm_.reason().c_str());
    }
    publish_status(raw_fresh && localization_healthy, state == AvoidanceState::kHoverRequired);
  }

  std::unique_ptr<ConservativeLocalPlanner> planner_;
  LocalAvoidanceStateMachine fsm_;
  double goal_tolerance_m_{0.35};
  double waypoint_reached_m_{0.35};
  double raw_timeout_s_{0.20};
  double odom_timeout_s_{0.25};
  double mission_timeout_s_{0.50};
  double future_tolerance_s_{0.02};
  double planning_timeout_ms_{80.0};
  double vertical_limit_m_{0.40};
  double minimum_range_m_{0.12};
  double maximum_range_m_{40.0};
  double downsample_m_{0.20};
  std::size_t minimum_points_{20U};
  double last_raw_stamp_s_{0.0};
  double raw_arrival_s_{0.0};
  double odom_arrival_s_{0.0};
  double mission_arrival_s_{0.0};
  double last_planning_latency_ms_{0.0};
  double last_clearance_m_{std::numeric_limits<double>::infinity()};
  std::uint32_t replan_count_{0U};
  std::uint32_t consecutive_replans_{0U};
  bool raw_valid_{false};
  bool odom_received_{false};
  bool mission_received_{false};
  bool localization_hold_{false};
  std::string raw_reason_{"raw_not_received"};
  std::string last_detail_{"initializing"};
  Eigen::Isometry3d body_from_lidar_{Eigen::Isometry3d::Identity()};
  std::vector<Eigen::Vector3d> body_obstacles_;
  std::vector<Eigen::Vector3d> active_world_path_;
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::PoseStamped mission_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr planner_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<uf_interfaces::msg::LocalAvoidanceStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr raw_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uf_safety_supervisor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uf_safety_supervisor::LocalAvoidancePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
