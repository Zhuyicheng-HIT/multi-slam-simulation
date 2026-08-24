#include "uf_relocalization/active_relocalization_flight_core.hpp"
#include "uf_relocalization/active_relocalization_policy.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <uf_interfaces/msg/active_relocalization_status.hpp>
#include <uf_interfaces/msg/fusion_epoch.hpp>
#include <uf_interfaces/msg/obstacle_safety_state.hpp>
#include <uf_interfaces/msg/relocalization_result.hpp>
#include <uf_interfaces/msg/scheduler_state.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace uf_relocalization
{
namespace
{

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrap(const double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
    std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
    std::isfinite(q.w) && norm > 1.0e-6;
}

}  // namespace

class ActiveRelocalizationController : public rclcpp::Node
{
public:
  ActiveRelocalizationController() : Node("active_relocalization_controller")
  {
    ActiveFlightConfig flight;
    flight.initial_hold_s = declare_parameter<double>("initial_hold_s", flight.initial_hold_s);
    flight.active_timeout_s = declare_parameter<double>("active_timeout_s", flight.active_timeout_s);
    flight.recovery_dwell_s = declare_parameter<double>(
      "recovery_dwell_s", flight.recovery_dwell_s);
    flight.resume_dwell_s = declare_parameter<double>("resume_dwell_s", flight.resume_dwell_s);
    flight.maximum_failures = static_cast<std::uint32_t>(std::max<std::int64_t>(
      1, declare_parameter<int>("maximum_failures", 2)));
    flight_core_ = std::make_unique<ActiveRelocalizationFlightCore>(flight);

    ActiveRelocalizationPolicyConfig policy;
    policy.passive_attempt_limit = static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<int>("passive_attempt_limit", 3)));
    policy.yaw_scan_view_count = static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<int>("yaw_scan_view_count", 4)));
    policy.ego_motion_enabled = declare_parameter<bool>("safe_motion_enabled", true);
    passive_attempt_limit_ = policy.passive_attempt_limit;
    safe_motion_enabled_ = policy.ego_motion_enabled;
    policy_ = std::make_unique<ActiveRelocalizationPolicy>(policy);

    pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 0.25);
    scheduler_timeout_s_ = declare_parameter<double>("scheduler_timeout_s", 0.50);
    obstacle_timeout_s_ = declare_parameter<double>("obstacle_timeout_s", 0.25);
    passive_attempt_period_s_ = declare_parameter<double>("passive_attempt_period_s", 0.25);
    yaw_step_rad_ = declare_parameter<double>("yaw_step_deg", 90.0) * M_PI / 180.0;
    target_position_tolerance_m_ = declare_parameter<double>(
      "target_position_tolerance_m", 0.12);
    target_yaw_tolerance_rad_ = declare_parameter<double>(
      "target_yaw_tolerance_deg", 8.0) * M_PI / 180.0;
    target_settle_s_ = declare_parameter<double>("target_settle_s", 0.15);
    safe_motion_radius_m_ = declare_parameter<double>("safe_motion_radius_m", 0.35);
    if (!(pose_timeout_s_ > 0.0) || !(scheduler_timeout_s_ > 0.0) ||
      !(obstacle_timeout_s_ > 0.0) || !(passive_attempt_period_s_ > 0.0) ||
      !(yaw_step_rad_ > 0.0) || !(target_position_tolerance_m_ > 0.0) ||
      !(target_yaw_tolerance_rad_ > 0.0) || !(target_settle_s_ >= 0.0) ||
      !(safe_motion_radius_m_ > 0.0 && safe_motion_radius_m_ <= 0.75))
    {
      throw std::invalid_argument("invalid active relocalization controller configuration");
    }

    intent_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/autonomy/intent/relocalization/pose", 10);
    candidate_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/autonomy/relocalization_candidate_path", 10);
    status_pub_ = create_publisher<uf_interfaces::msg::ActiveRelocalizationStatus>(
      "/safety/active_relocalization_status", 10);

    request_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/relocalization/request", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {on_request(message->data);});
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        pose_ = *message;
        pose_arrival_s_ = get_clock()->now().seconds();
        pose_received_ = true;
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        odom_ = *message;
        odom_arrival_s_ = get_clock()->now().seconds();
        odom_received_ = true;
      });
    scheduler_sub_ = create_subscription<uf_interfaces::msg::SchedulerState>(
      "/reliability/scheduler_state", 20,
      [this](uf_interfaces::msg::SchedulerState::ConstSharedPtr message) {
        scheduler_ = *message;
        scheduler_arrival_s_ = get_clock()->now().seconds();
        scheduler_received_ = true;
      });
    obstacle_sub_ = create_subscription<uf_interfaces::msg::ObstacleSafetyState>(
      "/safety/raw_obstacle_state", 10,
      [this](uf_interfaces::msg::ObstacleSafetyState::ConstSharedPtr message) {
        obstacle_ = *message;
        obstacle_arrival_s_ = get_clock()->now().seconds();
        obstacle_received_ = true;
      });
    result_sub_ = create_subscription<uf_interfaces::msg::RelocalizationResult>(
      "/relocalization/result", 10,
      [this](uf_interfaces::msg::RelocalizationResult::ConstSharedPtr message) {
        result_ = *message;
        result_arrival_s_ = get_clock()->now().seconds();
        result_received_ = true;
      });
    epoch_sub_ = create_subscription<uf_interfaces::msg::FusionEpoch>(
      "/fusion/unified/epoch", rclcpp::QoS(1).reliable().transient_local(),
      [this](uf_interfaces::msg::FusionEpoch::ConstSharedPtr message) {
        epoch_ = *message;
        epoch_arrival_s_ = get_clock()->now().seconds();
        epoch_received_ = true;
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
  }

private:
  void on_request(const bool active)
  {
    if (active && !request_active_) {
      anchor_valid_ = false;
      passive_attempts_ = 0U;
      yaw_views_completed_ = 0U;
      safe_motion_steps_completed_ = 0U;
      target_active_ = false;
      target_verified_ = false;
      result_received_ = false;
      request_started_s_ = get_clock()->now().seconds();
      request_cancelled_ = false;
      last_passive_attempt_s_ = get_clock()->now().seconds();
    } else if (!active && request_active_) {
      const auto state = flight_core_->decision().state;
      request_cancelled_ = state == ActiveFlightState::HOLD ||
        state == ActiveFlightState::ACTIVE_RELOCALIZATION;
    }
    request_active_ = active;
  }

  bool pose_healthy(const double now) const
  {
    return pose_received_ && finite_pose(pose_.pose) && now >= pose_arrival_s_ &&
      now - pose_arrival_s_ <= pose_timeout_s_;
  }

  bool odom_healthy(const double now) const
  {
    return odom_received_ && finite_pose(odom_.pose.pose) && now >= odom_arrival_s_ &&
      now - odom_arrival_s_ <= pose_timeout_s_;
  }

  bool capability(const std::string & name) const
  {
    for (std::size_t index = 0; index < scheduler_.capability_names.size() &&
      index < scheduler_.capability_observable.size(); ++index)
    {
      if (scheduler_.capability_names[index] == name) {
        return scheduler_.capability_observable[index];
      }
    }
    return false;
  }

  bool scheduler_fresh(const double now) const
  {
    return scheduler_received_ && now >= scheduler_arrival_s_ &&
      now - scheduler_arrival_s_ <= scheduler_timeout_s_;
  }

  bool stabilization_healthy(const double now) const
  {
    return scheduler_fresh(now) && capability("propagation") &&
      capability("vertical_position") && capability("yaw_tracking");
  }

  bool recovery_healthy(const double now) const
  {
    return scheduler_fresh(now) &&
      (scheduler_.health_state == "NORMAL" || scheduler_.health_state == "RECOVERED") &&
      capability("propagation") && capability("horizontal_motion") &&
      capability("vertical_position") && capability("yaw_tracking") &&
      std::isfinite(scheduler_.estimator_support) && scheduler_.estimator_support >= 0.15F;
  }

  bool obstacle_fresh(const double now) const
  {
    return obstacle_received_ && obstacle_.raw_sensor_healthy &&
      now >= obstacle_arrival_s_ && now - obstacle_arrival_s_ <= obstacle_timeout_s_;
  }

  bool obstacle_allows_motion(const double now) const
  {
    if (!obstacle_fresh(now) || obstacle_arrival_s_ <= target_candidate_s_) {return false;}
    return obstacle_.state == uf_interfaces::msg::ObstacleSafetyState::CLEAR ||
      obstacle_.state == uf_interfaces::msg::ObstacleSafetyState::CAUTION;
  }

  geometry_msgs::msg::PoseStamped current_pose_message(const rclcpp::Time & now) const
  {
    auto message = pose_;
    message.header.stamp = now;
    message.header.frame_id = "map";
    return message;
  }

  void clear_candidate_path(const rclcpp::Time & now)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now;
    path.header.frame_id = "map";
    candidate_path_pub_->publish(path);
  }

  void set_target(
    const rclcpp::Time & now, const Eigen::Vector3d & position, const double yaw,
    const std::string & action)
  {
    target_ = current_pose_message(now);
    target_.pose.position.x = position.x();
    target_.pose.position.y = position.y();
    target_.pose.position.z = position.z();
    target_.pose.orientation.x = 0.0;
    target_.pose.orientation.y = 0.0;
    target_.pose.orientation.z = std::sin(0.5 * yaw);
    target_.pose.orientation.w = std::cos(0.5 * yaw);
    target_action_ = action;
    target_active_ = true;
    target_verified_ = false;
    target_settled_since_s_ = -1.0;
    target_candidate_s_ = now.seconds();
    publish_candidate_path(now);
  }

  void publish_candidate_path(const rclcpp::Time & now)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now;
    path.header.frame_id = "map";
    auto start = current_pose_message(now);
    auto goal = target_;
    goal.header = path.header;
    path.poses = {start, goal};
    candidate_path_pub_->publish(path);
  }

  bool target_reached(const double now)
  {
    if (!target_active_ || !pose_healthy(now)) {return false;}
    const Eigen::Vector3d current{pose_.pose.position.x, pose_.pose.position.y,
      pose_.pose.position.z};
    const Eigen::Vector3d target{target_.pose.position.x, target_.pose.position.y,
      target_.pose.position.z};
    const double yaw_error = std::abs(wrap(yaw_of(target_.pose.orientation) -
      yaw_of(pose_.pose.orientation)));
    const bool reached = (current - target).norm() <= target_position_tolerance_m_ &&
      yaw_error <= target_yaw_tolerance_rad_;
    if (!reached) {
      target_settled_since_s_ = -1.0;
      return false;
    }
    if (target_settled_since_s_ < 0.0) {target_settled_since_s_ = now;}
    return now - target_settled_since_s_ >= target_settle_s_;
  }

  ActiveRelocalizationAction prepare_action(const rclcpp::Time & now)
  {
    ActiveRelocalizationEvidence evidence;
    evidence.request_active = request_active_;
    evidence.attitude_healthy = scheduler_fresh(now.seconds()) &&
      capability("propagation") && capability("yaw_tracking");
    evidence.altitude_healthy = scheduler_fresh(now.seconds()) &&
      capability("vertical_position");
    evidence.local_odometry_healthy = odom_healthy(now.seconds());
    evidence.obstacle_map_fresh = obstacle_fresh(now.seconds());
    evidence.passive_attempts = passive_attempts_;
    evidence.yaw_scan_views_completed = yaw_views_completed_;
    const auto policy = policy_->decide(evidence);
    policy_reason_ = policy.reason;

    if (policy.action == ActiveRelocalizationAction::YAW_SCAN && !target_active_) {
      const Eigen::Vector3d anchor{anchor_.pose.position.x, anchor_.pose.position.y,
        anchor_.pose.position.z};
      const double target_yaw = wrap(anchor_yaw_ +
        yaw_step_rad_ * static_cast<double>(yaw_views_completed_ + 1U));
      set_target(now, anchor, target_yaw, "YAW_SCAN");
    } else if (policy.action == ActiveRelocalizationAction::EGO_SAFE_MOTION &&
      safe_motion_enabled_ && !target_active_ && safe_motion_steps_completed_ < 4U)
    {
      static constexpr std::array<std::array<double, 2>, 4> offsets{{
        {{1.0, 0.0}}, {{0.0, 1.0}}, {{-1.0, 0.0}}, {{0.0, 0.0}}}};
      const auto & offset = offsets[safe_motion_steps_completed_];
      const double c = std::cos(anchor_yaw_);
      const double s = std::sin(anchor_yaw_);
      Eigen::Vector3d target{anchor_.pose.position.x, anchor_.pose.position.y,
        anchor_.pose.position.z};
      target.x() += safe_motion_radius_m_ * (c * offset[0] - s * offset[1]);
      target.y() += safe_motion_radius_m_ * (s * offset[0] + c * offset[1]);
      set_target(now, target, anchor_yaw_, "EGO_SAFE_MOTION");
    }
    if (target_active_) {
      publish_candidate_path(now);
      if (obstacle_allows_motion(now.seconds())) {target_verified_ = true;}
    }
    return policy.action;
  }

  void tick()
  {
    const auto now = get_clock()->now();
    const double now_s = now.seconds();
    auto before = flight_core_->decision();
    if (before.state == ActiveFlightState::HOLD &&
      now_s - last_passive_attempt_s_ >= passive_attempt_period_s_ &&
      passive_attempts_ < passive_attempt_limit_)
    {
      ++passive_attempts_;
      last_passive_attempt_s_ = now_s;
    }
    if (request_active_ && !anchor_valid_ && pose_healthy(now_s)) {
      anchor_ = current_pose_message(now);
      anchor_yaw_ = yaw_of(anchor_.pose.orientation);
      anchor_valid_ = true;
    }

    ActiveRelocalizationAction action = ActiveRelocalizationAction::HOLD_POSITION;
    if (before.state == ActiveFlightState::ACTIVE_RELOCALIZATION) {
      action = prepare_action(now);
    }
    ActiveFlightEvent event;
    event.now_s = now_s;
    event.request_active = request_active_;
    event.pose_healthy = pose_healthy(now_s);
    event.stabilization_healthy = stabilization_healthy(now_s);
    event.action_available = target_active_ || action == ActiveRelocalizationAction::PASSIVE_SEARCH;
    event.action_safe = target_active_ && target_verified_ && obstacle_allows_motion(now_s);
    event.relocalization_failure = request_cancelled_ ||
      (result_received_ && result_.state == uf_interfaces::msg::RelocalizationResult::FAILED);
    event.relocalization_success = result_received_ && result_.accepted &&
      result_.state == uf_interfaces::msg::RelocalizationResult::SUCCESS;
    event.result_transaction_id = result_.transaction_id;
    event.result_candidate_id = result_.candidate_id;
    // FusionEpoch is transient-local. A retained epoch from before this
    // request must never release the recovery gate.
    event.epoch_applied = epoch_received_ && epoch_.applied &&
      epoch_arrival_s_ >= request_started_s_ && epoch_arrival_s_ >= result_arrival_s_;
    event.epoch_transaction_id = epoch_.transaction_id;
    event.epoch_candidate_id = epoch_.candidate_id;
    event.recovery_healthy = recovery_healthy(now_s) && pose_healthy(now_s);
    const auto decision = flight_core_->update(event);

    if (decision.state == ActiveFlightState::ACTIVE_RELOCALIZATION &&
      target_active_ && decision.motion_authorized)
    {
      target_.header.stamp = now;
      intent_pub_->publish(target_);
      if (target_reached(now_s)) {
        if (target_action_ == "YAW_SCAN") {++yaw_views_completed_;}
        if (target_action_ == "EGO_SAFE_MOTION") {++safe_motion_steps_completed_;}
        target_active_ = false;
        target_verified_ = false;
      }
    } else if (decision.localization_hold && pose_healthy(now_s)) {
      auto hold = current_pose_message(now);
      intent_pub_->publish(hold);
    }
    if (decision.state != ActiveFlightState::ACTIVE_RELOCALIZATION) {
      target_active_ = false;
      target_verified_ = false;
      clear_candidate_path(now);
    }

    publish_status(now, decision, action);
  }

  void publish_status(
    const rclcpp::Time & now, const ActiveFlightDecision & decision,
    const ActiveRelocalizationAction action)
  {
    uf_interfaces::msg::ActiveRelocalizationStatus status;
    status.header.stamp = now;
    status.header.frame_id = "map";
    status.state = static_cast<std::uint8_t>(decision.state);
    status.action = to_string(action);
    status.request_active = request_active_;
    status.motion_authorized = decision.motion_authorized;
    status.epoch_committed = decision.epoch_committed;
    status.transaction_id = decision.transaction_id;
    status.candidate_id = decision.candidate_id;
    status.yaw_views_completed = static_cast<std::uint32_t>(yaw_views_completed_);
    status.safe_motion_steps_completed = static_cast<std::uint32_t>(safe_motion_steps_completed_);
    status.failure_count = decision.failure_count;
    status.elapsed_s = static_cast<float>(flight_core_->state_elapsed_s(now.seconds()));
    status.reason = decision.reason + ":" + policy_reason_;
    status_pub_->publish(status);
  }

  std::unique_ptr<ActiveRelocalizationFlightCore> flight_core_;
  std::unique_ptr<ActiveRelocalizationPolicy> policy_;
  std::size_t passive_attempt_limit_{3U};
  bool safe_motion_enabled_{true};
  double pose_timeout_s_{0.25};
  double scheduler_timeout_s_{0.50};
  double obstacle_timeout_s_{0.25};
  double passive_attempt_period_s_{0.25};
  double yaw_step_rad_{M_PI_2};
  double target_position_tolerance_m_{0.12};
  double target_yaw_tolerance_rad_{8.0 * M_PI / 180.0};
  double target_settle_s_{0.15};
  double safe_motion_radius_m_{0.35};
  double pose_arrival_s_{0.0};
  double odom_arrival_s_{0.0};
  double scheduler_arrival_s_{0.0};
  double obstacle_arrival_s_{0.0};
  double last_passive_attempt_s_{0.0};
  double target_candidate_s_{0.0};
  double target_settled_since_s_{-1.0};
  double request_started_s_{0.0};
  double result_arrival_s_{0.0};
  double epoch_arrival_s_{0.0};
  bool request_active_{false};
  bool request_cancelled_{false};
  bool pose_received_{false};
  bool odom_received_{false};
  bool scheduler_received_{false};
  bool obstacle_received_{false};
  bool result_received_{false};
  bool epoch_received_{false};
  bool anchor_valid_{false};
  bool target_active_{false};
  bool target_verified_{false};
  std::size_t passive_attempts_{0U};
  std::size_t yaw_views_completed_{0U};
  std::size_t safe_motion_steps_completed_{0U};
  double anchor_yaw_{0.0};
  std::string target_action_{"HOLD_POSITION"};
  std::string policy_reason_{"request_inactive"};
  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::PoseStamped anchor_;
  geometry_msgs::msg::PoseStamped target_;
  nav_msgs::msg::Odometry odom_;
  uf_interfaces::msg::SchedulerState scheduler_;
  uf_interfaces::msg::ObstacleSafetyState obstacle_;
  uf_interfaces::msg::RelocalizationResult result_;
  uf_interfaces::msg::FusionEpoch epoch_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr intent_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr candidate_path_pub_;
  rclcpp::Publisher<uf_interfaces::msg::ActiveRelocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr request_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<uf_interfaces::msg::SchedulerState>::SharedPtr scheduler_sub_;
  rclcpp::Subscription<uf_interfaces::msg::ObstacleSafetyState>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<uf_interfaces::msg::RelocalizationResult>::SharedPtr result_sub_;
  rclcpp::Subscription<uf_interfaces::msg::FusionEpoch>::SharedPtr epoch_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uf_relocalization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uf_relocalization::ActiveRelocalizationController>());
  rclcpp::shutdown();
  return 0;
}
