#include "uf_safety_supervisor/command_arbiter_core.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <uf_interfaces/msg/flight_command_decision.hpp>
#include <uf_interfaces/msg/obstacle_safety_state.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

namespace uf_safety_supervisor
{
namespace
{

double stamp_s(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

PoseIntent intent(const geometry_msgs::msg::PoseStamped & message)
{
  PoseIntent output;
  output.received = true;
  output.stamp_s = stamp_s(message.header.stamp);
  output.position = {message.pose.position.x, message.pose.position.y, message.pose.position.z};
  const auto & q = message.pose.orientation;
  output.yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double qnorm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  output.finite = output.position.allFinite() && std::isfinite(output.yaw) &&
    std::isfinite(output.stamp_s) && output.stamp_s > 0.0 && qnorm > 1.0e-6;
  return output;
}

geometry_msgs::msg::PoseStamped pose_message(const PoseIntent & intent, const rclcpp::Time & now)
{
  geometry_msgs::msg::PoseStamped output;
  output.header.stamp = now;
  output.header.frame_id = "map";
  output.pose.position.x = intent.position.x();
  output.pose.position.y = intent.position.y();
  output.pose.position.z = intent.position.z();
  output.pose.orientation.z = std::sin(0.5 * intent.yaw);
  output.pose.orientation.w = std::cos(0.5 * intent.yaw);
  return output;
}

}  // namespace

class FlightCommandArbiter : public rclcpp::Node
{
public:
  FlightCommandArbiter() : Node("flight_command_arbiter")
  {
    CommandArbiterConfig config;
    config.intent_timeout_s = declare_parameter<double>("intent_timeout_s", config.intent_timeout_s);
    config.current_pose_timeout_s = declare_parameter<double>(
      "current_pose_timeout_s", config.current_pose_timeout_s);
    config.obstacle_timeout_s = declare_parameter<double>(
      "obstacle_timeout_s", config.obstacle_timeout_s);
    config.maximum_setpoint_jump_m = declare_parameter<double>(
      "maximum_setpoint_jump_m", config.maximum_setpoint_jump_m);
    config.caution_step_m = declare_parameter<double>("caution_step_m", config.caution_step_m);
    core_ = std::make_unique<CommandArbiterCore>(config);
    auto_mode_name_ = declare_parameter<std::string>("automatic_mode_name", "GUIDED");

    output_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
    selected_candidate_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/autonomy/selected_candidate_pose", 10);
    decision_pub_ = create_publisher<uf_interfaces::msg::FlightCommandDecision>(
      "/autonomy/command_decision", 10);
    mode_pub_ = create_publisher<std_msgs::msg::String>("/autonomy/selected_mode_intent", 10);

    current_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {input_.current_pose = intent(*msg);});
    mission_sub_ = pose_subscription("/autonomy/intent/mission/pose", input_.mission);
    planner_sub_ = pose_subscription("/autonomy/intent/planner/pose", input_.planner);
    relocalization_sub_ = pose_subscription(
      "/autonomy/intent/relocalization/pose", input_.relocalization);
    obstacle_sub_ = create_subscription<uf_interfaces::msg::ObstacleSafetyState>(
      "/safety/raw_obstacle_state", 10,
      [this](uf_interfaces::msg::ObstacleSafetyState::ConstSharedPtr msg) {
        input_.obstacle_stamp_s = stamp_s(msg->header.stamp);
        input_.obstacle_healthy = msg->raw_sensor_healthy;
        input_.obstacle_state = static_cast<ObstacleState>(msg->state);
      });
    localization_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/safety/localization_hold", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {input_.localization_hold = msg->data;});
    manual_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/safety/manual_override", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {explicit_manual_override_ = msg->data;});
    failsafe_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/safety/fcu_failsafe", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {explicit_fcu_failsafe_ = msg->data;});
    mode_request_sub_ = create_subscription<std_msgs::msg::String>(
      "/autonomy/intent/mode", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        input_.land_requested = msg->data == "LAND";
        input_.return_requested = msg->data == "RETURN";
      });
    fcu_state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      [this](mavros_msgs::msg::State::ConstSharedPtr msg) {
        fcu_received_ = true;
        fcu_connected_ = msg->connected;
        fcu_mode_ = msg->mode;
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription(
    const std::string & topic, PoseIntent & destination)
  {
    return create_subscription<geometry_msgs::msg::PoseStamped>(
      topic, 10, [&destination](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        destination = intent(*msg);
      });
  }

  void tick()
  {
    const auto now = get_clock()->now();
    input_.now_s = now.seconds();
    input_.manual_override = explicit_manual_override_ ||
      (fcu_received_ && fcu_connected_ && fcu_mode_ != auto_mode_name_);
    // Absence of an FCU heartbeat is not evidence that automatic control is
    // safe.  Release ownership until MAVROS has positively reported a live
    // connection, and continue to honour both the FCU and external failsafes.
    input_.fcu_failsafe =
      explicit_fcu_failsafe_ || !fcu_received_ || !fcu_connected_;
    const auto decision = core_->evaluate(input_);
    bool published = false;
    if (decision.publish_setpoint) {
      auto message = pose_message(decision.selected, now);
      output_pub_->publish(message);
      selected_candidate_pub_->publish(message);
      published = true;
    }
    if (decision.action == CommandAction::kLand || decision.action == CommandAction::kReturn) {
      std_msgs::msg::String mode;
      mode.data = to_string(decision.action);
      mode_pub_->publish(mode);
    }
    uf_interfaces::msg::FlightCommandDecision status;
    status.header.stamp = now;
    status.action = static_cast<std::uint8_t>(decision.action);
    status.owner = decision.owner;
    status.reason = decision.reason;
    status.fail_closed = decision.fail_closed;
    status.setpoint_published = published;
    status.selected_intent_age_s = decision.selected.received ?
      static_cast<float>(std::max(0.0, input_.now_s - decision.selected.stamp_s)) :
      std::numeric_limits<float>::infinity();
    decision_pub_->publish(status);
  }

  std::unique_ptr<CommandArbiterCore> core_;
  CommandArbiterInput input_;
  std::string auto_mode_name_{"GUIDED"};
  bool explicit_manual_override_{false};
  bool explicit_fcu_failsafe_{false};
  bool fcu_received_{false};
  bool fcu_connected_{false};
  std::string fcu_mode_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr output_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr selected_candidate_pub_;
  rclcpp::Publisher<uf_interfaces::msg::FlightCommandDecision>::SharedPtr decision_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mission_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr planner_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr relocalization_sub_;
  rclcpp::Subscription<uf_interfaces::msg::ObstacleSafetyState>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr failsafe_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_request_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr fcu_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uf_safety_supervisor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uf_safety_supervisor::FlightCommandArbiter>());
  rclcpp::shutdown();
  return 0;
}
