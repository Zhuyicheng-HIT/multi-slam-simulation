#include "uf_safety_supervisor/command_arbiter_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uf_safety_supervisor
{

CommandArbiterCore::CommandArbiterCore(CommandArbiterConfig config) : config_(config)
{
  if (!(config_.intent_timeout_s > 0.0) || !(config_.current_pose_timeout_s > 0.0) ||
    !(config_.obstacle_timeout_s > 0.0) || !(config_.maximum_setpoint_jump_m > 0.0))
  {
    throw std::invalid_argument("invalid command arbiter timeout or jump configuration");
  }
}

bool CommandArbiterCore::fresh(const PoseIntent & intent, const double now_s) const
{
  return intent.received && intent.finite && std::isfinite(intent.stamp_s) &&
         now_s >= intent.stamp_s && now_s - intent.stamp_s <= config_.intent_timeout_s;
}

CommandDecision CommandArbiterCore::hold(
  const CommandArbiterInput & input, const std::string & owner,
  const std::string & reason, const bool fail_closed) const
{
  CommandDecision decision;
  decision.action = CommandAction::kHold;
  decision.owner = owner;
  decision.reason = reason;
  decision.fail_closed = fail_closed;
  const bool pose_fresh = input.current_pose.received && input.current_pose.finite &&
    input.now_s >= input.current_pose.stamp_s &&
    input.now_s - input.current_pose.stamp_s <= config_.current_pose_timeout_s;
  decision.publish_setpoint = pose_fresh;
  decision.selected = input.current_pose;
  return decision;
}

CommandDecision CommandArbiterCore::forward(
  const CommandArbiterInput & input, const PoseIntent & intent, const std::string & owner) const
{
  if (!fresh(intent, input.now_s)) {
    return hold(input, "fail_closed", owner + "_intent_stale_or_invalid", true);
  }
  if (!input.current_pose.received || !input.current_pose.finite ||
    input.now_s < input.current_pose.stamp_s ||
    input.now_s - input.current_pose.stamp_s > config_.current_pose_timeout_s)
  {
    return hold(input, "fail_closed", "current_pose_stale_or_invalid", true);
  }
  const Eigen::Vector3d delta = intent.position - input.current_pose.position;
  if (!delta.allFinite() || delta.norm() > config_.maximum_setpoint_jump_m) {
    return hold(input, "fail_closed", owner + "_setpoint_jump", true);
  }
  CommandDecision decision;
  decision.action = CommandAction::kForward;
  decision.owner = owner;
  decision.reason = input.obstacle_state == ObstacleState::kCaution ?
    "caution_limited" : "selected_by_priority";
  decision.fail_closed = false;
  decision.publish_setpoint = true;
  decision.selected = intent;
  if (input.obstacle_state == ObstacleState::kCaution && delta.norm() > config_.caution_step_m) {
    decision.selected.position = input.current_pose.position +
      delta.normalized() * config_.caution_step_m;
  }
  return decision;
}

CommandDecision CommandArbiterCore::evaluate(const CommandArbiterInput & input) const
{
  if (!std::isfinite(input.now_s)) {
    CommandDecision decision;
    decision.action = CommandAction::kRelease;
    decision.owner = "fcu";
    decision.reason = "clock_nonfinite";
    decision.fail_closed = true;
    return decision;
  }
  if (input.manual_override || input.fcu_failsafe) {
    CommandDecision decision;
    decision.action = CommandAction::kRelease;
    decision.owner = input.manual_override ? "manual" : "fcu_failsafe";
    decision.reason = "automatic_setpoint_released";
    decision.fail_closed = false;
    return decision;
  }
  if (!input.obstacle_healthy || input.now_s < input.obstacle_stamp_s ||
    input.now_s - input.obstacle_stamp_s > config_.obstacle_timeout_s)
  {
    return hold(input, "obstacle_safety", "raw_obstacle_state_stale_or_unhealthy", true);
  }
  if (input.obstacle_state == ObstacleState::kBrake) {
    return hold(input, "obstacle_safety", "brake", false);
  }
  if (input.obstacle_state == ObstacleState::kHover) {
    return hold(input, "obstacle_safety", "hover_required", true);
  }
  if (input.land_requested) {
    auto decision = hold(input, "land_return", "land_requested", false);
    decision.action = CommandAction::kLand;
    return decision;
  }
  if (input.return_requested) {
    auto decision = hold(input, "land_return", "return_requested", false);
    decision.action = CommandAction::kReturn;
    return decision;
  }
  if ((input.localization_hold || input.active_relocalization_hold) &&
    !input.active_relocalization_authorized)
  {
    return hold(input,
      input.active_relocalization_hold ? "active_relocalization" : "localization_safety",
      input.active_relocalization_hold ? "active_relocalization_hold" : "localization_hold",
      false);
  }
  if (input.active_relocalization_authorized) {
    if (fresh(input.relocalization, input.now_s)) {
      return forward(input, input.relocalization, "active_relocalization");
    }
    return hold(input, "fail_closed", "authorized_relocalization_intent_missing", true);
  }
  if (fresh(input.planner, input.now_s)) {
    return forward(input, input.planner, "local_planner");
  }
  if (input.planner.received && !fresh(input.planner, input.now_s)) {
    return hold(input, "fail_closed", "planner_intent_stale_or_invalid", true);
  }
  if (fresh(input.mission, input.now_s)) {
    return forward(input, input.mission, "mission");
  }
  return hold(input, "fail_closed", "no_fresh_automatic_intent", true);
}

const char * to_string(const CommandAction action)
{
  switch (action) {
    case CommandAction::kRelease: return "RELEASE";
    case CommandAction::kForward: return "FORWARD";
    case CommandAction::kHold: return "HOLD";
    case CommandAction::kLand: return "LAND";
    case CommandAction::kReturn: return "RETURN";
  }
  return "UNKNOWN";
}

}  // namespace uf_safety_supervisor
