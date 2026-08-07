#include "uf_relocalization/active_relocalization_policy.hpp"

#include <stdexcept>

namespace uf_relocalization
{

ActiveRelocalizationPolicy::ActiveRelocalizationPolicy(
  const ActiveRelocalizationPolicyConfig & config)
: config_(config)
{
  if (config_.passive_attempt_limit == 0U || config_.yaw_scan_view_count == 0U) {
    throw std::invalid_argument("active relocalization attempt limits must be positive");
  }
}

ActiveRelocalizationDecision ActiveRelocalizationPolicy::decide(
  const ActiveRelocalizationEvidence & evidence) const
{
  if (!evidence.request_active) {
    return {ActiveRelocalizationAction::IDLE, "request_inactive"};
  }
  if (!evidence.attitude_healthy || !evidence.altitude_healthy) {
    return {ActiveRelocalizationAction::FAILSAFE, "stabilization_observability_lost"};
  }
  if (evidence.passive_attempts < config_.passive_attempt_limit) {
    return {ActiveRelocalizationAction::PASSIVE_SEARCH, "passive_budget_available"};
  }
  if (evidence.yaw_scan_views_completed < config_.yaw_scan_view_count) {
    return {ActiveRelocalizationAction::YAW_SCAN, "yaw_scan_incomplete"};
  }
  if (!config_.ego_motion_enabled) {
    return {ActiveRelocalizationAction::HOLD_POSITION, "ego_experiment_disabled"};
  }
  if (!evidence.local_odometry_healthy) {
    return {ActiveRelocalizationAction::HOLD_POSITION, "local_odometry_unhealthy"};
  }
  if (!evidence.obstacle_map_fresh) {
    return {ActiveRelocalizationAction::HOLD_POSITION, "obstacle_map_stale"};
  }
  return {ActiveRelocalizationAction::EGO_SAFE_MOTION, "ego_safety_gates_passed"};
}

const char * to_string(const ActiveRelocalizationAction action)
{
  switch (action) {
    case ActiveRelocalizationAction::IDLE:
      return "IDLE";
    case ActiveRelocalizationAction::PASSIVE_SEARCH:
      return "PASSIVE_SEARCH";
    case ActiveRelocalizationAction::HOLD_POSITION:
      return "HOLD_POSITION";
    case ActiveRelocalizationAction::YAW_SCAN:
      return "YAW_SCAN";
    case ActiveRelocalizationAction::EGO_SAFE_MOTION:
      return "EGO_SAFE_MOTION";
    case ActiveRelocalizationAction::FAILSAFE:
      return "FAILSAFE";
  }
  return "UNKNOWN";
}

}  // namespace uf_relocalization
