#include "uf_relocalization/active_relocalization_flight_core.hpp"

#include <cmath>
#include <stdexcept>

namespace uf_relocalization
{

ActiveRelocalizationFlightCore::ActiveRelocalizationFlightCore(ActiveFlightConfig config)
: config_(config)
{
  if (!(config_.initial_hold_s >= 0.0) || !(config_.active_timeout_s > 0.0) ||
    !(config_.recovery_dwell_s >= 0.0) || !(config_.resume_dwell_s >= 0.0) ||
    config_.maximum_failures == 0U)
  {
    throw std::invalid_argument("invalid active relocalization flight configuration");
  }
}

void ActiveRelocalizationFlightCore::transition(
  const ActiveFlightState state, const double now_s, const std::string & reason)
{
  state_ = state;
  state_since_s_ = now_s;
  reason_ = reason;
}

ActiveFlightDecision ActiveRelocalizationFlightCore::update(const ActiveFlightEvent & event)
{
  if (!std::isfinite(event.now_s) || event.now_s < state_since_s_) {
    ++failure_count_;
    transition(ActiveFlightState::FAILSAFE, std::isfinite(event.now_s) ? event.now_s : 0.0,
      "clock_invalid_or_regressed");
    return decision();
  }
  if (event.relocalization_failure && state_ != ActiveFlightState::NORMAL_NAVIGATION) {
    ++failure_count_;
    transition(ActiveFlightState::FAILSAFE, event.now_s, "relocalization_failed");
    return decision();
  }
  switch (state_) {
    case ActiveFlightState::NORMAL_NAVIGATION:
      if (event.request_active) {
        request_since_s_ = event.now_s;
        epoch_committed_ = false;
        transaction_id_ = 0U;
        candidate_id_ = 0U;
        recovery_healthy_since_s_ = -1.0;
        transition(
          event.pose_healthy ? ActiveFlightState::HOLD : ActiveFlightState::FAILSAFE,
          event.now_s, event.pose_healthy ? "localization_request_hold" : "pose_unavailable_at_request");
        if (!event.pose_healthy) {++failure_count_;}
      }
      break;
    case ActiveFlightState::HOLD:
      if (!event.pose_healthy || !event.stabilization_healthy) {
        ++failure_count_;
        transition(ActiveFlightState::FAILSAFE, event.now_s, "hold_stabilization_lost");
      } else if (event.now_s - state_since_s_ >= config_.initial_hold_s) {
        transition(ActiveFlightState::ACTIVE_RELOCALIZATION, event.now_s,
          "initial_hold_complete");
      }
      break;
    case ActiveFlightState::ACTIVE_RELOCALIZATION:
      if (event.relocalization_success) {
        if (event.result_transaction_id == 0U || event.result_candidate_id == 0U) {
          ++failure_count_;
          transition(ActiveFlightState::FAILSAFE, event.now_s, "success_identity_invalid");
        } else {
          transaction_id_ = event.result_transaction_id;
          candidate_id_ = event.result_candidate_id;
          transition(ActiveFlightState::RECOVERY_VALIDATION, event.now_s,
            "candidate_accepted_awaiting_epoch");
        }
      } else if (event.now_s - request_since_s_ >= config_.active_timeout_s) {
        ++failure_count_;
        transition(ActiveFlightState::FAILSAFE, event.now_s, "active_relocalization_timeout");
      } else if (!event.pose_healthy || !event.stabilization_healthy) {
        ++failure_count_;
        transition(ActiveFlightState::FAILSAFE, event.now_s, "active_stabilization_lost");
      } else if (!event.action_safe) {
        reason_ = "raw_obstacle_veto";
      } else if (!event.action_available) {
        reason_ = "active_action_unavailable";
      } else {
        reason_ = "active_action_authorized";
      }
      break;
    case ActiveFlightState::RECOVERY_VALIDATION:
      if (event.epoch_applied && event.epoch_transaction_id == transaction_id_ &&
        event.epoch_candidate_id == candidate_id_)
      {
        epoch_committed_ = true;
        reason_ = "matching_epoch_committed";
      }
      if (epoch_committed_ && event.recovery_healthy && !event.request_active) {
        if (recovery_healthy_since_s_ < 0.0) {recovery_healthy_since_s_ = event.now_s;}
        if (event.now_s - recovery_healthy_since_s_ >= config_.recovery_dwell_s) {
          transition(ActiveFlightState::RESUME, event.now_s, "recovery_gate_passed");
        }
      } else {
        recovery_healthy_since_s_ = -1.0;
        if (epoch_committed_ && event.recovery_healthy && event.request_active) {
          reason_ = "matching_epoch_awaiting_request_release";
        }
      }
      if (event.now_s - request_since_s_ >= config_.active_timeout_s) {
        ++failure_count_;
        transition(ActiveFlightState::FAILSAFE, event.now_s, "recovery_validation_timeout");
      }
      break;
    case ActiveFlightState::RESUME:
      if (event.now_s - state_since_s_ >= config_.resume_dwell_s) {
        transition(ActiveFlightState::NORMAL_NAVIGATION, event.now_s, "mission_resumed");
      }
      break;
    case ActiveFlightState::FAILSAFE:
      reason_ = failure_count_ >= config_.maximum_failures ?
        "repeated_failure_manual_intervention_required" : reason_;
      break;
  }
  return decision();
}

ActiveFlightDecision ActiveRelocalizationFlightCore::decision() const
{
  ActiveFlightDecision output;
  output.state = state_;
  output.localization_hold = state_ != ActiveFlightState::NORMAL_NAVIGATION &&
    state_ != ActiveFlightState::RESUME;
  output.motion_authorized = state_ == ActiveFlightState::ACTIVE_RELOCALIZATION &&
    reason_ == "active_action_authorized";
  output.epoch_committed = epoch_committed_;
  output.transaction_id = transaction_id_;
  output.candidate_id = candidate_id_;
  output.failure_count = failure_count_;
  output.reason = reason_;
  return output;
}

double ActiveRelocalizationFlightCore::state_elapsed_s(const double now_s) const
{
  return std::isfinite(now_s) && now_s >= state_since_s_ ? now_s - state_since_s_ : 0.0;
}

void ActiveRelocalizationFlightCore::reset(const double now_s)
{
  state_ = ActiveFlightState::NORMAL_NAVIGATION;
  state_since_s_ = now_s;
  request_since_s_ = now_s;
  recovery_healthy_since_s_ = -1.0;
  transaction_id_ = 0U;
  candidate_id_ = 0U;
  failure_count_ = 0U;
  epoch_committed_ = false;
  reason_ = "normal_navigation";
}

const char * to_string(const ActiveFlightState state)
{
  switch (state) {
    case ActiveFlightState::NORMAL_NAVIGATION: return "NORMAL_NAVIGATION";
    case ActiveFlightState::HOLD: return "HOLD";
    case ActiveFlightState::ACTIVE_RELOCALIZATION: return "ACTIVE_RELOCALIZATION";
    case ActiveFlightState::RECOVERY_VALIDATION: return "RECOVERY_VALIDATION";
    case ActiveFlightState::RESUME: return "RESUME";
    case ActiveFlightState::FAILSAFE: return "FAILSAFE";
  }
  return "UNKNOWN";
}

}  // namespace uf_relocalization
