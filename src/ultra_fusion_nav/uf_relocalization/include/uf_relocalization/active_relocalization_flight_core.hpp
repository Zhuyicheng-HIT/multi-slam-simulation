#pragma once

#include <cstdint>
#include <string>

namespace uf_relocalization
{

enum class ActiveFlightState : std::uint8_t
{
  NORMAL_NAVIGATION = 0,
  HOLD = 1,
  ACTIVE_RELOCALIZATION = 2,
  RECOVERY_VALIDATION = 3,
  RESUME = 4,
  FAILSAFE = 5
};

struct ActiveFlightConfig
{
  double initial_hold_s{1.0};
  double active_timeout_s{20.0};
  double recovery_dwell_s{0.75};
  double resume_dwell_s{0.25};
  std::uint32_t maximum_failures{2U};
};

struct ActiveFlightEvent
{
  double now_s{0.0};
  bool request_active{false};
  bool pose_healthy{false};
  bool stabilization_healthy{false};
  bool action_safe{false};
  bool action_available{false};
  bool relocalization_success{false};
  bool relocalization_failure{false};
  std::uint64_t result_transaction_id{0U};
  std::uint32_t result_candidate_id{0U};
  bool epoch_applied{false};
  std::uint64_t epoch_transaction_id{0U};
  std::uint32_t epoch_candidate_id{0U};
  bool recovery_healthy{false};
};

struct ActiveFlightDecision
{
  ActiveFlightState state{ActiveFlightState::NORMAL_NAVIGATION};
  bool localization_hold{false};
  bool motion_authorized{false};
  bool epoch_committed{false};
  std::uint64_t transaction_id{0U};
  std::uint32_t candidate_id{0U};
  std::uint32_t failure_count{0U};
  std::string reason{"normal_navigation"};
};

class ActiveRelocalizationFlightCore
{
public:
  explicit ActiveRelocalizationFlightCore(ActiveFlightConfig config = {});
  ActiveFlightDecision update(const ActiveFlightEvent & event);
  ActiveFlightDecision decision() const;
  double state_elapsed_s(double now_s) const;
  void reset(double now_s = 0.0);

private:
  void transition(ActiveFlightState state, double now_s, const std::string & reason);
  ActiveFlightConfig config_;
  ActiveFlightState state_{ActiveFlightState::NORMAL_NAVIGATION};
  double state_since_s_{0.0};
  double request_since_s_{0.0};
  double recovery_healthy_since_s_{-1.0};
  std::uint64_t transaction_id_{0U};
  std::uint32_t candidate_id_{0U};
  std::uint32_t failure_count_{0U};
  bool epoch_committed_{false};
  std::string reason_{"normal_navigation"};
};

const char * to_string(ActiveFlightState state);

}  // namespace uf_relocalization
