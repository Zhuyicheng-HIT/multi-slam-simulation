#pragma once

#include <cstddef>
#include <string>

namespace uf_relocalization
{

enum class ActiveRelocalizationAction
{
  IDLE,
  PASSIVE_SEARCH,
  HOLD_POSITION,
  YAW_SCAN,
  EGO_SAFE_MOTION,
  FAILSAFE
};

struct ActiveRelocalizationPolicyConfig
{
  std::size_t passive_attempt_limit{3U};
  std::size_t yaw_scan_view_count{4U};
  bool ego_motion_enabled{false};
};

struct ActiveRelocalizationEvidence
{
  bool request_active{false};
  bool attitude_healthy{false};
  bool altitude_healthy{false};
  bool local_odometry_healthy{false};
  bool obstacle_map_fresh{false};
  std::size_t passive_attempts{0U};
  std::size_t yaw_scan_views_completed{0U};
};

struct ActiveRelocalizationDecision
{
  ActiveRelocalizationAction action{ActiveRelocalizationAction::IDLE};
  std::string reason{"request_inactive"};
};

class ActiveRelocalizationPolicy
{
public:
  explicit ActiveRelocalizationPolicy(
    const ActiveRelocalizationPolicyConfig & config = ActiveRelocalizationPolicyConfig{});

  ActiveRelocalizationDecision decide(
    const ActiveRelocalizationEvidence & evidence) const;

private:
  ActiveRelocalizationPolicyConfig config_;
};

const char * to_string(ActiveRelocalizationAction action);

}  // namespace uf_relocalization
