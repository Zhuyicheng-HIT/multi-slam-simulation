#pragma once

#include <cstddef>
#include <string>

namespace uf_relocalization
{

struct AutomaticLoopClosureConfig
{
  bool enabled{true};
  std::size_t minimum_database_keyframes{6U};
  double search_cooldown_s{15.0};
  double minimum_keyframe_age_s{20.0};
  double maximum_candidate_distance_m{3.0};
  double maximum_correction_translation_m{0.75};
  double maximum_correction_rotation_rad{0.25};
};

struct AutomaticLoopClosureEvidence
{
  bool request_active{false};
  bool manual_request_asserted{false};
  bool scheduler_healthy{false};
  bool lidar_enabled{false};
  std::size_t database_keyframes{0U};
  std::size_t nearby_historical_keyframes{0U};
  double query_stamp_s{0.0};
  double last_search_started_s{-1.0};
};

struct AutomaticLoopClosureDecision
{
  bool start_search{false};
  std::string reason{"disabled"};
};

AutomaticLoopClosureDecision evaluate_automatic_loop_closure(
  const AutomaticLoopClosureConfig & config,
  const AutomaticLoopClosureEvidence & evidence);

bool automatic_loop_candidate_is_historical(
  const AutomaticLoopClosureConfig & config,
  double query_stamp_s,
  double keyframe_stamp_s);

bool automatic_loop_candidate_is_spatially_near(
  const AutomaticLoopClosureConfig & config,
  double candidate_distance_m);

bool automatic_loop_correction_is_safe(
  const AutomaticLoopClosureConfig & config,
  double correction_translation_m,
  double correction_rotation_rad);

}  // namespace uf_relocalization
