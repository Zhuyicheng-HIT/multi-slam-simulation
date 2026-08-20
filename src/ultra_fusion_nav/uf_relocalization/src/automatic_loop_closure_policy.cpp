#include "uf_relocalization/automatic_loop_closure_policy.hpp"

#include <cmath>
#include <stdexcept>

namespace uf_relocalization
{

namespace
{

void validate_config(const AutomaticLoopClosureConfig & config)
{
  if (config.minimum_database_keyframes == 0U ||
    !std::isfinite(config.search_cooldown_s) || config.search_cooldown_s <= 0.0 ||
    !std::isfinite(config.minimum_keyframe_age_s) || config.minimum_keyframe_age_s <= 0.0 ||
    !std::isfinite(config.maximum_candidate_distance_m) ||
    config.maximum_candidate_distance_m <= 0.0 ||
    !std::isfinite(config.maximum_correction_translation_m) ||
    config.maximum_correction_translation_m <= 0.0 ||
    !std::isfinite(config.maximum_correction_rotation_rad) ||
    config.maximum_correction_rotation_rad <= 0.0)
  {
    throw std::invalid_argument("automatic loop-closure limits must be positive");
  }
}

}  // namespace

AutomaticLoopClosureDecision evaluate_automatic_loop_closure(
  const AutomaticLoopClosureConfig & config,
  const AutomaticLoopClosureEvidence & evidence)
{
  validate_config(config);
  if (!config.enabled) {
    return {false, "disabled"};
  }
  if (evidence.manual_request_asserted) {
    return {false, "manual_request_has_priority"};
  }
  if (evidence.request_active) {
    return {false, "search_already_active"};
  }
  if (!evidence.scheduler_healthy || !evidence.lidar_enabled) {
    return {false, "localization_health_unavailable"};
  }
  if (evidence.database_keyframes < config.minimum_database_keyframes) {
    return {false, "database_not_ready"};
  }
  if (evidence.nearby_historical_keyframes == 0U) {
    return {false, "no_nearby_historical_place"};
  }
  if (!std::isfinite(evidence.query_stamp_s) || evidence.query_stamp_s <= 0.0) {
    return {false, "invalid_query_stamp"};
  }
  if (std::isfinite(evidence.last_search_started_s) &&
    evidence.last_search_started_s >= 0.0 &&
    evidence.query_stamp_s >= evidence.last_search_started_s &&
    evidence.query_stamp_s - evidence.last_search_started_s < config.search_cooldown_s)
  {
    return {false, "cooldown_active"};
  }
  return {true, "historical_place_search_due"};
}

bool automatic_loop_candidate_is_historical(
  const AutomaticLoopClosureConfig & config,
  const double query_stamp_s,
  const double keyframe_stamp_s)
{
  validate_config(config);
  if (!std::isfinite(query_stamp_s) || !std::isfinite(keyframe_stamp_s)) {
    return false;
  }
  return query_stamp_s > keyframe_stamp_s &&
         query_stamp_s - keyframe_stamp_s >= config.minimum_keyframe_age_s;
}

bool automatic_loop_correction_is_safe(
  const AutomaticLoopClosureConfig & config,
  const double correction_translation_m,
  const double correction_rotation_rad)
{
  validate_config(config);
  return relocalization_correction_is_safe(
    correction_translation_m, correction_rotation_rad,
    config.maximum_correction_translation_m,
    config.maximum_correction_rotation_rad);
}

bool relocalization_correction_is_safe(
  const double correction_translation_m,
  const double correction_rotation_rad,
  const double maximum_translation_m,
  const double maximum_rotation_rad)
{
  return std::isfinite(correction_translation_m) &&
         std::isfinite(correction_rotation_rad) &&
         correction_translation_m >= 0.0 && correction_rotation_rad >= 0.0 &&
         std::isfinite(maximum_translation_m) && maximum_translation_m > 0.0 &&
         std::isfinite(maximum_rotation_rad) && maximum_rotation_rad > 0.0 &&
         correction_translation_m <= maximum_translation_m &&
         correction_rotation_rad <= maximum_rotation_rad;
}

bool automatic_loop_candidate_is_spatially_near(
  const AutomaticLoopClosureConfig & config,
  const double candidate_distance_m)
{
  validate_config(config);
  return std::isfinite(candidate_distance_m) && candidate_distance_m >= 0.0 &&
         candidate_distance_m <= config.maximum_candidate_distance_m;
}

}  // namespace uf_relocalization
