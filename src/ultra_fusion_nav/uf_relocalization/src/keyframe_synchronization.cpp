#include "uf_relocalization/keyframe_synchronization.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace uf_relocalization
{

KeyframeSynchronizationDecision decide_keyframe_synchronization(
  const double cloud_stamp_s,
  const TimestampEvidence & optimized_map_pose,
  const TimestampEvidence & lio_pose,
  const TimestampEvidence & body_cloud)
{
  return decide_keyframe_synchronization(
    cloud_stamp_s, optimized_map_pose, lio_pose, body_cloud,
    0.0, std::numeric_limits<double>::infinity());
}

KeyframeSynchronizationDecision decide_keyframe_synchronization(
  const double cloud_stamp_s,
  const TimestampEvidence & optimized_map_pose,
  const TimestampEvidence & lio_pose,
  const TimestampEvidence & body_cloud,
  const double waiting_age_s,
  const double waiting_timeout_s)
{
  if (!std::isfinite(cloud_stamp_s)) {
    throw std::invalid_argument("keyframe cloud timestamp must be finite");
  }
  if (!std::isfinite(waiting_age_s) || waiting_age_s < 0.0 ||
    std::isnan(waiting_timeout_s) || waiting_timeout_s <= 0.0)
  {
    throw std::invalid_argument("keyframe synchronization timeout is invalid");
  }

  struct NamedEvidence
  {
    const char * name;
    const TimestampEvidence * evidence;
  };
  const std::array<NamedEvidence, 3> streams{{
    {"optimized_map_pose", &optimized_map_pose},
    {"lio_pose", &lio_pose},
    {"body_cloud", &body_cloud},
  }};

  bool all_matched = true;
  std::string waiting_reason = "awaiting";
  for (const auto & stream : streams) {
    const auto & evidence = *stream.evidence;
    if (!std::isfinite(evidence.tolerance_s) || evidence.tolerance_s < 0.0) {
      throw std::invalid_argument("keyframe synchronization tolerance must be finite and nonnegative");
    }
    if (evidence.matched) {
      continue;
    }
    all_matched = false;
    if (evidence.latest_stamp_s && std::isfinite(*evidence.latest_stamp_s) &&
      *evidence.latest_stamp_s > cloud_stamp_s + evidence.tolerance_s)
    {
      return {
        KeyframeSynchronizationState::EXPIRED,
        std::string(stream.name) + "_advanced_past_tolerance"};
    }
    waiting_reason += std::string("_") + stream.name;
  }

  if (all_matched) {
    return {KeyframeSynchronizationState::MATCHED, "synchronized"};
  }
  if (waiting_age_s >= waiting_timeout_s) {
    return {KeyframeSynchronizationState::EXPIRED, "waiting_timeout"};
  }
  return {KeyframeSynchronizationState::WAITING, waiting_reason};
}

}  // namespace uf_relocalization
