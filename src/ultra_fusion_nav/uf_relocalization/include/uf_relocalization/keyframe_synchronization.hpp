#pragma once

#include <optional>
#include <string>

namespace uf_relocalization
{

enum class KeyframeSynchronizationState
{
  WAITING,
  MATCHED,
  EXPIRED,
};

struct TimestampEvidence
{
  bool matched{false};
  std::optional<double> latest_stamp_s;
  double tolerance_s{0.0};
};

struct KeyframeSynchronizationDecision
{
  KeyframeSynchronizationState state{KeyframeSynchronizationState::WAITING};
  std::string reason;
};

KeyframeSynchronizationDecision decide_keyframe_synchronization(
  double cloud_stamp_s,
  const TimestampEvidence & optimized_map_pose,
  const TimestampEvidence & lio_pose,
  const TimestampEvidence & body_cloud);

KeyframeSynchronizationDecision decide_keyframe_synchronization(
  double cloud_stamp_s,
  const TimestampEvidence & optimized_map_pose,
  const TimestampEvidence & lio_pose,
  const TimestampEvidence & body_cloud,
  double waiting_age_s,
  double waiting_timeout_s);

}  // namespace uf_relocalization
