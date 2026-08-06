#include "uf_relocalization/multi_frame_consistency_gate.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uf_relocalization
{
namespace
{

double rotation_angle(const Eigen::Matrix3d & rotation)
{
  return Eigen::AngleAxisd(rotation).angle();
}

bool finite_isometry(const Eigen::Isometry3d & transform)
{
  return transform.matrix().allFinite();
}

}  // namespace

MultiFrameConsistencyGate::MultiFrameConsistencyGate(
  const MultiFrameConsistencyConfig & config)
: config_(config)
{
  if (config_.required_queries == 0U ||
    !std::isfinite(config_.maximum_translation_delta_m) ||
    config_.maximum_translation_delta_m <= 0.0 ||
    !std::isfinite(config_.maximum_rotation_delta_rad) ||
    config_.maximum_rotation_delta_rad <= 0.0)
  {
    throw std::invalid_argument("invalid multi-frame consistency limits");
  }
  hypotheses_.reserve(config_.required_queries);
}

MultiFrameConsistencyUpdate MultiFrameConsistencyGate::observe(
  const std::int64_t query_token, const Eigen::Isometry3d & map_from_lio)
{
  if (!finite_isometry(map_from_lio)) {
    throw std::invalid_argument("map_from_lio must be finite");
  }
  if (has_query_token_ && query_token <= last_query_token_) {
    return MultiFrameConsistencyUpdate{
      MultiFrameConsistencyStatus::STALE_OR_DUPLICATE, hypotheses_.size(), 0.0, 0.0};
  }
  last_query_token_ = query_token;
  has_query_token_ = true;

  double maximum_translation_delta_m = 0.0;
  double maximum_rotation_delta_rad = 0.0;
  bool consistent = true;
  for (const auto & hypothesis : hypotheses_) {
    const Eigen::Isometry3d delta = hypothesis.inverse() * map_from_lio;
    maximum_translation_delta_m = std::max(
      maximum_translation_delta_m, delta.translation().norm());
    maximum_rotation_delta_rad = std::max(
      maximum_rotation_delta_rad, rotation_angle(delta.rotation()));
    if (maximum_translation_delta_m > config_.maximum_translation_delta_m ||
      maximum_rotation_delta_rad > config_.maximum_rotation_delta_rad)
    {
      consistent = false;
      break;
    }
  }

  if (!consistent) {
    hypotheses_.clear();
    hypotheses_.push_back(map_from_lio);
    return MultiFrameConsistencyUpdate{
      MultiFrameConsistencyStatus::RESTARTED, hypotheses_.size(),
      maximum_translation_delta_m, maximum_rotation_delta_rad};
  }

  hypotheses_.push_back(map_from_lio);
  const auto status = hypotheses_.size() >= config_.required_queries ?
    MultiFrameConsistencyStatus::CONFIRMED :
    MultiFrameConsistencyStatus::ACCUMULATING;
  return MultiFrameConsistencyUpdate{
    status, hypotheses_.size(), maximum_translation_delta_m, maximum_rotation_delta_rad};
}

void MultiFrameConsistencyGate::reset()
{
  hypotheses_.clear();
  last_query_token_ = 0;
  has_query_token_ = false;
}

std::size_t MultiFrameConsistencyGate::consistent_queries() const
{
  return hypotheses_.size();
}

}  // namespace uf_relocalization
