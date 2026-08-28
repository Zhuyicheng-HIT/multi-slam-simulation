#include "uf_relocalization/offline_loop_edge.hpp"

#include <cmath>
#include <stdexcept>

namespace uf_relocalization
{

OfflineLoopEdge make_offline_loop_edge(
  const std::size_t candidate_keyframe,
  const std::size_t query_keyframe,
  const double candidate_stamp_s,
  const double query_stamp_s,
  const Eigen::Isometry3d & map_from_candidate,
  const double descriptor_distance,
  const RegistrationResult & registration)
{
  if (candidate_keyframe == query_keyframe || query_stamp_s <= candidate_stamp_s ||
    !std::isfinite(candidate_stamp_s) || !std::isfinite(query_stamp_s) ||
    !std::isfinite(descriptor_distance) || !map_from_candidate.matrix().allFinite() ||
    !registration.target_from_source.allFinite())
  {
    throw std::invalid_argument("invalid offline loop edge");
  }
  OfflineLoopEdge edge;
  edge.candidate_keyframe = candidate_keyframe;
  edge.query_keyframe = query_keyframe;
  edge.candidate_stamp_s = candidate_stamp_s;
  edge.query_stamp_s = query_stamp_s;
  edge.temporal_separation_s = query_stamp_s - candidate_stamp_s;
  edge.descriptor_distance = descriptor_distance;
  edge.candidate_from_query = map_from_candidate.inverse() *
    Eigen::Isometry3d(registration.target_from_source.cast<double>());
  edge.registration = registration;
  return edge;
}

}  // namespace uf_relocalization
