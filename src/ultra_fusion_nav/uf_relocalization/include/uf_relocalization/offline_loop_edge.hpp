#pragma once

#include "uf_relocalization/registration_core.hpp"

#include <Eigen/Geometry>

#include <cstddef>

namespace uf_relocalization
{

struct OfflineLoopEdge
{
  std::size_t candidate_keyframe{0};
  std::size_t query_keyframe{0};
  double candidate_stamp_s{0.0};
  double query_stamp_s{0.0};
  double temporal_separation_s{0.0};
  double descriptor_distance{0.0};
  Eigen::Isometry3d candidate_from_query{Eigen::Isometry3d::Identity()};
  RegistrationResult registration;
};

OfflineLoopEdge make_offline_loop_edge(
  std::size_t candidate_keyframe,
  std::size_t query_keyframe,
  double candidate_stamp_s,
  double query_stamp_s,
  const Eigen::Isometry3d & map_from_candidate,
  double descriptor_distance,
  const RegistrationResult & registration);

}  // namespace uf_relocalization
