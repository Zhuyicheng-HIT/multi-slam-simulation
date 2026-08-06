#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace uf_relocalization
{

struct MultiFrameConsistencyConfig
{
  std::size_t required_queries{3U};
  double maximum_translation_delta_m{0.15};
  double maximum_rotation_delta_rad{0.05};
};

enum class MultiFrameConsistencyStatus
{
  ACCUMULATING,
  RESTARTED,
  CONFIRMED,
  STALE_OR_DUPLICATE
};

struct MultiFrameConsistencyUpdate
{
  MultiFrameConsistencyStatus status{MultiFrameConsistencyStatus::ACCUMULATING};
  std::size_t consistent_queries{0U};
  double maximum_translation_delta_m{0.0};
  double maximum_rotation_delta_rad{0.0};
};

class MultiFrameConsistencyGate
{
public:
  explicit MultiFrameConsistencyGate(
    const MultiFrameConsistencyConfig & config = MultiFrameConsistencyConfig{});

  MultiFrameConsistencyUpdate observe(
    std::int64_t query_token, const Eigen::Isometry3d & map_from_lio);

  void reset();

  std::size_t consistent_queries() const;

private:
  MultiFrameConsistencyConfig config_;
  std::vector<Eigen::Isometry3d> hypotheses_;
  std::int64_t last_query_token_{0};
  bool has_query_token_{false};
};

}  // namespace uf_relocalization
