#pragma once

#include <cstdint>
#include <tuple>

namespace mid360_reliable_mapper
{

struct VoxelRetentionEvidence
{
  int32_t x{0};
  int32_t y{0};
  int32_t z{0};
  uint32_t support_count{0};
  uint64_t last_observed_frame{0};
  uint16_t occupied_neighbors{0};
  bool has_lower_support{false};
};

class VoxelRetentionPolicy
{
public:
  VoxelRetentionPolicy(uint32_t stable_support_hits, uint16_t isolated_neighbor_threshold)
  : stable_support_hits_(stable_support_hits),
    isolated_neighbor_threshold_(isolated_neighbor_threshold)
  {}

  bool should_evict_before(
    const VoxelRetentionEvidence & lhs,
    const VoxelRetentionEvidence & rhs,
    uint64_t current_frame) const
  {
    return rank(lhs, current_frame) < rank(rhs, current_frame);
  }

private:
  using Rank = std::tuple<bool, uint32_t, uint64_t, bool, bool, int32_t, int32_t, int32_t>;

  Rank rank(const VoxelRetentionEvidence & value, uint64_t current_frame) const
  {
    const bool stable = value.support_count >= stable_support_hits_;
    const bool connected = value.occupied_neighbors > isolated_neighbor_threshold_;
    const uint64_t age = current_frame >= value.last_observed_frame ?
      current_frame - value.last_observed_frame : 0U;

    // Lower tuples are evicted first. This protects repeatedly observed,
    // spatially supported walls and ground without assuming a ground height.
    return std::make_tuple(
      stable,
      value.support_count,
      static_cast<uint64_t>(~age),
      connected,
      value.has_lower_support,
      value.x,
      value.y,
      value.z);
  }

  uint32_t stable_support_hits_{4U};
  uint16_t isolated_neighbor_threshold_{1U};
};

}  // namespace mid360_reliable_mapper
