#pragma once

#include "uf_dynamic_observer/conservative_free_space.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace uf_dynamic_observer
{

enum class LongTermVoxelState : std::uint8_t
{
  kUnknown = 0,
  kStaticCandidate = 1,
  kStaticConfirmed = 2,
  kDynamicCandidate = 3,
  kDynamicConfirmed = 4,
};

const char * to_string(LongTermVoxelState state);

struct LongTermMapConfig
{
  double voxel_size_m{0.25};
  double min_range_m{0.5};
  double max_range_m{35.0};
  std::uint16_t static_candidate_observations{2U};
  std::uint16_t static_confirmed_observations{6U};
  double static_confirmed_duration_s{1.0};
  std::uint8_t static_confirmed_view_bins{2U};
  double static_consistency_ratio{0.65};
  std::uint16_t candidate_free_contradictions{2U};
  std::uint16_t dynamic_candidate_free_traversals{3U};
  std::uint16_t dynamic_confirmed_free_traversals{6U};
  std::uint8_t dynamic_confirmed_view_bins{2U};
  double dynamic_confirmed_duration_s{0.4};
  std::uint16_t dynamic_label_confirmations{2U};
  std::uint16_t dynamic_recovery_static_observations{12U};
  double dynamic_recovery_duration_s{2.0};
  std::uint16_t far_static_confirmed_observations{60U};
  double far_static_confirmed_duration_s{15.0};
  std::uint8_t far_static_confirmed_view_bins{6U};
  double far_range_m{12.0};
  int endpoint_guard_voxels{1};
  int ray_stride{2};
  std::size_t max_voxels{1500000U};
  std::uint64_t stale_dynamic_after_scans{1200U};
  float semantic_dynamic_threshold{0.70F};
};

struct LongTermMapStats
{
  std::uint64_t scan_index{0U};
  std::uint64_t accepted_scans{0U};
  std::uint64_t rejected_scans{0U};
  std::uint64_t timestamp_regressions{0U};
  std::uint64_t input_points{0U};
  std::uint64_t valid_points{0U};
  std::uint64_t promoted_static_voxels{0U};
  std::uint64_t removed_ghost_voxels{0U};
  std::uint64_t semantic_messages{0U};
  std::uint64_t semantic_shadow_hits{0U};
  std::uint64_t semantic_applied_hits{0U};
  std::uint64_t capacity_rejected_voxels{0U};
  std::size_t allocated_voxels{0U};
  std::size_t unknown_voxels{0U};
  std::size_t static_candidate_voxels{0U};
  std::size_t static_confirmed_voxels{0U};
  std::size_t dynamic_candidate_voxels{0U};
  std::size_t dynamic_confirmed_voxels{0U};
  std::size_t actual_ray_voxels{0U};
  std::size_t approximate_memory_bytes{0U};
  double mean_admission_delay_s{0.0};
  double promoted_static_ratio{0.0};
  double permanent_rejection_ratio{0.0};
};

struct LongTermUpdateResult
{
  bool accepted{false};
  std::string reason;
  LongTermMapStats stats;
};

struct StaticMapPoint
{
  Point point;
  std::uint32_t support{0U};
  float confidence{0.0F};
};

class LongTermStaticMap
{
public:
  explicit LongTermStaticMap(LongTermMapConfig config = {});

  LongTermUpdateResult integrate(
    const std::vector<LabeledPoint> & observations, const Point & sensor_origin,
    double stamp_s);
  bool add_semantic_evidence(
    const Point & world_point, float dynamic_confidence, double stamp_s,
    bool shadow_only);
  std::vector<StaticMapPoint> static_confirmed_points() const;
  LongTermVoxelState state_at(const Point & world_point) const;
  LongTermMapStats stats() const;
  void reset();
  const LongTermMapConfig & config() const {return config_;}

private:
  struct Element
  {
    Point centroid;
    LongTermVoxelState state{LongTermVoxelState::kUnknown};
    std::uint32_t occupied_support{0U};
    std::uint32_t static_support{0U};
    std::uint32_t dynamic_support{0U};
    std::uint32_t free_traversals{0U};
    std::uint32_t free_since_occupied{0U};
    std::uint64_t occupied_view_mask{0U};
    std::uint64_t free_view_mask{0U};
    std::uint64_t last_seen_scan{0U};
    double first_occupied_s{0.0};
    double candidate_since_s{0.0};
    double last_occupied_s{0.0};
    double first_free_contradiction_s{0.0};
    double last_free_s{0.0};
    bool was_permanently_admitted{false};
    bool ghost_removal_counted{false};
  };

  VoxelKey key(const Point & point) const;
  Point center(const VoxelKey & key) const;
  bool valid(const Point & point, const Point & origin) const;
  std::uint64_t view_bit(const Point & origin, const Point & endpoint) const;
  void trace_ray(
    const Point & origin, const Point & endpoint,
    std::vector<VoxelKey> & traversed) const;
  void apply_free_evidence(const VoxelKey & voxel, std::uint64_t view, double stamp_s);
  void apply_occupied_evidence(
    const VoxelKey & voxel, const LabeledPoint & observation,
    std::uint64_t view, double stamp_s, bool far_range);
  void transition(Element & element, LongTermVoxelState next, double stamp_s);
  void prune_if_needed();
  void refresh_counts(LongTermMapStats & output) const;

  LongTermMapConfig config_;
  std::unordered_map<VoxelKey, Element, VoxelKeyHash> elements_;
  std::uint64_t scan_index_{0U};
  double last_stamp_s_{-1.0};
  std::uint64_t accepted_scans_{0U};
  std::uint64_t rejected_scans_{0U};
  std::uint64_t timestamp_regressions_{0U};
  std::uint64_t input_points_{0U};
  std::uint64_t valid_points_{0U};
  std::uint64_t promoted_static_voxels_{0U};
  std::uint64_t removed_ghost_voxels_{0U};
  std::uint64_t semantic_messages_{0U};
  std::uint64_t semantic_shadow_hits_{0U};
  std::uint64_t semantic_applied_hits_{0U};
  std::uint64_t capacity_rejected_voxels_{0U};
  std::uint64_t actual_ray_voxels_{0U};
  double admission_delay_sum_s_{0.0};
};

}  // namespace uf_dynamic_observer
