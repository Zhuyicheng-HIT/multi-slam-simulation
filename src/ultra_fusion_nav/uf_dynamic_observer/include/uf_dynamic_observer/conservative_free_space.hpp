#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace uf_dynamic_observer
{

struct Point
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  float intensity{0.0F};
};

enum class PointLabel : std::uint8_t
{
  kStatic = 0,
  kDynamic = 1,
  kUnknown = 2,
};

struct LabeledPoint
{
  Point point;
  PointLabel label{PointLabel::kUnknown};
  float dynamic_score{0.5F};
};

struct FilterConfig
{
  double voxel_size_m{0.25};
  double min_range_m{0.5};
  double max_range_m{35.0};
  std::uint16_t free_confirmations{4};
  std::uint16_t static_confirmations{2};
  std::uint16_t occupied_recovery{20};
  int endpoint_guard_voxels{1};
  int dynamic_growth_voxels{2};
  int ray_stride{4};
  std::size_t max_voxels{1500000U};
  std::uint64_t stale_after_scans{600U};
};

struct VisibilityFilterConfig : public FilterConfig
{
  // A state transition is driven only by rays that were actually measured.
  // No angular inpainting is allowed for the non-repetitive MID360 pattern.
  std::uint16_t dynamic_confirmations{1U};
  std::uint16_t dynamic_hold_scans{12U};
  std::uint16_t vacated_hold_scans{8U};
  std::uint16_t static_vacate_confirmations{1U};
  int dynamic_track_radius_voxels{1};
  int vacated_surface_radius_voxels{1};
  int static_support_radius_voxels{1};
  std::size_t min_static_neighbor_voxels{0U};
  double far_range_m{20.0};
  std::uint16_t far_static_confirmations{4U};
};

struct FilterStats
{
  std::uint64_t scan_index{0U};
  std::size_t input_points{0U};
  std::size_t valid_points{0U};
  std::size_t static_points{0U};
  std::size_t dynamic_points{0U};
  std::size_t unknown_points{0U};
  std::size_t dynamic_seed_voxels{0U};
  std::size_t observed_ray_voxels{0U};
  std::size_t vacated_surface_voxels{0U};
  std::size_t persistent_dynamic_voxels{0U};
  std::size_t free_voxels{0U};
  std::size_t allocated_voxels{0U};
  std::size_t approximate_memory_bytes{0U};
};

struct FilterResult
{
  std::vector<LabeledPoint> points;
  FilterStats stats;
};

struct VoxelKey
{
  std::int32_t x{0};
  std::int32_t y{0};
  std::int32_t z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const noexcept;
};

class ConservativeFreeSpaceObserver
{
public:
  explicit ConservativeFreeSpaceObserver(FilterConfig config = {});

  FilterResult process(const std::vector<Point> & world_points, const Point & sensor_origin);
  void reset();
  const FilterConfig & config() const {return config_;}

private:
  struct Evidence
  {
    std::uint16_t free_streak{0U};
    std::uint16_t occupied_streak{0U};
    std::uint16_t occupied_observations{0U};
    bool confirmed_free{false};
    std::uint64_t last_seen_scan{0U};
  };

  VoxelKey key(const Point & point) const;
  bool is_valid(const Point & point, const Point & origin) const;
  bool has_neighbor(
    const VoxelKey & key, const std::unordered_set<VoxelKey, VoxelKeyHash> & candidates,
    int radius) const;
  void trace_ray(
    const Point & origin, const Point & endpoint,
    std::unordered_set<VoxelKey, VoxelKeyHash> & traversed) const;
  void prune_if_needed();

  FilterConfig config_;
  std::unordered_map<VoxelKey, Evidence, VoxelKeyHash> evidence_;
  std::uint64_t scan_index_{0U};
};

// A clean-room FreeDOM/DUFOMap-style observer.  It deliberately keeps the v1
// ConservativeFreeSpaceObserver unchanged so benchmark reports can compare the
// original implementation and this stateful visibility model on identical
// inputs.
class VisibilityAwareDynamicObserver
{
public:
  explicit VisibilityAwareDynamicObserver(VisibilityFilterConfig config = {});

  FilterResult process(const std::vector<Point> & world_points, const Point & sensor_origin);
  void reset();
  const VisibilityFilterConfig & config() const {return config_;}

private:
  struct Evidence
  {
    std::uint16_t free_streak{0U};
    std::uint16_t occupied_streak{0U};
    std::uint16_t occupied_observations{0U};
    std::uint16_t dynamic_streak{0U};
    bool confirmed_free{false};
    bool confirmed_static{false};
    std::uint64_t dynamic_until_scan{0U};
    std::uint64_t vacated_until_scan{0U};
    std::uint64_t last_free_scan{0U};
    std::uint64_t last_occupied_scan{0U};
    std::uint64_t last_seen_scan{0U};
  };

  VoxelKey key(const Point & point) const;
  bool is_valid(const Point & point, const Point & origin) const;
  bool has_neighbor(
    const VoxelKey & key, const std::unordered_set<VoxelKey, VoxelKeyHash> & candidates,
    int radius) const;
  std::size_t neighbor_count(
    const VoxelKey & key, const std::unordered_set<VoxelKey, VoxelKeyHash> & candidates,
    int radius) const;
  bool has_recent_dynamic_neighbor(const VoxelKey & key, int radius) const;
  bool has_recent_vacated_neighbor(const VoxelKey & key, int radius) const;
  void trace_ray(
    const Point & origin, const Point & endpoint,
    std::unordered_set<VoxelKey, VoxelKeyHash> & traversed) const;
  void prune_if_needed();

  VisibilityFilterConfig config_;
  std::unordered_map<VoxelKey, Evidence, VoxelKeyHash> evidence_;
  std::uint64_t scan_index_{0U};
};

class TemporalVoxelBaseline
{
public:
  TemporalVoxelBaseline(
    double voxel_size_m = 0.25, std::size_t window_frames = 5U,
    std::size_t min_static_support = 2U, int neighbor_radius = 1);

  FilterResult process(const std::vector<Point> & world_points, const Point & sensor_origin);
  void reset();

private:
  VoxelKey key(const Point & point) const;
  bool supported(
    const VoxelKey & key, const std::unordered_set<VoxelKey, VoxelKeyHash> & frame) const;

  double voxel_size_m_;
  std::size_t window_frames_;
  std::size_t min_static_support_;
  int neighbor_radius_;
  std::deque<std::unordered_set<VoxelKey, VoxelKeyHash>> history_;
  std::uint64_t scan_index_{0U};
};

}  // namespace uf_dynamic_observer
