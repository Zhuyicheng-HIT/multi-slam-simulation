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
  int dynamic_growth_voxels{1};
  int ray_stride{4};
  std::size_t max_voxels{1500000U};
  std::uint64_t stale_after_scans{600U};
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
  std::size_t free_voxels{0U};
  std::size_t allocated_voxels{0U};
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
