#include "uf_dynamic_observer/conservative_free_space.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace uf_dynamic_observer
{
namespace
{

std::uint16_t saturating_increment(std::uint16_t value)
{
  if (value == std::numeric_limits<std::uint16_t>::max()) {
    return value;
  }
  return static_cast<std::uint16_t>(value + 1U);
}

double squared_distance(const Point & lhs, const Point & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  const double dz = lhs.z - rhs.z;
  return dx * dx + dy * dy + dz * dz;
}

}  // namespace

std::size_t VoxelKeyHash::operator()(const VoxelKey & key) const noexcept
{
  const auto x = static_cast<std::uint32_t>(key.x);
  const auto y = static_cast<std::uint32_t>(key.y);
  const auto z = static_cast<std::uint32_t>(key.z);
  std::size_t seed = static_cast<std::size_t>(x) * 0x9e3779b1U;
  seed ^= static_cast<std::size_t>(y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  seed ^= static_cast<std::size_t>(z) + 0x85ebca6bU + (seed << 6U) + (seed >> 2U);
  return seed;
}

ConservativeFreeSpaceObserver::ConservativeFreeSpaceObserver(FilterConfig config)
: config_(std::move(config))
{
  if (!(config_.voxel_size_m > 0.0) || !(config_.max_range_m > config_.min_range_m)) {
    throw std::invalid_argument("invalid voxel size or range");
  }
  if (config_.free_confirmations == 0U || config_.static_confirmations == 0U ||
    config_.occupied_recovery == 0U || config_.endpoint_guard_voxels < 0 ||
    config_.dynamic_growth_voxels < 0 || config_.ray_stride < 1)
  {
    throw std::invalid_argument("invalid conservative free-space configuration");
  }
}

VoxelKey ConservativeFreeSpaceObserver::key(const Point & point) const
{
  return {
    static_cast<std::int32_t>(std::floor(point.x / config_.voxel_size_m)),
    static_cast<std::int32_t>(std::floor(point.y / config_.voxel_size_m)),
    static_cast<std::int32_t>(std::floor(point.z / config_.voxel_size_m))};
}

bool ConservativeFreeSpaceObserver::is_valid(const Point & point, const Point & origin) const
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
    return false;
  }
  const double range_squared = squared_distance(point, origin);
  return range_squared >= config_.min_range_m * config_.min_range_m &&
         range_squared <= config_.max_range_m * config_.max_range_m;
}

bool ConservativeFreeSpaceObserver::has_neighbor(
  const VoxelKey & query, const std::unordered_set<VoxelKey, VoxelKeyHash> & candidates,
  int radius) const
{
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -radius; dz <= radius; ++dz) {
        if (candidates.count({query.x + dx, query.y + dy, query.z + dz}) != 0U) {
          return true;
        }
      }
    }
  }
  return false;
}

void ConservativeFreeSpaceObserver::trace_ray(
  const Point & origin, const Point & endpoint,
  std::unordered_set<VoxelKey, VoxelKeyHash> & traversed) const
{
  const double dx = endpoint.x - origin.x;
  const double dy = endpoint.y - origin.y;
  const double dz = endpoint.z - origin.z;
  const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
  const int steps = static_cast<int>(std::ceil(length / config_.voxel_size_m));
  const int stop = std::max(1, steps - config_.endpoint_guard_voxels - 1);
  for (int index = 1; index < stop; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(steps);
    traversed.insert(key({
      origin.x + ratio * dx, origin.y + ratio * dy, origin.z + ratio * dz, 0.0F}));
  }
}

FilterResult ConservativeFreeSpaceObserver::process(
  const std::vector<Point> & world_points, const Point & sensor_origin)
{
  ++scan_index_;
  FilterResult result;
  result.stats.scan_index = scan_index_;
  result.stats.input_points = world_points.size();
  result.points.reserve(world_points.size());

  std::vector<Point> valid_points;
  valid_points.reserve(world_points.size());
  std::unordered_set<VoxelKey, VoxelKeyHash> endpoints;
  for (const auto & point : world_points) {
    if (!is_valid(point, sensor_origin)) {
      continue;
    }
    valid_points.push_back(point);
    endpoints.insert(key(point));
  }
  result.stats.valid_points = valid_points.size();

  // Classification always uses evidence from earlier scans. Updating the free-space
  // map afterwards prevents a point from explaining itself as dynamic.
  std::unordered_set<VoxelKey, VoxelKeyHash> dynamic_seeds;
  for (const auto & endpoint : endpoints) {
    const auto it = evidence_.find(endpoint);
    if (it != evidence_.end() && it->second.confirmed_free) {
      dynamic_seeds.insert(endpoint);
    }
  }

  for (const auto & point : valid_points) {
    const auto endpoint = key(point);
    LabeledPoint labeled;
    labeled.point = point;
    if (dynamic_seeds.count(endpoint) != 0U) {
      labeled.label = PointLabel::kDynamic;
      labeled.dynamic_score = 1.0F;
      ++result.stats.dynamic_points;
    } else if (config_.dynamic_growth_voxels > 0 &&
      has_neighbor(endpoint, dynamic_seeds, config_.dynamic_growth_voxels))
    {
      labeled.label = PointLabel::kDynamic;
      labeled.dynamic_score = 0.75F;
      ++result.stats.dynamic_points;
    } else {
      const auto it = evidence_.find(endpoint);
      if (it != evidence_.end() && !it->second.confirmed_free &&
        it->second.occupied_observations >= config_.static_confirmations)
      {
        labeled.label = PointLabel::kStatic;
        labeled.dynamic_score = 0.0F;
        ++result.stats.static_points;
      } else {
        labeled.label = PointLabel::kUnknown;
        labeled.dynamic_score = 0.5F;
        ++result.stats.unknown_points;
      }
    }
    result.points.push_back(labeled);
  }
  result.stats.dynamic_seed_voxels = dynamic_seeds.size();

  std::unordered_set<VoxelKey, VoxelKeyHash> guarded_endpoints;
  const int guard = config_.endpoint_guard_voxels;
  for (const auto & endpoint : endpoints) {
    for (int dx = -guard; dx <= guard; ++dx) {
      for (int dy = -guard; dy <= guard; ++dy) {
        for (int dz = -guard; dz <= guard; ++dz) {
          guarded_endpoints.insert({endpoint.x + dx, endpoint.y + dy, endpoint.z + dz});
        }
      }
    }
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> traversed;
  std::unordered_set<VoxelKey, VoxelKeyHash> ray_endpoints;
  for (std::size_t index = 0U; index < valid_points.size();
    index += static_cast<std::size_t>(config_.ray_stride))
  {
    const auto endpoint = key(valid_points[index]);
    if (ray_endpoints.insert(endpoint).second) {
      trace_ray(sensor_origin, valid_points[index], traversed);
    }
  }

  for (const auto & traversed_key : traversed) {
    if (guarded_endpoints.count(traversed_key) != 0U) {
      continue;
    }
    auto & evidence = evidence_[traversed_key];
    evidence.free_streak = saturating_increment(evidence.free_streak);
    evidence.occupied_streak = 0U;
    evidence.last_seen_scan = scan_index_;
    if (evidence.free_streak >= config_.free_confirmations) {
      evidence.confirmed_free = true;
    }
  }

  for (const auto & endpoint : endpoints) {
    auto & evidence = evidence_[endpoint];
    evidence.occupied_streak = saturating_increment(evidence.occupied_streak);
    evidence.occupied_observations = saturating_increment(evidence.occupied_observations);
    evidence.free_streak = 0U;
    evidence.last_seen_scan = scan_index_;
    if (evidence.confirmed_free && evidence.occupied_streak >= config_.occupied_recovery) {
      evidence.confirmed_free = false;
      evidence.occupied_observations = config_.static_confirmations;
    }
  }

  prune_if_needed();
  result.stats.observed_ray_voxels = traversed.size();
  result.stats.allocated_voxels = evidence_.size();
  result.stats.approximate_memory_bytes = evidence_.size() *
    (sizeof(VoxelKey) + sizeof(Evidence) + 2U * sizeof(void *));
  result.stats.free_voxels = static_cast<std::size_t>(std::count_if(
    evidence_.begin(), evidence_.end(),
    [](const auto & item) {return item.second.confirmed_free;}));
  return result;
}

void ConservativeFreeSpaceObserver::prune_if_needed()
{
  if (evidence_.size() <= config_.max_voxels) {
    return;
  }
  const auto oldest = scan_index_ > config_.stale_after_scans ?
    scan_index_ - config_.stale_after_scans : 0U;
  for (auto it = evidence_.begin(); it != evidence_.end();) {
    if (it->second.last_seen_scan < oldest && !it->second.confirmed_free) {
      it = evidence_.erase(it);
    } else {
      ++it;
    }
  }
}

void ConservativeFreeSpaceObserver::reset()
{
  evidence_.clear();
  scan_index_ = 0U;
}

VisibilityAwareDynamicObserver::VisibilityAwareDynamicObserver(
  VisibilityFilterConfig config)
: config_(std::move(config))
{
  if (!(config_.voxel_size_m > 0.0) || !(config_.max_range_m > config_.min_range_m)) {
    throw std::invalid_argument("invalid voxel size or range");
  }
  if (config_.free_confirmations == 0U || config_.static_confirmations == 0U ||
    config_.occupied_recovery == 0U || config_.dynamic_confirmations == 0U ||
    config_.dynamic_hold_scans == 0U || config_.vacated_hold_scans == 0U ||
    config_.static_vacate_confirmations == 0U || config_.endpoint_guard_voxels < 0 ||
    config_.dynamic_track_radius_voxels < 0 || config_.vacated_surface_radius_voxels < 0 ||
    config_.static_support_radius_voxels < 0 ||
    !(config_.far_range_m > config_.min_range_m) ||
    config_.far_static_confirmations < config_.static_confirmations ||
    config_.ray_stride < 1)
  {
    throw std::invalid_argument("invalid visibility-aware free-space configuration");
  }
}

VoxelKey VisibilityAwareDynamicObserver::key(const Point & point) const
{
  return {
    static_cast<std::int32_t>(std::floor(point.x / config_.voxel_size_m)),
    static_cast<std::int32_t>(std::floor(point.y / config_.voxel_size_m)),
    static_cast<std::int32_t>(std::floor(point.z / config_.voxel_size_m))};
}

bool VisibilityAwareDynamicObserver::is_valid(const Point & point, const Point & origin) const
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
    return false;
  }
  const double range_squared = squared_distance(point, origin);
  return range_squared >= config_.min_range_m * config_.min_range_m &&
         range_squared <= config_.max_range_m * config_.max_range_m;
}

bool VisibilityAwareDynamicObserver::has_neighbor(
  const VoxelKey & query, const std::unordered_set<VoxelKey, VoxelKeyHash> & candidates,
  int radius) const
{
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -radius; dz <= radius; ++dz) {
        if (candidates.count({query.x + dx, query.y + dy, query.z + dz}) != 0U) {
          return true;
        }
      }
    }
  }
  return false;
}

std::size_t VisibilityAwareDynamicObserver::neighbor_count(
  const VoxelKey & query, const std::unordered_set<VoxelKey, VoxelKeyHash> & candidates,
  int radius) const
{
  std::size_t count = 0U;
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -radius; dz <= radius; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        if (candidates.count({query.x + dx, query.y + dy, query.z + dz}) != 0U) {
          ++count;
        }
      }
    }
  }
  return count;
}

bool VisibilityAwareDynamicObserver::has_recent_dynamic_neighbor(
  const VoxelKey & query, int radius) const
{
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -radius; dz <= radius; ++dz) {
        const auto it = evidence_.find({query.x + dx, query.y + dy, query.z + dz});
        if (it != evidence_.end() && it->second.dynamic_until_scan >= scan_index_) {
          return true;
        }
      }
    }
  }
  return false;
}

bool VisibilityAwareDynamicObserver::has_recent_vacated_neighbor(
  const VoxelKey & query, int radius) const
{
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -radius; dz <= radius; ++dz) {
        const auto it = evidence_.find({query.x + dx, query.y + dy, query.z + dz});
        if (it != evidence_.end() && it->second.vacated_until_scan >= scan_index_) {
          return true;
        }
      }
    }
  }
  return false;
}

void VisibilityAwareDynamicObserver::trace_ray(
  const Point & origin, const Point & endpoint,
  std::unordered_set<VoxelKey, VoxelKeyHash> & traversed) const
{
  const double dx = endpoint.x - origin.x;
  const double dy = endpoint.y - origin.y;
  const double dz = endpoint.z - origin.z;
  const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
  const int steps = static_cast<int>(std::ceil(length / config_.voxel_size_m));
  const int stop = std::max(1, steps - config_.endpoint_guard_voxels - 1);
  for (int index = 1; index < stop; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(steps);
    traversed.insert(key({
      origin.x + ratio * dx, origin.y + ratio * dy, origin.z + ratio * dz, 0.0F}));
  }
}

FilterResult VisibilityAwareDynamicObserver::process(
  const std::vector<Point> & world_points, const Point & sensor_origin)
{
  ++scan_index_;
  FilterResult result;
  result.stats.scan_index = scan_index_;
  result.stats.input_points = world_points.size();
  result.points.reserve(world_points.size());

  std::vector<Point> valid_points;
  valid_points.reserve(world_points.size());
  std::unordered_set<VoxelKey, VoxelKeyHash> endpoints;
  std::unordered_map<VoxelKey, double, VoxelKeyHash> endpoint_ranges;
  for (const auto & point : world_points) {
    if (!is_valid(point, sensor_origin)) {
      continue;
    }
    valid_points.push_back(point);
    const auto endpoint = key(point);
    endpoints.insert(endpoint);
    const double range = std::sqrt(squared_distance(point, sensor_origin));
    const auto range_it = endpoint_ranges.find(endpoint);
    if (range_it == endpoint_ranges.end() || range < range_it->second) {
      endpoint_ranges[endpoint] = range;
    }
  }
  result.stats.valid_points = valid_points.size();

  std::unordered_set<VoxelKey, VoxelKeyHash> guarded_endpoints;
  for (const auto & endpoint : endpoints) {
    for (int dx = -config_.endpoint_guard_voxels; dx <= config_.endpoint_guard_voxels; ++dx) {
      for (int dy = -config_.endpoint_guard_voxels; dy <= config_.endpoint_guard_voxels; ++dy) {
        for (int dz = -config_.endpoint_guard_voxels; dz <= config_.endpoint_guard_voxels; ++dz) {
          guarded_endpoints.insert({endpoint.x + dx, endpoint.y + dy, endpoint.z + dz});
        }
      }
    }
  }

  // Fully-observed free space is the union of voxels traversed by rays that
  // exist in this scan. Missing angular samples never create free evidence.
  std::unordered_set<VoxelKey, VoxelKeyHash> traversed;
  std::unordered_set<VoxelKey, VoxelKeyHash> ray_endpoints;
  for (std::size_t index = 0U; index < valid_points.size();
    index += static_cast<std::size_t>(config_.ray_stride))
  {
    const auto endpoint = key(valid_points[index]);
    if (ray_endpoints.insert(endpoint).second) {
      trace_ray(sensor_origin, valid_points[index], traversed);
    }
  }

  // A previously occupied surface is vacated only when a measured ray passes
  // through it. Occluded or unsampled cells are left untouched.
  std::unordered_set<VoxelKey, VoxelKeyHash> vacated_surfaces;
  for (const auto & traversed_key : traversed) {
    if (guarded_endpoints.count(traversed_key) != 0U) {
      continue;
    }
    const auto it = evidence_.find(traversed_key);
    if (it != evidence_.end() && it->second.confirmed_static) {
      vacated_surfaces.insert(traversed_key);
    }
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> strong_dynamic_seeds;
  for (const auto & endpoint : endpoints) {
    const auto it = evidence_.find(endpoint);
    if (it != evidence_.end() &&
      (it->second.confirmed_free || it->second.dynamic_until_scan >= scan_index_))
    {
      strong_dynamic_seeds.insert(endpoint);
    }
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> predicted_dynamic_voxels;
  for (const auto & point : valid_points) {
    const auto endpoint = key(point);
    const auto existing = evidence_.find(endpoint);
    const bool stable_static = existing != evidence_.end() && existing->second.confirmed_static;
    const bool direct_free_contradiction = strong_dynamic_seeds.count(endpoint) != 0U;
    // Never relabel a still-observed, confirmed static endpoint merely because
    // an adjacent surface was vacated. This protects walls behind people and
    // the static hinge/frame around a moving door.
    const bool articulated_surface_motion = !stable_static && (
      has_neighbor(endpoint, vacated_surfaces, config_.vacated_surface_radius_voxels) ||
      has_recent_vacated_neighbor(endpoint, config_.vacated_surface_radius_voxels));
    const bool current_seed_growth = !stable_static && config_.dynamic_growth_voxels > 0 &&
      has_neighbor(endpoint, strong_dynamic_seeds, config_.dynamic_growth_voxels);
    const bool tracked_motion = !stable_static && has_recent_dynamic_neighbor(
      endpoint, config_.dynamic_track_radius_voxels);

    LabeledPoint labeled;
    labeled.point = point;
    if (direct_free_contradiction) {
      labeled.label = PointLabel::kDynamic;
      labeled.dynamic_score = 1.0F;
    } else if (articulated_surface_motion) {
      labeled.label = PointLabel::kDynamic;
      labeled.dynamic_score = 0.90F;
    } else if (current_seed_growth || tracked_motion) {
      labeled.label = PointLabel::kDynamic;
      labeled.dynamic_score = 0.80F;
    } else if (stable_static) {
      labeled.label = PointLabel::kStatic;
      labeled.dynamic_score = 0.0F;
    } else {
      labeled.label = PointLabel::kUnknown;
      labeled.dynamic_score = 0.5F;
    }

    if (labeled.label == PointLabel::kDynamic) {
      predicted_dynamic_voxels.insert(endpoint);
      ++result.stats.dynamic_points;
    } else if (labeled.label == PointLabel::kStatic) {
      ++result.stats.static_points;
    } else {
      ++result.stats.unknown_points;
    }
    result.points.push_back(labeled);
  }

  for (const auto & traversed_key : traversed) {
    if (guarded_endpoints.count(traversed_key) != 0U) {
      continue;
    }
    auto & evidence = evidence_[traversed_key];
    evidence.free_streak = evidence.last_free_scan + 1U == scan_index_ ?
      saturating_increment(evidence.free_streak) : 1U;
    evidence.last_free_scan = scan_index_;
    evidence.occupied_streak = 0U;
    evidence.last_seen_scan = scan_index_;
    if (evidence.confirmed_static &&
      evidence.free_streak >= config_.static_vacate_confirmations)
    {
      evidence.confirmed_static = false;
      evidence.vacated_until_scan = scan_index_ + config_.vacated_hold_scans;
    }
    if (evidence.free_streak >= config_.free_confirmations) {
      evidence.confirmed_free = true;
    }
  }

  for (const auto & endpoint : endpoints) {
    auto & evidence = evidence_[endpoint];
    evidence.occupied_streak = evidence.last_occupied_scan + 1U == scan_index_ ?
      saturating_increment(evidence.occupied_streak) : 1U;
    evidence.last_occupied_scan = scan_index_;
    evidence.last_seen_scan = scan_index_;
    evidence.free_streak = 0U;

    const bool predicted_dynamic = predicted_dynamic_voxels.count(endpoint) != 0U;
    if (predicted_dynamic) {
      evidence.dynamic_streak = saturating_increment(evidence.dynamic_streak);
      if (evidence.dynamic_streak >= config_.dynamic_confirmations) {
        evidence.dynamic_until_scan = std::max(
          evidence.dynamic_until_scan, scan_index_ + config_.dynamic_hold_scans);
      }
      // A persistent contradiction can be a newly installed static object or
      // a bounded pose error. Recover conservatively instead of keeping an
      // immortal dynamic label.
      if (evidence.occupied_streak >= config_.occupied_recovery) {
        evidence.confirmed_free = false;
        evidence.confirmed_static = true;
        evidence.occupied_observations = config_.static_confirmations;
        evidence.dynamic_streak = 0U;
        evidence.dynamic_until_scan = 0U;
      }
    } else {
      evidence.dynamic_streak = 0U;
      evidence.occupied_observations = saturating_increment(evidence.occupied_observations);
      const bool spatially_supported = neighbor_count(
        endpoint, endpoints, config_.static_support_radius_voxels) >=
        config_.min_static_neighbor_voxels;
      const auto required_static_confirmations =
        endpoint_ranges.at(endpoint) >= config_.far_range_m ?
        config_.far_static_confirmations : config_.static_confirmations;
      if (!evidence.confirmed_free &&
        evidence.occupied_observations >= required_static_confirmations && spatially_supported)
      {
        evidence.confirmed_static = true;
      }
    }
  }

  prune_if_needed();
  result.stats.dynamic_seed_voxels = strong_dynamic_seeds.size();
  result.stats.observed_ray_voxels = traversed.size();
  result.stats.vacated_surface_voxels = vacated_surfaces.size();
  result.stats.allocated_voxels = evidence_.size();
  result.stats.approximate_memory_bytes = evidence_.size() *
    (sizeof(VoxelKey) + sizeof(Evidence) + 2U * sizeof(void *));
  for (const auto & item : evidence_) {
    if (item.second.confirmed_free) {
      ++result.stats.free_voxels;
    }
    if (item.second.dynamic_until_scan >= scan_index_) {
      ++result.stats.persistent_dynamic_voxels;
    }
  }
  return result;
}

void VisibilityAwareDynamicObserver::prune_if_needed()
{
  if (evidence_.size() <= config_.max_voxels) {
    return;
  }
  const auto oldest = scan_index_ > config_.stale_after_scans ?
    scan_index_ - config_.stale_after_scans : 0U;
  for (auto it = evidence_.begin(); it != evidence_.end();) {
    if (it->second.last_seen_scan < oldest &&
      it->second.dynamic_until_scan < scan_index_ && !it->second.confirmed_free)
    {
      it = evidence_.erase(it);
    } else {
      ++it;
    }
  }
}

void VisibilityAwareDynamicObserver::reset()
{
  evidence_.clear();
  scan_index_ = 0U;
}

TemporalVoxelBaseline::TemporalVoxelBaseline(
  double voxel_size_m, std::size_t window_frames, std::size_t min_static_support,
  int neighbor_radius)
: voxel_size_m_(voxel_size_m),
  window_frames_(window_frames),
  min_static_support_(min_static_support),
  neighbor_radius_(neighbor_radius)
{
  if (!(voxel_size_m_ > 0.0) || window_frames_ == 0U || min_static_support_ == 0U ||
    neighbor_radius_ < 0)
  {
    throw std::invalid_argument("invalid temporal baseline configuration");
  }
}

VoxelKey TemporalVoxelBaseline::key(const Point & point) const
{
  return {
    static_cast<std::int32_t>(std::floor(point.x / voxel_size_m_)),
    static_cast<std::int32_t>(std::floor(point.y / voxel_size_m_)),
    static_cast<std::int32_t>(std::floor(point.z / voxel_size_m_))};
}

bool TemporalVoxelBaseline::supported(
  const VoxelKey & query, const std::unordered_set<VoxelKey, VoxelKeyHash> & frame) const
{
  for (int dx = -neighbor_radius_; dx <= neighbor_radius_; ++dx) {
    for (int dy = -neighbor_radius_; dy <= neighbor_radius_; ++dy) {
      for (int dz = -neighbor_radius_; dz <= neighbor_radius_; ++dz) {
        if (frame.count({query.x + dx, query.y + dy, query.z + dz}) != 0U) {
          return true;
        }
      }
    }
  }
  return false;
}

FilterResult TemporalVoxelBaseline::process(
  const std::vector<Point> & world_points, const Point & /*sensor_origin*/)
{
  ++scan_index_;
  FilterResult result;
  result.stats.scan_index = scan_index_;
  result.stats.input_points = world_points.size();
  result.stats.valid_points = world_points.size();
  result.points.reserve(world_points.size());

  std::unordered_set<VoxelKey, VoxelKeyHash> current;
  for (const auto & point : world_points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      current.insert(key(point));
    }
  }
  const bool warm = history_.size() >= window_frames_;
  for (const auto & point : world_points) {
    LabeledPoint labeled;
    labeled.point = point;
    const auto point_key = key(point);
    std::size_t support = 0U;
    for (const auto & frame : history_) {
      if (supported(point_key, frame)) {
        ++support;
      }
    }
    if (support >= min_static_support_) {
      labeled.label = PointLabel::kStatic;
      labeled.dynamic_score = 0.0F;
      ++result.stats.static_points;
    } else if (warm && support == 0U) {
      labeled.label = PointLabel::kDynamic;
      labeled.dynamic_score = 1.0F;
      ++result.stats.dynamic_points;
    } else {
      labeled.label = PointLabel::kUnknown;
      labeled.dynamic_score = 0.5F;
      ++result.stats.unknown_points;
    }
    result.points.push_back(labeled);
  }
  history_.push_back(std::move(current));
  while (history_.size() > window_frames_) {
    history_.pop_front();
  }
  std::size_t keys = 0U;
  for (const auto & frame : history_) {
    keys += frame.size();
  }
  result.stats.allocated_voxels = keys;
  result.stats.approximate_memory_bytes = keys *
    (sizeof(VoxelKey) + 2U * sizeof(void *));
  return result;
}

void TemporalVoxelBaseline::reset()
{
  history_.clear();
  scan_index_ = 0U;
}

}  // namespace uf_dynamic_observer
