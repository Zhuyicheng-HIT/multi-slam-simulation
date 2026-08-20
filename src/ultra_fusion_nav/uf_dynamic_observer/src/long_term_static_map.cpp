#include "uf_dynamic_observer/long_term_static_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace uf_dynamic_observer
{
namespace
{

std::uint32_t saturating_increment(std::uint32_t value)
{
  return value == std::numeric_limits<std::uint32_t>::max() ? value : value + 1U;
}

std::uint8_t bit_count(std::uint64_t value)
{
  std::uint8_t count = 0U;
  while (value != 0U) {
    value &= value - 1U;
    ++count;
  }
  return count;
}

double squared_distance(const Point & lhs, const Point & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  const double dz = lhs.z - rhs.z;
  return dx * dx + dy * dy + dz * dz;
}

}  // namespace

const char * to_string(LongTermVoxelState state)
{
  switch (state) {
    case LongTermVoxelState::kUnknown:
      return "UNKNOWN";
    case LongTermVoxelState::kStaticCandidate:
      return "STATIC_CANDIDATE";
    case LongTermVoxelState::kStaticConfirmed:
      return "STATIC_CONFIRMED";
    case LongTermVoxelState::kDynamicCandidate:
      return "DYNAMIC_CANDIDATE";
    case LongTermVoxelState::kDynamicConfirmed:
      return "DYNAMIC_CONFIRMED";
  }
  return "UNKNOWN";
}

LongTermStaticMap::LongTermStaticMap(LongTermMapConfig config)
: config_(std::move(config))
{
  if (!(config_.voxel_size_m > 0.0) || !(config_.max_range_m > config_.min_range_m) ||
    config_.static_candidate_observations == 0U ||
    config_.static_confirmed_observations < config_.static_candidate_observations ||
    !(config_.static_confirmed_duration_s >= 0.0) ||
    config_.static_confirmed_view_bins == 0U ||
    !(config_.static_consistency_ratio > 0.0 && config_.static_consistency_ratio <= 1.0) ||
    config_.candidate_free_contradictions == 0U ||
    config_.dynamic_candidate_free_traversals == 0U ||
    config_.dynamic_confirmed_free_traversals < config_.dynamic_candidate_free_traversals ||
    config_.dynamic_confirmed_view_bins == 0U ||
    config_.dynamic_label_confirmations == 0U ||
    config_.dynamic_recovery_static_observations == 0U ||
    config_.far_static_confirmed_observations == 0U ||
    config_.far_static_confirmed_view_bins == 0U ||
    !(config_.far_static_confirmed_duration_s >= config_.static_confirmed_duration_s) ||
    config_.endpoint_guard_voxels < 0 || config_.ray_stride < 1 || config_.max_voxels == 0U)
  {
    throw std::invalid_argument("invalid long-term static-map configuration");
  }
}

VoxelKey LongTermStaticMap::key(const Point & point) const
{
  return {
    static_cast<std::int32_t>(std::floor(point.x / config_.voxel_size_m)),
    static_cast<std::int32_t>(std::floor(point.y / config_.voxel_size_m)),
    static_cast<std::int32_t>(std::floor(point.z / config_.voxel_size_m))};
}

Point LongTermStaticMap::center(const VoxelKey & voxel) const
{
  return {
    (static_cast<double>(voxel.x) + 0.5) * config_.voxel_size_m,
    (static_cast<double>(voxel.y) + 0.5) * config_.voxel_size_m,
    (static_cast<double>(voxel.z) + 0.5) * config_.voxel_size_m, 0.0F};
}

bool LongTermStaticMap::valid(const Point & point, const Point & origin) const
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
    return false;
  }
  const double range_squared = squared_distance(point, origin);
  return range_squared >= config_.min_range_m * config_.min_range_m &&
         range_squared <= config_.max_range_m * config_.max_range_m;
}

std::uint64_t LongTermStaticMap::view_bit(const Point & origin, const Point & endpoint) const
{
  constexpr double kPi = 3.14159265358979323846;
  double azimuth = std::atan2(endpoint.y - origin.y, endpoint.x - origin.x);
  const double elevation = std::atan2(
    endpoint.z - origin.z,
    std::hypot(endpoint.x - origin.x, endpoint.y - origin.y));
  if (azimuth < 0.0) {
    azimuth += 2.0 * kPi;
  }
  const auto azimuth_bin = static_cast<std::uint64_t>(
    std::min(15, static_cast<int>(std::floor(azimuth / (2.0 * kPi) * 16.0))));
  const auto elevation_bin = static_cast<std::uint64_t>(
    std::clamp(static_cast<int>(std::floor((elevation + 0.5 * kPi) / kPi * 4.0)), 0, 3));
  return std::uint64_t{1} << (elevation_bin * 16U + azimuth_bin);
}

void LongTermStaticMap::trace_ray(
  const Point & origin, const Point & endpoint, std::vector<VoxelKey> & traversed) const
{
  const double dx = endpoint.x - origin.x;
  const double dy = endpoint.y - origin.y;
  const double dz = endpoint.z - origin.z;
  const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
  const int steps = static_cast<int>(std::ceil(length / config_.voxel_size_m));
  const int stop = std::max(1, steps - config_.endpoint_guard_voxels - 1);
  VoxelKey previous{std::numeric_limits<std::int32_t>::min(), 0, 0};
  for (int index = 1; index < stop; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(steps);
    const auto current = key({
      origin.x + ratio * dx, origin.y + ratio * dy, origin.z + ratio * dz, 0.0F});
    if (!(current == previous)) {
      traversed.push_back(current);
      previous = current;
    }
  }
}

void LongTermStaticMap::transition(
  Element & element, LongTermVoxelState next, double stamp_s)
{
  const auto previous = element.state;
  if (previous == next) {
    return;
  }
  if (next == LongTermVoxelState::kStaticCandidate) {
    element.candidate_since_s = stamp_s;
  } else if (next == LongTermVoxelState::kStaticConfirmed) {
    ++promoted_static_voxels_;
    admission_delay_sum_s_ += std::max(0.0, stamp_s - element.candidate_since_s);
    element.was_permanently_admitted = true;
    element.ghost_removal_counted = false;
    element.free_since_occupied = 0U;
    element.free_view_mask = 0U;
    element.first_free_contradiction_s = 0.0;
  } else if (next == LongTermVoxelState::kDynamicConfirmed &&
    element.was_permanently_admitted && !element.ghost_removal_counted)
  {
    ++removed_ghost_voxels_;
    element.ghost_removal_counted = true;
  }
  if (next == LongTermVoxelState::kDynamicConfirmed) {
    // Recovery must be earned from fresh post-dynamic observations. Historical
    // static support cannot immediately re-admit a vacated object.
    element.static_support = 0U;
    element.first_occupied_s = stamp_s;
  }
  element.state = next;
}

void LongTermStaticMap::apply_free_evidence(
  const VoxelKey & voxel, std::uint64_t view, double stamp_s)
{
  auto it = elements_.find(voxel);
  if (it == elements_.end()) {
    return;
  }
  auto & element = it->second;
  element.free_traversals = saturating_increment(element.free_traversals);
  element.free_since_occupied = saturating_increment(element.free_since_occupied);
  element.free_view_mask |= view;
  element.last_free_s = stamp_s;
  element.last_seen_scan = scan_index_;
  if (element.first_free_contradiction_s <= 0.0) {
    element.first_free_contradiction_s = stamp_s;
  }

  if (element.state == LongTermVoxelState::kStaticCandidate &&
    element.free_since_occupied >= config_.candidate_free_contradictions)
  {
    transition(element, LongTermVoxelState::kUnknown, stamp_s);
    element.static_support = 0U;
    element.occupied_view_mask = 0U;
    return;
  }
  if (element.state == LongTermVoxelState::kStaticConfirmed &&
    element.free_since_occupied >= config_.dynamic_candidate_free_traversals)
  {
    transition(element, LongTermVoxelState::kDynamicCandidate, stamp_s);
  }
  if (element.state == LongTermVoxelState::kDynamicCandidate &&
    element.free_since_occupied >= config_.dynamic_confirmed_free_traversals &&
    bit_count(element.free_view_mask) >= config_.dynamic_confirmed_view_bins &&
    stamp_s - element.first_free_contradiction_s >= config_.dynamic_confirmed_duration_s)
  {
    transition(element, LongTermVoxelState::kDynamicConfirmed, stamp_s);
  }
}

void LongTermStaticMap::apply_occupied_evidence(
  const VoxelKey & voxel, const LabeledPoint & observation,
  std::uint64_t view, double stamp_s, bool far_range)
{
  auto existing = elements_.find(voxel);
  if (existing == elements_.end() && elements_.size() >= config_.max_voxels) {
    prune_if_needed();
    existing = elements_.find(voxel);
    if (existing == elements_.end() && elements_.size() >= config_.max_voxels) {
      // Never evict STATIC_CONFIRMED merely to admit a new candidate. Holding
      // the bounded map is safer than unbounded growth or silent map erosion.
      ++capacity_rejected_voxels_;
      return;
    }
  }
  auto inserted = elements_.try_emplace(voxel);
  auto & element = inserted.first->second;
  if (inserted.second) {
    element.centroid = observation.point;
    element.first_occupied_s = stamp_s;
    element.candidate_since_s = stamp_s;
  }
  const auto previous_occupied = element.occupied_support;
  element.occupied_support = saturating_increment(element.occupied_support);
  const double denominator = static_cast<double>(element.occupied_support);
  element.centroid.x =
    (element.centroid.x * previous_occupied + observation.point.x) / denominator;
  element.centroid.y =
    (element.centroid.y * previous_occupied + observation.point.y) / denominator;
  element.centroid.z =
    (element.centroid.z * previous_occupied + observation.point.z) / denominator;
  element.centroid.intensity = static_cast<float>(
    (static_cast<double>(element.centroid.intensity) * previous_occupied +
    observation.point.intensity) / denominator);
  element.last_occupied_s = stamp_s;
  element.last_seen_scan = scan_index_;
  element.occupied_view_mask |= view;
  element.free_since_occupied = 0U;
  element.free_view_mask = 0U;
  element.first_free_contradiction_s = 0.0;

  if (observation.label == PointLabel::kDynamic) {
    element.dynamic_support = saturating_increment(element.dynamic_support);
    if (element.state == LongTermVoxelState::kStaticConfirmed ||
      element.state == LongTermVoxelState::kStaticCandidate ||
      element.state == LongTermVoxelState::kUnknown)
    {
      transition(element, LongTermVoxelState::kDynamicCandidate, stamp_s);
    }
    if (element.dynamic_support >= config_.dynamic_label_confirmations) {
      transition(element, LongTermVoxelState::kDynamicConfirmed, stamp_s);
    }
    return;
  }

  if (observation.label == PointLabel::kUnknown) {
    // UNKNOWN is deliberately retained by the clean scan but never earns
    // permanent-map admission on its own.
    return;
  }

  element.static_support = saturating_increment(element.static_support);
  if (element.state == LongTermVoxelState::kDynamicCandidate ||
    element.state == LongTermVoxelState::kDynamicConfirmed)
  {
    const bool recovered = element.static_support >= config_.dynamic_recovery_static_observations &&
      stamp_s - element.first_occupied_s >= config_.dynamic_recovery_duration_s;
    if (!recovered) {
      return;
    }
    element.dynamic_support = 0U;
    element.free_traversals = 0U;
    transition(element, LongTermVoxelState::kStaticCandidate, stamp_s);
  }
  if (element.state == LongTermVoxelState::kUnknown &&
    element.static_support >= config_.static_candidate_observations)
  {
    transition(element, LongTermVoxelState::kStaticCandidate, stamp_s);
  }
  const auto required_observations = far_range ?
    config_.far_static_confirmed_observations : config_.static_confirmed_observations;
  const auto required_views = far_range ?
    config_.far_static_confirmed_view_bins : config_.static_confirmed_view_bins;
  const double required_duration_s = far_range ?
    config_.far_static_confirmed_duration_s : config_.static_confirmed_duration_s;
  const double evidence_total = static_cast<double>(
    element.static_support + element.dynamic_support + element.free_traversals);
  const double consistency = static_cast<double>(element.static_support) /
    std::max(1.0, evidence_total);
  if (element.state == LongTermVoxelState::kStaticCandidate &&
    element.static_support >= required_observations &&
    stamp_s - element.candidate_since_s >= required_duration_s &&
    bit_count(element.occupied_view_mask) >= required_views &&
    consistency >= config_.static_consistency_ratio)
  {
    transition(element, LongTermVoxelState::kStaticConfirmed, stamp_s);
  }
}

LongTermUpdateResult LongTermStaticMap::integrate(
  const std::vector<LabeledPoint> & observations, const Point & sensor_origin,
  double stamp_s)
{
  LongTermUpdateResult result;
  if (!std::isfinite(stamp_s)) {
    ++rejected_scans_;
    result.reason = "non_finite_timestamp";
    result.stats = stats();
    return result;
  }
  if (last_stamp_s_ >= 0.0 && stamp_s < last_stamp_s_) {
    ++rejected_scans_;
    ++timestamp_regressions_;
    result.reason = "timestamp_regression";
    result.stats = stats();
    return result;
  }
  if (!std::isfinite(sensor_origin.x) || !std::isfinite(sensor_origin.y) ||
    !std::isfinite(sensor_origin.z))
  {
    ++rejected_scans_;
    result.reason = "invalid_sensor_origin";
    result.stats = stats();
    return result;
  }

  ++scan_index_;
  ++accepted_scans_;
  last_stamp_s_ = stamp_s;
  input_points_ += observations.size();
  std::vector<const LabeledPoint *> valid_observations;
  valid_observations.reserve(observations.size());
  std::unordered_set<VoxelKey, VoxelKeyHash> endpoints;
  for (const auto & observation : observations) {
    if (!valid(observation.point, sensor_origin)) {
      continue;
    }
    valid_observations.push_back(&observation);
    endpoints.insert(key(observation.point));
  }
  valid_points_ += valid_observations.size();

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

  std::unordered_set<VoxelKey, VoxelKeyHash> unique_traversed;
  std::vector<VoxelKey> ray;
  for (std::size_t index = 0U; index < valid_observations.size();
    index += static_cast<std::size_t>(config_.ray_stride))
  {
    ray.clear();
    const auto & observation = *valid_observations[index];
    trace_ray(sensor_origin, observation.point, ray);
    const auto view = view_bit(sensor_origin, observation.point);
    for (const auto & traversed : ray) {
      if (guarded_endpoints.count(traversed) == 0U && unique_traversed.insert(traversed).second) {
        apply_free_evidence(traversed, view, stamp_s);
      }
    }
  }
  actual_ray_voxels_ += unique_traversed.size();

  for (const auto * observation : valid_observations) {
    apply_occupied_evidence(
      key(observation->point), *observation,
      view_bit(sensor_origin, observation->point), stamp_s,
      squared_distance(sensor_origin, observation->point) >=
      config_.far_range_m * config_.far_range_m);
  }
  prune_if_needed();
  result.accepted = true;
  result.reason = "accepted";
  result.stats = stats();
  return result;
}

bool LongTermStaticMap::add_semantic_evidence(
  const Point & world_point, float dynamic_confidence, double stamp_s,
  bool shadow_only)
{
  ++semantic_messages_;
  if (!std::isfinite(stamp_s) || !std::isfinite(dynamic_confidence) ||
    dynamic_confidence < config_.semantic_dynamic_threshold)
  {
    return false;
  }
  auto it = elements_.find(key(world_point));
  if (it == elements_.end()) {
    return false;
  }
  if (shadow_only) {
    ++semantic_shadow_hits_;
    return true;
  }
  ++semantic_applied_hits_;
  auto & element = it->second;
  element.dynamic_support = saturating_increment(element.dynamic_support);
  transition(element, LongTermVoxelState::kDynamicCandidate, stamp_s);
  if (element.dynamic_support >= config_.dynamic_label_confirmations) {
    transition(element, LongTermVoxelState::kDynamicConfirmed, stamp_s);
  }
  return true;
}

std::vector<StaticMapPoint> LongTermStaticMap::static_confirmed_points() const
{
  std::vector<StaticMapPoint> output;
  output.reserve(elements_.size());
  for (const auto & item : elements_) {
    const auto & element = item.second;
    if (element.state != LongTermVoxelState::kStaticConfirmed) {
      continue;
    }
    const double evidence_total = static_cast<double>(
      element.static_support + element.dynamic_support + element.free_traversals);
    output.push_back({
      element.centroid, element.static_support,
      static_cast<float>(static_cast<double>(element.static_support) /
      std::max(1.0, evidence_total))});
  }
  return output;
}

LongTermVoxelState LongTermStaticMap::state_at(const Point & world_point) const
{
  const auto it = elements_.find(key(world_point));
  return it == elements_.end() ? LongTermVoxelState::kUnknown : it->second.state;
}

void LongTermStaticMap::prune_if_needed()
{
  if (elements_.size() < config_.max_voxels) {
    return;
  }
  const auto stale_before = scan_index_ > config_.stale_dynamic_after_scans ?
    scan_index_ - config_.stale_dynamic_after_scans : 0U;
  for (auto it = elements_.begin(); it != elements_.end() && elements_.size() >= config_.max_voxels;) {
    if (it->second.state != LongTermVoxelState::kStaticConfirmed &&
      it->second.last_seen_scan < stale_before)
    {
      it = elements_.erase(it);
    } else {
      ++it;
    }
  }
}

void LongTermStaticMap::refresh_counts(LongTermMapStats & output) const
{
  for (const auto & item : elements_) {
    switch (item.second.state) {
      case LongTermVoxelState::kUnknown:
        ++output.unknown_voxels;
        break;
      case LongTermVoxelState::kStaticCandidate:
        ++output.static_candidate_voxels;
        break;
      case LongTermVoxelState::kStaticConfirmed:
        ++output.static_confirmed_voxels;
        break;
      case LongTermVoxelState::kDynamicCandidate:
        ++output.dynamic_candidate_voxels;
        break;
      case LongTermVoxelState::kDynamicConfirmed:
        ++output.dynamic_confirmed_voxels;
        break;
    }
  }
}

LongTermMapStats LongTermStaticMap::stats() const
{
  LongTermMapStats output;
  output.scan_index = scan_index_;
  output.accepted_scans = accepted_scans_;
  output.rejected_scans = rejected_scans_;
  output.timestamp_regressions = timestamp_regressions_;
  output.input_points = input_points_;
  output.valid_points = valid_points_;
  output.promoted_static_voxels = promoted_static_voxels_;
  output.removed_ghost_voxels = removed_ghost_voxels_;
  output.semantic_messages = semantic_messages_;
  output.semantic_shadow_hits = semantic_shadow_hits_;
  output.semantic_applied_hits = semantic_applied_hits_;
  output.capacity_rejected_voxels = capacity_rejected_voxels_;
  output.allocated_voxels = elements_.size();
  output.actual_ray_voxels = static_cast<std::size_t>(actual_ray_voxels_);
  output.approximate_memory_bytes = elements_.size() *
    (sizeof(VoxelKey) + sizeof(Element) + 2U * sizeof(void *));
  refresh_counts(output);
  output.mean_admission_delay_s = promoted_static_voxels_ == 0U ? 0.0 :
    admission_delay_sum_s_ / static_cast<double>(promoted_static_voxels_);
  const auto permanent_total = output.static_confirmed_voxels +
    output.dynamic_confirmed_voxels + output.static_candidate_voxels +
    output.dynamic_candidate_voxels + output.unknown_voxels;
  output.promoted_static_ratio = static_cast<double>(output.static_confirmed_voxels) /
    static_cast<double>(std::max<std::size_t>(1U, permanent_total));
  output.permanent_rejection_ratio = 1.0 - output.promoted_static_ratio;
  return output;
}

void LongTermStaticMap::reset()
{
  elements_.clear();
  scan_index_ = 0U;
  last_stamp_s_ = -1.0;
  accepted_scans_ = 0U;
  rejected_scans_ = 0U;
  timestamp_regressions_ = 0U;
  input_points_ = 0U;
  valid_points_ = 0U;
  promoted_static_voxels_ = 0U;
  removed_ghost_voxels_ = 0U;
  semantic_messages_ = 0U;
  semantic_shadow_hits_ = 0U;
  semantic_applied_hits_ = 0U;
  capacity_rejected_voxels_ = 0U;
  actual_ray_voxels_ = 0U;
  admission_delay_sum_s_ = 0.0;
}

}  // namespace uf_dynamic_observer
