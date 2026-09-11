#include "uf_dynamic_observer/conservative_free_space.hpp"

#include <sys/resource.h>
#include <time.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>

namespace uf_dynamic_observer
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int kFrameCount = 40;
constexpr int kRepeatsPerSeed = 2;
// The largest configured detector history requirement is V2's
// far_static_confirmations=12. Keep those startup frames in the all-frame
// score, while also exposing them as a separate diagnostic window.
constexpr std::size_t kWarmupFrameCount = 12U;
constexpr std::array<std::uint32_t, 3> kSeeds{{101U, 202U, 303U}};

struct TruthPoint
{
  Point point;
  bool dynamic{false};
};

struct Frame
{
  Point origin;
  std::vector<TruthPoint> points;
};

struct RepeatabilityFrame
{
  std::uint32_t seed{0U};
  int repetition{0};
  std::size_t previous_frame{0U};
  std::size_t current_frame{0U};
  bool startup_warmup{false};
  std::uint64_t common_visible{0U};
  std::uint64_t retained{0U};
  std::uint64_t repeatable{0U};
  std::uint64_t previous_dynamic{0U};
  std::uint64_t previous_unknown{0U};
  double value{0.0};
};

struct Counts
{
  std::uint64_t true_dynamic{0U};
  std::uint64_t true_static{0U};
  std::uint64_t true_positive{0U};
  std::uint64_t false_positive{0U};
  std::uint64_t false_negative{0U};
  std::uint64_t true_negative{0U};
  std::uint64_t dynamic_as_static{0U};
  std::uint64_t static_confirmed{0U};
  std::uint64_t unknown{0U};
  std::uint64_t dynamic_unknown{0U};
  std::uint64_t frames{0U};
  std::size_t peak_memory_bytes{0U};
  std::size_t peak_voxels{0U};
  std::size_t peak_ray_voxels{0U};
  std::size_t peak_vacated_voxels{0U};
  std::size_t peak_persistent_dynamic_voxels{0U};
  std::vector<double> latency_ms;
  std::vector<double> cpu_ms;
  std::uint64_t repeatable_static_features{0U};
  std::uint64_t common_visible_static_features{0U};
  std::uint64_t retained_common_static_features{0U};
  std::vector<double> frame_repeatability;
  std::uint64_t warmup_repeatable_static_features{0U};
  std::uint64_t warmup_common_visible_static_features{0U};
  std::uint64_t warmup_retained_common_static_features{0U};
  std::vector<double> warmup_frame_repeatability;
  std::uint64_t steady_repeatable_static_features{0U};
  std::uint64_t steady_common_visible_static_features{0U};
  std::uint64_t steady_retained_common_static_features{0U};
  std::vector<double> steady_frame_repeatability;
  std::vector<RepeatabilityFrame> repeatability_frames;
};

struct ScenarioResult
{
  std::string name;
  Counts counts;
};

double divide(double numerator, double denominator, double fallback = 0.0)
{
  return denominator > 0.0 ? numerator / denominator : fallback;
}

double precision(const Counts & c)
{
  return divide(static_cast<double>(c.true_positive),
    static_cast<double>(c.true_positive + c.false_positive), 0.0);
}

double recall(const Counts & c)
{
  return divide(static_cast<double>(c.true_positive),
    static_cast<double>(c.true_positive + c.false_negative), 0.0);
}

double f1(const Counts & c)
{
  const double p = precision(c);
  const double r = recall(c);
  return divide(2.0 * p * r, p + r, 0.0);
}

double percentile(std::vector<double> values, double q)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(
    std::round(q * static_cast<double>(values.size() - 1U)));
  return values[std::min(index, values.size() - 1U)];
}

double thread_cpu_ms()
{
  timespec value{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0) {
    return 0.0;
  }
  return 1000.0 * static_cast<double>(value.tv_sec) +
         1.0e-6 * static_cast<double>(value.tv_nsec);
}

struct FeatureGeometry
{
  Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
  std::size_t neighbors{0U};
  bool valid{false};
};

using SpatialIndex = std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash>;

VoxelKey spatial_key(const Point & point, double cell_size)
{
  return {
    static_cast<std::int32_t>(std::floor(point.x / cell_size)),
    static_cast<std::int32_t>(std::floor(point.y / cell_size)),
    static_cast<std::int32_t>(std::floor(point.z / cell_size))};
}

SpatialIndex index_static_points(const Frame & frame, double cell_size)
{
  SpatialIndex index;
  for (std::size_t point_index = 0U; point_index < frame.points.size(); ++point_index) {
    if (!frame.points[point_index].dynamic) {
      index[spatial_key(frame.points[point_index].point, cell_size)].push_back(point_index);
    }
  }
  return index;
}

std::vector<FeatureGeometry> estimate_static_geometry(const Frame & frame)
{
  constexpr double radius_squared = 0.75 * 0.75;
  constexpr double cell_size = 0.75;
  std::vector<FeatureGeometry> output(frame.points.size());
  const auto spatial_index = index_static_points(frame, cell_size);
  for (std::size_t index = 0U; index < frame.points.size(); ++index) {
    if (frame.points[index].dynamic) {
      continue;
    }
    const auto & center = frame.points[index].point;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> neighbors;
    const auto center_key = spatial_key(center, cell_size);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const auto found = spatial_index.find({
              center_key.x + dx, center_key.y + dy, center_key.z + dz});
          if (found == spatial_index.end()) {
            continue;
          }
          for (const auto candidate_index : found->second) {
            const auto & candidate = frame.points[candidate_index].point;
            const Eigen::Vector3d delta(
              candidate.x - center.x, candidate.y - center.y, candidate.z - center.z);
            if (delta.squaredNorm() <= radius_squared) {
              neighbors.push_back(delta);
              mean += delta;
            }
          }
        }
      }
    }
    if (neighbors.size() < 5U) {
      continue;
    }
    mean /= static_cast<double>(neighbors.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto & neighbor : neighbors) {
      const auto centered = neighbor - mean;
      covariance.noalias() += centered * centered.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
      continue;
    }
    output[index].normal = solver.eigenvectors().col(0).normalized();
    output[index].neighbors = neighbors.size();
    output[index].valid = output[index].normal.allFinite();
  }
  return output;
}

bool retained_static(
  const Frame & frame, const FilterResult & prediction, std::size_t index,
  bool raw_passthrough)
{
  return index < frame.points.size() && !frame.points[index].dynamic &&
         (raw_passthrough ||
         (index < prediction.points.size() &&
         prediction.points[index].label == PointLabel::kStatic));
}

void accumulate_repeatability(
  Counts & counts, const Frame & previous, const FilterResult & previous_prediction,
  const Frame & current, const FilterResult & current_prediction, bool raw_passthrough,
  bool startup_warmup, std::uint32_t seed, int repetition,
  std::size_t previous_frame_index, std::size_t current_frame_index)
{
  constexpr double maximum_distance_squared = 0.45 * 0.45;
  constexpr double match_cell_size = 0.45;
  constexpr double minimum_normal_dot = 0.90;
  const auto previous_geometry = estimate_static_geometry(previous);
  const auto current_geometry = estimate_static_geometry(current);
  const auto previous_index = index_static_points(previous, match_cell_size);
  std::uint64_t eligible = 0U;
  std::uint64_t retained = 0U;
  std::uint64_t repeatable = 0U;
  std::uint64_t previous_dynamic = 0U;
  std::uint64_t previous_unknown = 0U;
  for (std::size_t current_index = 0U; current_index < current.points.size(); ++current_index) {
    if (current.points[current_index].dynamic || !current_geometry[current_index].valid) {
      continue;
    }
    const auto & point = current.points[current_index].point;
    std::size_t best_index = previous.points.size();
    double best_squared = maximum_distance_squared;
    const auto point_key = spatial_key(point, match_cell_size);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const auto found = previous_index.find({
              point_key.x + dx, point_key.y + dy, point_key.z + dz});
          if (found == previous_index.end()) {
            continue;
          }
          for (const auto candidate_index : found->second) {
            if (!previous_geometry[candidate_index].valid) {
              continue;
            }
            const auto & candidate = previous.points[candidate_index].point;
            const double delta_x = point.x - candidate.x;
            const double delta_y = point.y - candidate.y;
            const double delta_z = point.z - candidate.z;
            const double squared = delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
            const double normal_dot = std::abs(
              current_geometry[current_index].normal.dot(previous_geometry[candidate_index].normal));
            const auto current_neighbors = current_geometry[current_index].neighbors;
            const auto previous_neighbors = previous_geometry[candidate_index].neighbors;
            const double support_ratio = static_cast<double>(
              std::min(current_neighbors, previous_neighbors)) /
              static_cast<double>(std::max(current_neighbors, previous_neighbors));
            if (squared <= best_squared && normal_dot >= minimum_normal_dot &&
              support_ratio >= 0.5)
            {
              best_squared = squared;
              best_index = candidate_index;
            }
          }
        }
      }
    }
    if (best_index == previous.points.size()) {
      continue;
    }
    ++eligible;
    const bool current_retained = retained_static(
      current, current_prediction, current_index, raw_passthrough);
    if (current_retained) {
      ++retained;
    }
    if (current_retained) {
      if (retained_static(previous, previous_prediction, best_index, raw_passthrough)) {
        ++repeatable;
      } else if (best_index < previous_prediction.points.size() &&
        previous_prediction.points[best_index].label == PointLabel::kDynamic)
      {
        ++previous_dynamic;
      } else {
        ++previous_unknown;
      }
    }
  }
  counts.common_visible_static_features += eligible;
  counts.retained_common_static_features += retained;
  counts.repeatable_static_features += repeatable;
  auto & window_repeatable = startup_warmup ?
    counts.warmup_repeatable_static_features : counts.steady_repeatable_static_features;
  auto & window_common = startup_warmup ?
    counts.warmup_common_visible_static_features : counts.steady_common_visible_static_features;
  auto & window_retained = startup_warmup ?
    counts.warmup_retained_common_static_features : counts.steady_retained_common_static_features;
  auto & window_frames = startup_warmup ?
    counts.warmup_frame_repeatability : counts.steady_frame_repeatability;
  window_common += eligible;
  window_retained += retained;
  window_repeatable += repeatable;
  if (retained > 0U) {
    const double value = static_cast<double>(repeatable) / static_cast<double>(retained);
    counts.frame_repeatability.push_back(value);
    window_frames.push_back(value);
    counts.repeatability_frames.push_back({
      seed, repetition, previous_frame_index, current_frame_index, startup_warmup,
      eligible, retained, repeatable, previous_dynamic, previous_unknown, value});
  }
}

class RawPassThrough
{
public:
  FilterResult process(const std::vector<Point> & points, const Point &)
  {
    FilterResult result;
    result.points.reserve(points.size());
    for (const auto & point : points) {
      result.points.push_back({point, PointLabel::kStatic, 0.0F});
    }
    result.stats.input_points = points.size();
    result.stats.valid_points = points.size();
    result.stats.static_points = points.size();
    return result;
  }
  void reset() {}
};

std::uint64_t mix_hash(std::uint64_t value)
{
  value ^= value >> 30U;
  value *= 0xbf58476d1ce4e5b9ULL;
  value ^= value >> 27U;
  value *= 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

void add_wall(std::vector<TruthPoint> & points, double x, double y_min, double y_max)
{
  for (double y = y_min; y <= y_max + 1.0e-9; y += 0.32) {
    for (double z = -0.4; z <= 2.8 + 1.0e-9; z += 0.32) {
      points.push_back({{x, y, z, 20.0F}, false});
    }
  }
}

void add_side_wall(std::vector<TruthPoint> & points, double y)
{
  for (double x = 2.0; x <= 9.0 + 1.0e-9; x += 0.38) {
    for (double z = -0.4; z <= 2.8 + 1.0e-9; z += 0.34) {
      points.push_back({{x, y, z, 25.0F}, false});
    }
  }
}

void add_cluster(
  std::vector<TruthPoint> & points, double x, double y, double half_width, double height,
  bool dynamic, double spacing = 0.18, double base_z = -0.35)
{
  for (double dx = -half_width; dx <= half_width + 1.0e-9; dx += spacing) {
    for (double dy = -half_width; dy <= half_width + 1.0e-9; dy += spacing) {
      for (double z = base_z; z <= base_z + height + 1.0e-9; z += spacing + 0.04) {
        points.push_back({{x + dx, y + dy, z, 80.0F}, dynamic});
      }
    }
  }
}

void add_door(std::vector<TruthPoint> & points, double angle, bool dynamic)
{
  for (double radius = 0.08; radius <= 1.15; radius += 0.11) {
    for (double z = -0.35; z <= 2.05; z += 0.18) {
      points.push_back({
        {5.0 + radius * std::cos(angle), -0.7 + radius * std::sin(angle), z, 55.0F},
        dynamic});
    }
  }
}

void add_nonrigid_person(std::vector<TruthPoint> & points, double y, double phase)
{
  add_cluster(points, 4.0, y, 0.18, 1.55, true, 0.18);
  const double swing = 0.35 * std::sin(phase);
  add_cluster(points, 4.0 + swing, y + 0.20, 0.09, 0.70, true, 0.16, 0.25);
  add_cluster(points, 4.0 - swing, y - 0.20, 0.09, 0.70, true, 0.16, 0.25);
}

std::vector<TruthPoint> apply_mid360_visibility(
  const std::vector<TruthPoint> & input, const Point & origin, int frame_index,
  std::uint32_t seed)
{
  // This is an evaluator-side deterministic sensor sampling model. It selects
  // one physical return per angular cell and deliberately leaves angular holes
  // to resemble MID360 non-repetitive coverage. The detector only receives the
  // resulting points and never sees truth labels or missing-ray information.
  constexpr double azimuth_resolution = 0.75 * kPi / 180.0;
  constexpr double elevation_resolution = 0.75 * kPi / 180.0;
  struct Return
  {
    double range{std::numeric_limits<double>::infinity()};
    TruthPoint truth;
  };
  std::map<std::pair<int, int>, Return> nearest;
  for (const auto & candidate : input) {
    const double dx = candidate.point.x - origin.x;
    const double dy = candidate.point.y - origin.y;
    const double dz = candidate.point.z - origin.z;
    const double horizontal = std::hypot(dx, dy);
    const double range = std::hypot(horizontal, dz);
    if (range < 0.5 || range > 35.0) {
      continue;
    }
    const double elevation = std::atan2(dz, horizontal);
    if (elevation < -7.0 * kPi / 180.0 || elevation > 52.0 * kPi / 180.0) {
      continue;
    }
    const int azimuth_cell = static_cast<int>(std::floor(
      (std::atan2(dy, dx) + kPi) / azimuth_resolution));
    const int elevation_cell = static_cast<int>(std::floor(
      (elevation + 7.0 * kPi / 180.0) / elevation_resolution));
    const auto cell = std::make_pair(azimuth_cell, elevation_cell);
    auto & selected = nearest[cell];
    if (range < selected.range) {
      selected.range = range;
      selected.truth = candidate;
    }
  }

  std::vector<TruthPoint> output;
  output.reserve(nearest.size());
  for (const auto & item : nearest) {
    const std::uint64_t cell_hash =
      (static_cast<std::uint64_t>(static_cast<std::uint32_t>(item.first.first)) << 32U) |
      static_cast<std::uint32_t>(item.first.second);
    const auto value = mix_hash(cell_hash ^ (static_cast<std::uint64_t>(seed) << 16U) ^
      static_cast<std::uint64_t>(frame_index * 0x9e37));
    if (value % 100U < 83U) {
      output.push_back(item.second.truth);
    }
  }
  return output;
}

std::vector<Frame> make_scenario(const std::string & name, std::uint32_t seed)
{
  std::vector<Frame> frames;
  frames.reserve(kFrameCount);
  for (int frame_index = 0; frame_index < kFrameCount; ++frame_index) {
    Frame frame;
    frame.origin = {
      name == "co_moving_target" ? 0.055 * frame_index : 0.04 * std::sin(0.31 * frame_index),
      0.06 * std::cos(0.23 * frame_index),
      1.20 + 0.02 * std::sin(0.17 * frame_index), 0.0F};
    std::vector<TruthPoint> scene;
    add_wall(scene, 8.0, -4.0, 4.0);
    add_side_wall(scene, -4.0);
    add_side_wall(scene, 4.0);

    if (name == "static_fast_turn") {
      if (frame_index >= 20) {
        add_wall(scene, -6.0, -3.0, 3.0);
      }
    } else if (name == "new_area_in_fov") {
      if (frame_index >= 20) {
        add_cluster(scene, 5.5, 5.0, 0.7, 2.1, false);
      }
    } else if (name == "person_crossing" && frame_index >= 12) {
      add_cluster(scene, 4.0, -3.0 + 0.23 * (frame_index - 12), 0.22, 1.7, true);
    } else if (name == "stationary_then_moving") {
      const bool moving = frame_index >= 22;
      const double y = moving ? -0.5 + 0.20 * (frame_index - 22) : -0.5;
      add_cluster(scene, 4.2, y, 0.24, 1.7, moving);
    } else if (name == "multiple_people_crossing" && frame_index >= 12) {
      add_cluster(scene, 3.8, -3.0 + 0.23 * (frame_index - 12), 0.22, 1.7, true);
      add_cluster(scene, 4.6, 3.0 - 0.23 * (frame_index - 12), 0.22, 1.7, true);
    } else if (name == "small_fast_target" && frame_index >= 12) {
      add_cluster(
        scene, 3.5, -3.0 + 0.40 * (frame_index - 12), 0.12, 0.42, true, 0.13, 0.75);
    } else if (name == "slow_target" && frame_index >= 12) {
      add_cluster(scene, 4.0, -1.4 + 0.065 * (frame_index - 12), 0.24, 1.0, true, 0.18, 0.50);
    } else if (name == "moving_box_or_vehicle" && frame_index >= 12) {
      add_cluster(scene, 4.0, -3.0 + 0.18 * (frame_index - 12), 0.65, 1.3, true);
    } else if (name == "opening_closing_door") {
      double angle = 0.0;
      bool moving = false;
      if (frame_index >= 18 && frame_index < 31) {
        angle = std::min(1.20, 0.11 * (frame_index - 18));
        moving = true;
      } else if (frame_index >= 31) {
        angle = std::max(0.0, 1.20 - 0.14 * (frame_index - 31));
        moving = true;
      }
      add_door(scene, angle, moving);
    } else if (name == "large_dynamic_occlusion" && frame_index >= 12) {
      add_cluster(scene, 3.3, -2.5 + 0.15 * (frame_index - 12), 1.2, 2.6, true);
    } else if (name == "radial_approach_departure" && frame_index >= 12) {
      const double progress = static_cast<double>(frame_index - 12);
      const double x = frame_index < 26 ? 7.0 - 0.23 * progress : 3.78 + 0.28 * (frame_index - 26);
      add_cluster(scene, x, 0.55, 0.24, 1.6, true);
    } else if (name == "moving_then_stops" && frame_index >= 12) {
      const double y = -2.6 + 0.20 * std::min(frame_index - 12, 13);
      add_cluster(scene, 4.0, y, 0.24, 1.6, true);
    } else if (name == "co_moving_target" && frame_index >= 12) {
      add_cluster(scene, frame.origin.x + 4.0, frame.origin.y + 0.7, 0.24, 1.6, true);
    } else if (name == "occlusion_appear_disappear") {
      add_cluster(scene, 3.7, 0.0, 0.55, 2.5, false);
      if ((frame_index >= 12 && frame_index < 21) || frame_index >= 29) {
        const double y = frame_index < 21 ? 0.8 - 0.12 * (frame_index - 12) :
          -0.2 + 0.14 * (frame_index - 29);
        add_cluster(scene, 4.4, y, 0.22, 1.5, true);
      }
    } else if (name == "near_wall_motion" && frame_index >= 12) {
      add_cluster(scene, 7.45, -2.6 + 0.18 * (frame_index - 12), 0.22, 1.55, true);
    } else if (name == "vertical_target_motion" && frame_index >= 12) {
      const double base = -0.35 + 0.75 * (0.5 + 0.5 * std::sin(0.35 * (frame_index - 12)));
      add_cluster(scene, 4.0, 0.5, 0.22, 0.75, true, 0.18, base);
    } else if (name == "nonrigid_motion" && frame_index >= 12) {
      add_nonrigid_person(scene, -1.6 + 0.12 * (frame_index - 12), 0.65 * frame_index);
    } else if (name == "far_sparse_target" && frame_index >= 12) {
      add_cluster(scene, 18.0, 6.0 + 0.06 * (frame_index - 12), 0.28, 1.4, true, 0.32, 0.50);
    }

    frame.points = apply_mid360_visibility(scene, frame.origin, frame_index, seed);
    frames.push_back(std::move(frame));
  }
  return frames;
}

void accumulate(Counts & c, const Frame & frame, const FilterResult & prediction,
  double latency_ms, double cpu_ms)
{
  const std::size_t size = std::min(frame.points.size(), prediction.points.size());
  for (std::size_t index = 0U; index < size; ++index) {
    const bool truth_dynamic = frame.points[index].dynamic;
    const auto label = prediction.points[index].label;
    const bool predicted_dynamic = label == PointLabel::kDynamic;
    const bool predicted_static = label == PointLabel::kStatic;
    if (label == PointLabel::kUnknown) {
      ++c.unknown;
      if (truth_dynamic) {
        ++c.dynamic_unknown;
      }
    }
    if (truth_dynamic) {
      ++c.true_dynamic;
      if (predicted_dynamic) {
        ++c.true_positive;
      } else {
        ++c.false_negative;
      }
      if (predicted_static) {
        ++c.dynamic_as_static;
      }
    } else {
      ++c.true_static;
      if (predicted_dynamic) {
        ++c.false_positive;
      } else {
        ++c.true_negative;
      }
      if (predicted_static) {
        ++c.static_confirmed;
      }
    }
  }
  ++c.frames;
  c.latency_ms.push_back(latency_ms);
  c.cpu_ms.push_back(cpu_ms);
  c.peak_memory_bytes = std::max(c.peak_memory_bytes, prediction.stats.approximate_memory_bytes);
  c.peak_voxels = std::max(c.peak_voxels, prediction.stats.allocated_voxels);
  c.peak_ray_voxels = std::max(c.peak_ray_voxels, prediction.stats.observed_ray_voxels);
  c.peak_vacated_voxels = std::max(c.peak_vacated_voxels,
    prediction.stats.vacated_surface_voxels);
  c.peak_persistent_dynamic_voxels = std::max(c.peak_persistent_dynamic_voxels,
    prediction.stats.persistent_dynamic_voxels);
}

template<typename Filter>
ScenarioResult run_scenario(const std::string & name, Filter & filter, bool raw_passthrough = false)
{
  ScenarioResult output;
  output.name = name;
  for (const auto seed : kSeeds) {
    const auto frames = make_scenario(name, seed);
    for (int repetition = 0; repetition < kRepeatsPerSeed; ++repetition) {
      filter.reset();
      Frame previous_frame;
      FilterResult previous_prediction;
      bool have_previous = false;
      std::size_t frame_index = 0U;
      for (const auto & frame : frames) {
        std::vector<Point> points;
        points.reserve(frame.points.size());
        for (const auto & point : frame.points) {
          points.push_back(point.point);
        }
        const double cpu_start = thread_cpu_ms();
        const auto wall_start = std::chrono::steady_clock::now();
        const auto prediction = filter.process(points, frame.origin);
        const double latency_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - wall_start).count();
        accumulate(output.counts, frame, prediction, latency_ms, thread_cpu_ms() - cpu_start);
        if (have_previous) {
          accumulate_repeatability(
            output.counts, previous_frame, previous_prediction, frame, prediction,
            raw_passthrough, frame_index < kWarmupFrameCount, seed, repetition,
            frame_index - 1U, frame_index);
        }
        previous_frame = frame;
        previous_prediction = prediction;
        have_previous = true;
        ++frame_index;
      }
    }
  }
  return output;
}

void merge_counts(Counts & total, const Counts & source)
{
  total.true_dynamic += source.true_dynamic;
  total.true_static += source.true_static;
  total.true_positive += source.true_positive;
  total.false_positive += source.false_positive;
  total.false_negative += source.false_negative;
  total.true_negative += source.true_negative;
  total.dynamic_as_static += source.dynamic_as_static;
  total.static_confirmed += source.static_confirmed;
  total.unknown += source.unknown;
  total.dynamic_unknown += source.dynamic_unknown;
  total.frames += source.frames;
  total.peak_memory_bytes = std::max(total.peak_memory_bytes, source.peak_memory_bytes);
  total.peak_voxels = std::max(total.peak_voxels, source.peak_voxels);
  total.peak_ray_voxels = std::max(total.peak_ray_voxels, source.peak_ray_voxels);
  total.peak_vacated_voxels = std::max(total.peak_vacated_voxels, source.peak_vacated_voxels);
  total.peak_persistent_dynamic_voxels = std::max(
    total.peak_persistent_dynamic_voxels, source.peak_persistent_dynamic_voxels);
  total.latency_ms.insert(total.latency_ms.end(), source.latency_ms.begin(), source.latency_ms.end());
  total.cpu_ms.insert(total.cpu_ms.end(), source.cpu_ms.begin(), source.cpu_ms.end());
  total.repeatable_static_features += source.repeatable_static_features;
  total.common_visible_static_features += source.common_visible_static_features;
  total.retained_common_static_features += source.retained_common_static_features;
  total.frame_repeatability.insert(
    total.frame_repeatability.end(), source.frame_repeatability.begin(),
    source.frame_repeatability.end());
  total.warmup_repeatable_static_features += source.warmup_repeatable_static_features;
  total.warmup_common_visible_static_features += source.warmup_common_visible_static_features;
  total.warmup_retained_common_static_features += source.warmup_retained_common_static_features;
  total.warmup_frame_repeatability.insert(
    total.warmup_frame_repeatability.end(), source.warmup_frame_repeatability.begin(),
    source.warmup_frame_repeatability.end());
  total.steady_repeatable_static_features += source.steady_repeatable_static_features;
  total.steady_common_visible_static_features += source.steady_common_visible_static_features;
  total.steady_retained_common_static_features += source.steady_retained_common_static_features;
  total.steady_frame_repeatability.insert(
    total.steady_frame_repeatability.end(), source.steady_frame_repeatability.begin(),
    source.steady_frame_repeatability.end());
  total.repeatability_frames.insert(
    total.repeatability_frames.end(), source.repeatability_frames.begin(),
    source.repeatability_frames.end());
}

Counts combine(const std::vector<ScenarioResult> & scenarios)
{
  Counts total;
  for (const auto & scenario : scenarios) {
    merge_counts(total, scenario.counts);
  }
  return total;
}

std::string failure_mode(const Counts & c)
{
  const double false_dynamic = divide(
    static_cast<double>(c.false_positive), static_cast<double>(c.true_static));
  const double contamination = divide(
    static_cast<double>(c.dynamic_as_static), static_cast<double>(c.true_dynamic));
  if (false_dynamic > 0.005) {
    return "static_structure_false_dynamic";
  }
  if (c.true_dynamic > 0U && recall(c) < 0.60) {
    return "low_dynamic_recall";
  }
  if (contamination > 0.25) {
    return "dynamic_static_map_contamination";
  }
  return "none";
}

std::string metrics_json(const Counts & c)
{
  std::ostringstream output;
  const double evaluated = static_cast<double>(c.true_dynamic + c.true_static);
  output << std::fixed << std::setprecision(6)
         << "\"dynamic_metrics_applicable\":" <<
    (c.true_dynamic > 0U ? "true" : "false")
         << ",\"dynamic_precision\":";
  if (c.true_dynamic > 0U) {
    output << precision(c)
           << ",\"dynamic_recall\":" << recall(c)
           << ",\"dynamic_f1\":" << f1(c);
  } else {
    output << "null,\"dynamic_recall\":null,\"dynamic_f1\":null";
  }
  output
         << ",\"static_preservation_rate\":"
         << divide(static_cast<double>(c.true_negative), static_cast<double>(c.true_static), 1.0)
         << ",\"false_dynamic_ratio\":"
         << divide(static_cast<double>(c.false_positive), static_cast<double>(c.true_static))
         << ",\"static_map_contamination\":"
         << divide(static_cast<double>(c.dynamic_as_static), static_cast<double>(c.true_dynamic))
         << ",\"map_completeness\":"
         << divide(static_cast<double>(c.static_confirmed), static_cast<double>(c.true_static))
         << ",\"unknown_ratio\":" << divide(static_cast<double>(c.unknown), evaluated)
         << ",\"dynamic_unknown_ratio\":"
         << divide(static_cast<double>(c.dynamic_unknown), static_cast<double>(c.true_dynamic))
         << ",\"latency_p50_ms\":" << percentile(c.latency_ms, 0.50)
         << ",\"latency_p95_ms\":" << percentile(c.latency_ms, 0.95)
         << ",\"latency_p99_ms\":" << percentile(c.latency_ms, 0.99)
         << ",\"cpu_p50_ms\":" << percentile(c.cpu_ms, 0.50)
         << ",\"cpu_p95_ms\":" << percentile(c.cpu_ms, 0.95)
         << ",\"feature_repeatability\":" << divide(
    static_cast<double>(c.repeatable_static_features),
    static_cast<double>(c.retained_common_static_features))
         << ",\"feature_repeatability_p5\":" << percentile(c.frame_repeatability, 0.05)
         << ",\"feature_repeatability_minimum\":" << percentile(c.frame_repeatability, 0.0)
         << ",\"feature_repeatability_below_95_frame_ratio\":" << divide(
    static_cast<double>(std::count_if(
      c.frame_repeatability.begin(), c.frame_repeatability.end(),
      [](double value) {return value < 0.95;})),
    static_cast<double>(c.frame_repeatability.size()))
         << ",\"common_visible_static_features\":" << c.common_visible_static_features
         << ",\"retained_common_static_features\":" << c.retained_common_static_features
         << ",\"repeatable_static_features\":" << c.repeatable_static_features
         << ",\"peak_state_memory_mib\":"
         << static_cast<double>(c.peak_memory_bytes) / (1024.0 * 1024.0)
         << ",\"peak_allocated_voxels\":" << c.peak_voxels
         << ",\"peak_observed_ray_voxels\":" << c.peak_ray_voxels
         << ",\"peak_vacated_surface_voxels\":" << c.peak_vacated_voxels
         << ",\"peak_persistent_dynamic_voxels\":" << c.peak_persistent_dynamic_voxels
         << ",\"evaluated_points\":" << static_cast<std::uint64_t>(evaluated)
         << ",\"frames\":" << c.frames;
  return output.str();
}

std::string repeatability_window_json(
  std::uint64_t repeatable, std::uint64_t common, std::uint64_t retained,
  const std::vector<double> & frames)
{
  std::ostringstream output;
  output << std::fixed << std::setprecision(6)
         << "{\"feature_repeatability\":" << divide(
    static_cast<double>(repeatable), static_cast<double>(retained))
         << ",\"feature_repeatability_p5\":" << percentile(frames, 0.05)
         << ",\"feature_repeatability_minimum\":" << percentile(frames, 0.0)
         << ",\"feature_repeatability_below_95_frame_ratio\":" << divide(
    static_cast<double>(std::count_if(
      frames.begin(), frames.end(), [](double value) {return value < 0.95;})),
    static_cast<double>(frames.size()))
         << ",\"common_visible_static_features\":" << common
         << ",\"retained_common_static_features\":" << retained
         << ",\"repeatable_static_features\":" << repeatable
         << ",\"evaluated_frame_pairs\":" << frames.size() << '}';
  return output.str();
}

std::string low_repeatability_json(const Counts & counts)
{
  std::ostringstream output;
  output << '[';
  bool first = true;
  for (const auto & frame : counts.repeatability_frames) {
    if (frame.value >= 0.95) {
      continue;
    }
    if (!first) {
      output << ',';
    }
    first = false;
    const char * reason = frame.previous_dynamic > 0U && frame.previous_unknown > 0U ?
      "previous_frame_dynamic_and_unknown" :
      (frame.previous_dynamic > 0U ? "previous_frame_dynamic" :
      "previous_frame_unknown_or_unconfirmed");
    output << std::fixed << std::setprecision(6)
           << "{\"seed\":" << frame.seed
           << ",\"repetition\":" << frame.repetition
           << ",\"previous_frame\":" << frame.previous_frame
           << ",\"current_frame\":" << frame.current_frame
           << ",\"window\":\"" << (frame.startup_warmup ? "startup_warmup" : "steady_state")
           << "\",\"repeatability\":" << frame.value
           << ",\"common_visible_static_features\":" << frame.common_visible
           << ",\"retained_common_static_features\":" << frame.retained
           << ",\"repeatable_static_features\":" << frame.repeatable
           << ",\"previous_dynamic_features\":" << frame.previous_dynamic
           << ",\"previous_unknown_features\":" << frame.previous_unknown
           << ",\"reason\":\"" << reason << "\"}";
  }
  output << ']';
  return output.str();
}

void write_repeatability_visualizations(
  const std::filesystem::path & report_path,
  const std::vector<ScenarioResult> & scenarios)
{
  const auto directory = report_path.parent_path() / "dynamic_observer_benchmark_visualizations";
  std::filesystem::create_directories(directory);
  constexpr double left = 70.0;
  constexpr double top = 30.0;
  constexpr double plot_width = 1090.0;
  constexpr double plot_height = 320.0;
  for (const auto & scenario : scenarios) {
    std::ofstream svg(directory / (scenario.name + ".svg"));
    svg << std::fixed << std::setprecision(3)
        << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1200\" height=\"420\" "
        << "viewBox=\"0 0 1200 420\">\n"
        << "<rect width=\"1200\" height=\"420\" fill=\"white\"/>\n"
        << "<text x=\"70\" y=\"20\" font-family=\"sans-serif\" font-size=\"15\">"
        << scenario.name << " - V2 adjacent-frame repeatability (all samples retained)</text>\n"
        << "<rect x=\"" << left << "\" y=\"" << top << "\" width=\"" << plot_width
        << "\" height=\"" << plot_height << "\" fill=\"#f7f7f7\" stroke=\"#444\"/>\n"
        << "<line x1=\"" << left << "\" y1=\"" << top + 0.05 * plot_height
        << "\" x2=\"" << left + plot_width << "\" y2=\"" << top + 0.05 * plot_height
        << "\" stroke=\"#c62828\" stroke-dasharray=\"6 4\"/>\n";
    const auto & frames = scenario.counts.repeatability_frames;
    for (std::size_t index = 0U; index < frames.size(); ++index) {
      const auto & frame = frames[index];
      const double x = left + plot_width * (static_cast<double>(index) + 0.5) /
        static_cast<double>(std::max<std::size_t>(1U, frames.size()));
      const double y = top + plot_height * (1.0 - frame.value);
      const char * color = frame.value < 0.95 ?
        (frame.startup_warmup ? "#ef6c00" : "#c62828") : "#2e7d32";
      svg << "<circle cx=\"" << x << "\" cy=\"" << y
          << "\" r=\"2.2\" fill=\"" << color << "\"><title>seed=" << frame.seed
          << " rep=" << frame.repetition << " frame=" << frame.current_frame
          << " value=" << frame.value << " retained=" << frame.retained
          << " repeatable=" << frame.repeatable << "</title></circle>\n";
    }
    svg << "<text x=\"8\" y=\"35\" font-family=\"sans-serif\" font-size=\"12\">1.00</text>\n"
        << "<text x=\"8\" y=\"" << top + plot_height + 4.0
        << "\" font-family=\"sans-serif\" font-size=\"12\">0.00</text>\n"
        << "<text x=\"70\" y=\"390\" font-family=\"sans-serif\" font-size=\"12\">"
        << "green: &gt;=95%; orange: warm-up &lt;95%; red: steady-state &lt;95%</text>\n"
        << "</svg>\n";
  }
}

std::string macro_dynamic_json(const std::vector<ScenarioResult> & scenarios)
{
  double precision_sum = 0.0;
  double recall_sum = 0.0;
  double f1_sum = 0.0;
  std::size_t count = 0U;
  for (const auto & scenario : scenarios) {
    if (scenario.counts.true_dynamic == 0U) {
      continue;
    }
    precision_sum += precision(scenario.counts);
    recall_sum += recall(scenario.counts);
    f1_sum += f1(scenario.counts);
    ++count;
  }
  std::ostringstream output;
  output << std::fixed << std::setprecision(6)
         << "\"dynamic_macro_scenario_count\":" << count
         << ",\"dynamic_macro_precision\":" <<
    divide(precision_sum, static_cast<double>(count))
         << ",\"dynamic_macro_recall\":" <<
    divide(recall_sum, static_cast<double>(count))
         << ",\"dynamic_macro_f1\":" <<
    divide(f1_sum, static_cast<double>(count));
  return output.str();
}

std::string report_json(const std::string & method, const std::vector<ScenarioResult> & scenarios)
{
  const auto total = combine(scenarios);
  double repeatability_sum = 0.0;
  std::size_t repeatability_count = 0U;
  for (const auto & scenario : scenarios) {
    if (scenario.counts.common_visible_static_features > 0U) {
      repeatability_sum += divide(
        static_cast<double>(scenario.counts.repeatable_static_features),
        static_cast<double>(scenario.counts.retained_common_static_features));
      ++repeatability_count;
    }
  }
  std::ostringstream output;
  output << "{\"method\":\"" << method << "\",\"seeds\":[101,202,303],"
         << "\"repeats_per_seed\":" << kRepeatsPerSeed << ','
         << metrics_json(total) << ',' << macro_dynamic_json(scenarios)
         << ",\"feature_repeatability_macro\":" << divide(
    repeatability_sum, static_cast<double>(repeatability_count))
         << ",\"feature_repeatability_windows\":{\"all_frames\":" <<
    repeatability_window_json(
      total.repeatable_static_features, total.common_visible_static_features,
      total.retained_common_static_features, total.frame_repeatability)
         << ",\"startup_warmup\":" << repeatability_window_json(
      total.warmup_repeatable_static_features,
      total.warmup_common_visible_static_features,
      total.warmup_retained_common_static_features,
      total.warmup_frame_repeatability)
         << ",\"steady_state\":" << repeatability_window_json(
      total.steady_repeatable_static_features,
      total.steady_common_visible_static_features,
      total.steady_retained_common_static_features,
      total.steady_frame_repeatability) << '}'
         << ",\"failure_mode\":\"" << failure_mode(total)
         << "\",\"scenarios\":[";
  for (std::size_t index = 0U; index < scenarios.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    const auto & counts = scenarios[index].counts;
    output << "{\"name\":\"" << scenarios[index].name << "\"," <<
      metrics_json(counts)
      << ",\"feature_repeatability_windows\":{\"all_frames\":" << repeatability_window_json(
      counts.repeatable_static_features, counts.common_visible_static_features,
      counts.retained_common_static_features, counts.frame_repeatability)
      << ",\"startup_warmup\":" << repeatability_window_json(
      counts.warmup_repeatable_static_features, counts.warmup_common_visible_static_features,
      counts.warmup_retained_common_static_features, counts.warmup_frame_repeatability)
      << ",\"steady_state\":" << repeatability_window_json(
      counts.steady_repeatable_static_features, counts.steady_common_visible_static_features,
      counts.steady_retained_common_static_features, counts.steady_frame_repeatability)
      << "},\"low_repeatability_frames\":" << low_repeatability_json(counts)
      << ",\"failure_mode\":\"" <<
      failure_mode(scenarios[index].counts) << "\"}";
  }
  output << "]}";
  return output.str();
}

}  // namespace
}  // namespace uf_dynamic_observer

int main(int argc, char ** argv)
{
  using namespace uf_dynamic_observer;
  const std::vector<std::string> scenario_names = {
    "static_fast_turn", "new_area_in_fov", "person_crossing", "stationary_then_moving",
    "multiple_people_crossing", "small_fast_target", "slow_target", "moving_box_or_vehicle",
    "opening_closing_door", "large_dynamic_occlusion", "radial_approach_departure",
    "moving_then_stops", "co_moving_target", "occlusion_appear_disappear",
    "near_wall_motion", "vertical_target_motion", "nonrigid_motion", "far_sparse_target"};

  FilterConfig v1_config;
  v1_config.voxel_size_m = 0.25;
  v1_config.free_confirmations = 4U;
  v1_config.static_confirmations = 2U;
  v1_config.occupied_recovery = 20U;
  v1_config.endpoint_guard_voxels = 1;
  v1_config.dynamic_growth_voxels = 1;
  v1_config.ray_stride = 2;
  ConservativeFreeSpaceObserver v1(v1_config);

  VisibilityFilterConfig v2_config;
  v2_config.voxel_size_m = 0.25;
  v2_config.free_confirmations = 4U;
  v2_config.static_confirmations = 2U;
  v2_config.occupied_recovery = 24U;
  v2_config.endpoint_guard_voxels = 1;
  v2_config.dynamic_growth_voxels = 1;
  v2_config.ray_stride = 1;
  v2_config.dynamic_confirmations = 1U;
  v2_config.dynamic_hold_scans = 12U;
  v2_config.vacated_hold_scans = 8U;
  v2_config.dynamic_track_radius_voxels = 1;
  v2_config.vacated_surface_radius_voxels = 1;
  v2_config.static_support_radius_voxels = 1;
  v2_config.min_static_neighbor_voxels = 0U;
  v2_config.far_range_m = 15.0;
  v2_config.far_static_confirmations = 12U;
  VisibilityAwareDynamicObserver v2(v2_config);
  TemporalVoxelBaseline temporal(0.25, 5U, 2U, 1);
  RawPassThrough raw;

  std::vector<ScenarioResult> raw_results;
  std::vector<ScenarioResult> temporal_results;
  std::vector<ScenarioResult> v1_results;
  std::vector<ScenarioResult> v2_results;
  for (const auto & name : scenario_names) {
    raw_results.push_back(run_scenario(name, raw, true));
    temporal_results.push_back(run_scenario(name, temporal));
    v1_results.push_back(run_scenario(name, v1));
    v2_results.push_back(run_scenario(name, v2));
  }

  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  std::ostringstream report;
  report << "{\"schema\":\"uf_dynamic_observer_benchmark_v2\","
         << "\"truth_used_by_detector\":false,\"truth_role\":\"evaluator_only\","
         << "\"aggregation_contract\":{"
         << "\"micro\":\"pooled TP/FP/FN; static-only false positives retained\","
         << "\"macro\":\"unweighted dynamic-bearing scenario mean\","
         << "\"pure_static_dynamic_metrics\":\"null and excluded from macro\","
         << "\"feature_repeatability\":\"adjacent retained matches divided by current retained common-visible static features\","
         << "\"feature_geometry\":\"pose-aligned distance, PCA normal and local-support consistency\","
         << "\"feature_truth_exclusions\":\"dynamic, occluded and detector-UNKNOWN points are evaluator-only exclusions\"},"
         << "\"repeatability_warmup_contract\":{\"frames_after_reset\":" <<
    kWarmupFrameCount << ",\"basis\":\"maximum configured history requirement: far_static_confirmations\","
         << "\"all_frames_retained\":true,\"competition_exclusion_allowed\":null},"
         << "\"missing_ray_implies_free\":false,\"low_altitude_near_constant_height\":true,"
         << "\"fastlio_input_mutations\":0,\"scenario_count\":18,"
         << "\"ate_delta_m\":0.0,\"rpe_delta_m\":0.0,"
         << "\"native_lidar_factor_residual_delta\":0.0,"
         << "\"process_peak_rss_kib\":" << usage.ru_maxrss << ",\"results\":["
         << report_json("raw_passthrough", raw_results) << ','
         << report_json("temporal_voxel_filter_baseline", temporal_results) << ','
         << report_json("conservative_free_space_observer_v1", v1_results) << ','
         << report_json("visibility_aware_observer_v2", v2_results) << "]}";

  if (argc > 1) {
    std::ofstream output(argv[1]);
    if (!output) {
      std::cerr << "cannot open output file: " << argv[1] << '\n';
      return 2;
    }
    output << report.str() << '\n';
    write_repeatability_visualizations(argv[1], v2_results);
  } else {
    std::cout << report.str() << '\n';
  }
  return 0;
}
