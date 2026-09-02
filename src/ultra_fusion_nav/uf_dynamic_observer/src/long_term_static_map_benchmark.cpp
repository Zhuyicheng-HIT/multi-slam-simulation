#include "uf_dynamic_observer/conservative_free_space.hpp"
#include "uf_dynamic_observer/long_term_static_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace uf_dynamic_observer
{
namespace
{

constexpr double kVoxel = 0.25;
constexpr double kPi = 3.14159265358979323846;

using VoxelSet = std::unordered_set<VoxelKey, VoxelKeyHash>;

struct TruthPoint
{
  Point point;
  bool dynamic{false};
};

struct MapMetrics
{
  double contamination{0.0};
  double ghost_ratio{0.0};
  double completeness{0.0};
  double static_preservation{0.0};
  double false_removal{0.0};
  double relocalization_contamination{0.0};
  double relocalization_overlap{0.0};
  std::size_t voxels{0U};
};

struct RunMetrics
{
  std::string scenario;
  int seed{0};
  MapMetrics raw;
  MapMetrics clean;
  MapMetrics refined;
  double convergence_s{0.0};
  double admission_delay_s{0.0};
  double promoted_static_ratio{0.0};
  double permanent_rejection_ratio{0.0};
  std::uint64_t removed_ghost_voxels{0U};
  double latency_p50_ms{0.0};
  double latency_p95_ms{0.0};
  double latency_p99_ms{0.0};
  double cpu_percent{0.0};
  double memory_mib{0.0};
};

struct Aggregate
{
  std::string scenario;
  int runs{0};
  RunMetrics mean;
};

VoxelKey voxel(const Point & point)
{
  return {
    static_cast<std::int32_t>(std::floor(point.x / kVoxel)),
    static_cast<std::int32_t>(std::floor(point.y / kVoxel)),
    static_cast<std::int32_t>(std::floor(point.z / kVoxel))};
}

Point sensor_pose(int frame, int seed)
{
  const double angle = 2.0 * kPi * static_cast<double>(frame) / 160.0 + seed * 0.013;
  return {2.6 * std::cos(angle), 2.2 * std::sin(angle), 1.25, 0.0F};
}

void add_box(
  std::vector<TruthPoint> & points, double x, double y, double half_width,
  double height, bool dynamic, double spacing = 0.25)
{
  const int count = std::max(1, static_cast<int>(std::round(2.0 * half_width / spacing)));
  const int height_count = std::max(1, static_cast<int>(std::round(height / spacing)));
  for (int ix = 0; ix <= count; ++ix) {
    for (int iy = 0; iy <= count; ++iy) {
      if (ix != 0 && ix != count && iy != 0 && iy != count) {
        continue;
      }
      for (int iz = 0; iz <= height_count; ++iz) {
        points.push_back({{
          x - half_width + static_cast<double>(ix) * spacing,
          y - half_width + static_cast<double>(iy) * spacing,
          0.1 + static_cast<double>(iz) * spacing,
          dynamic ? 80.0F : 30.0F}, dynamic});
      }
    }
  }
}

std::vector<TruthPoint> static_environment()
{
  std::vector<TruthPoint> points;
  for (int index = -7; index <= 7; ++index) {
    const double coordinate = static_cast<double>(index);
    for (double z : {0.25, 0.75, 1.25, 1.75, 2.25, 2.75}) {
      points.push_back({{coordinate, -7.0, z, 25.0F}, false});
      points.push_back({{coordinate, 7.0, z, 25.0F}, false});
      points.push_back({{-7.0, coordinate, z, 25.0F}, false});
      points.push_back({{7.0, coordinate, z, 25.0F}, false});
    }
  }
  for (int x = -5; x <= 5; x += 2) {
    for (int y = -5; y <= 5; y += 2) {
      points.push_back({{static_cast<double>(x), static_cast<double>(y), 0.0, 20.0F}, false});
    }
  }
  add_box(points, -3.5, 1.8, 0.45, 2.2, false, 0.3);
  add_box(points, 3.5, -1.8, 0.45, 2.2, false, 0.3);
  return points;
}

void dynamic_scene(
  const std::string & scenario, int frame, std::vector<TruthPoint> & points)
{
  const int t = frame - 70;
  if (scenario == "person_stays_then_leaves") {
    if (frame >= 70 && frame < 175) {
      add_box(points, 3.0, 0.0, 0.28, 1.75, true, 0.22);
    }
  } else if (scenario == "repeated_passes") {
    if (frame >= 70 && frame < 205) {
      add_box(points, 2.0, 3.0 * std::sin(t * 0.12), 0.28, 1.7, true, 0.22);
    }
  } else if (scenario == "multiple_crossing") {
    if (frame >= 70 && frame < 185) {
      add_box(points, -2.8 + 0.05 * t, -1.2, 0.27, 1.7, true, 0.22);
      add_box(points, 1.2, 2.8 - 0.05 * t, 0.27, 1.7, true, 0.22);
    }
  } else if (scenario == "opening_closing_door") {
    const double angle = frame < 70 ? 0.0 :
      (frame < 125 ? 1.25 * (frame - 70) / 55.0 :
      (frame < 180 ? 1.25 * (180 - frame) / 55.0 : 0.0));
    const bool moving = frame >= 70 && frame < 180;
    for (int radius_index = 1; radius_index <= 8; ++radius_index) {
      const double radius = radius_index * 0.18;
      for (int z_index = 0; z_index <= 8; ++z_index) {
        points.push_back({{
          5.8 - radius * std::cos(angle), -1.0 + radius * std::sin(angle),
          0.1 + z_index * 0.25, moving ? 80.0F : 30.0F}, moving});
      }
    }
  } else if (scenario == "occlusion_reappear") {
    add_box(points, 1.5, 0.0, 0.6, 2.5, false, 0.3);
    if ((frame >= 70 && frame < 120) || (frame >= 155 && frame < 200)) {
      add_box(points, 2.7, -1.5 + 0.035 * (frame % 50), 0.27, 1.6, true, 0.22);
    }
  } else if (scenario == "small_fast_target") {
    if (frame >= 70 && frame < 155) {
      add_box(points, -3.5 + 0.09 * t, 2.0, 0.14, 0.5, true, 0.14);
    }
  } else if (scenario == "slow_target") {
    if (frame >= 70 && frame < 210) {
      add_box(points, -1.5 + 0.012 * t, -2.2, 0.30, 1.2, true, 0.25);
    }
  } else if (scenario == "near_wall_motion") {
    if (frame >= 70 && frame < 190) {
      add_box(points, 6.55, -4.5 + 0.075 * t, 0.24, 1.6, true, 0.22);
    }
  } else if (scenario == "large_dynamic_occlusion") {
    if (frame >= 70 && frame < 180) {
      add_box(points, 1.0, -3.0 + 0.055 * t, 1.1, 2.5, true, 0.3);
    }
  } else if (scenario == "far_sparse_target") {
    if (frame >= 70 && frame < 190) {
      add_box(points, 16.0, -1.0 + 0.02 * t, 0.32, 1.4, true, 0.32);
    }
  } else if (scenario == "stopped_then_moves") {
    if (frame >= 70 && frame < 210) {
      const double y = frame < 105 ? -3.0 + 0.08 * t :
        (frame < 175 ? -0.2 : -0.2 + 0.10 * (frame - 175));
      add_box(points, -2.0, y, 0.28, 1.7, true, 0.22);
    }
  }
}

std::vector<TruthPoint> measured_returns(
  const std::vector<TruthPoint> & world, const Point & origin, int frame, int seed)
{
  struct Return
  {
    double range;
    TruthPoint point;
  };
  std::unordered_map<std::int64_t, Return> nearest;
  for (std::size_t index = 0U; index < world.size(); ++index) {
    const auto & candidate = world[index];
    const double dx = candidate.point.x - origin.x;
    const double dy = candidate.point.y - origin.y;
    const double dz = candidate.point.z - origin.z;
    const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (range < 0.5 || range > 35.0) {
      continue;
    }
    const double azimuth = std::atan2(dy, dx);
    const double elevation = std::atan2(dz, std::hypot(dx, dy));
    const int azimuth_bin = static_cast<int>(std::llround(azimuth / 0.018));
    const int elevation_bin = static_cast<int>(std::llround(elevation / 0.018));
    const std::int64_t angular_key =
      (static_cast<std::int64_t>(azimuth_bin) << 32) ^
      static_cast<std::uint32_t>(elevation_bin);
    // Deterministic MID360-style incomplete coverage. A missing sample is not
    // used as free evidence; only retained endpoints generate rays below.
    const auto sample_hash = static_cast<std::uint64_t>(index * 2654435761U) ^
      static_cast<std::uint64_t>(frame * 2246822519U) ^
      static_cast<std::uint64_t>(seed * 3266489917U);
    if (sample_hash % 100U >= (candidate.dynamic ? 88U : 72U)) {
      continue;
    }
    const auto found = nearest.find(angular_key);
    if (found == nearest.end() || range < found->second.range) {
      nearest[angular_key] = {range, candidate};
    }
  }
  std::vector<TruthPoint> output;
  output.reserve(nearest.size());
  for (const auto & item : nearest) {
    output.push_back(item.second.point);
  }
  return output;
}

double percentile(std::vector<double> values, double quantile)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double index = quantile * static_cast<double>(values.size() - 1U);
  const auto lower = static_cast<std::size_t>(std::floor(index));
  const auto upper = static_cast<std::size_t>(std::ceil(index));
  const double ratio = index - static_cast<double>(lower);
  return values[lower] * (1.0 - ratio) + values[upper] * ratio;
}

MapMetrics map_metrics(
  const VoxelSet & map, const VoxelSet & static_truth, const VoxelSet & dynamic_trace,
  const VoxelSet & relocalization_query)
{
  MapMetrics metrics;
  metrics.voxels = map.size();
  std::size_t static_hits = 0U;
  std::size_t dynamic_hits = 0U;
  for (const auto & cell : map) {
    static_hits += static_truth.count(cell);
    if (static_truth.count(cell) == 0U) {
      dynamic_hits += dynamic_trace.count(cell);
    }
  }
  std::size_t overlap = 0U;
  for (const auto & cell : relocalization_query) {
    overlap += map.count(cell);
  }
  metrics.contamination = static_cast<double>(map.size() - static_hits) /
    static_cast<double>(std::max<std::size_t>(1U, map.size()));
  metrics.ghost_ratio = static_cast<double>(dynamic_hits) /
    static_cast<double>(std::max<std::size_t>(1U, map.size()));
  metrics.completeness = static_cast<double>(static_hits) /
    static_cast<double>(std::max<std::size_t>(1U, static_truth.size()));
  metrics.static_preservation = metrics.completeness;
  metrics.false_removal = 1.0 - metrics.completeness;
  metrics.relocalization_contamination = metrics.contamination;
  metrics.relocalization_overlap = static_cast<double>(overlap) /
    static_cast<double>(std::max<std::size_t>(1U, relocalization_query.size()));
  return metrics;
}

double read_rss_mib()
{
  std::ifstream status("/proc/self/status");
  std::string key;
  while (status >> key) {
    if (key == "VmHWM:") {
      double kib = 0.0;
      status >> kib;
      return kib / 1024.0;
    }
    std::string remainder;
    std::getline(status, remainder);
  }
  return 0.0;
}

RunMetrics run_scenario(const std::string & scenario, int seed)
{
  VisibilityFilterConfig observer_config;
  observer_config.ray_stride = 1;
  observer_config.max_range_m = 35.0;
  observer_config.far_range_m = 15.0;
  observer_config.far_static_confirmations = 12U;
  VisibilityAwareDynamicObserver observer(observer_config);
  LongTermMapConfig map_config;
  map_config.voxel_size_m = kVoxel;
  map_config.ray_stride = 2;
  map_config.static_confirmed_observations = 6U;
  map_config.static_confirmed_duration_s = 1.0;
  map_config.static_confirmed_view_bins = 2U;
  map_config.static_consistency_ratio = 0.65;
  map_config.dynamic_candidate_free_traversals = 3U;
  map_config.dynamic_confirmed_free_traversals = 6U;
  map_config.dynamic_confirmed_duration_s = 0.4;
  map_config.far_range_m = 12.0;
  map_config.far_static_confirmed_observations = 60U;
  map_config.far_static_confirmed_duration_s = 15.0;
  map_config.far_static_confirmed_view_bins = 6U;
  LongTermStaticMap refined(map_config);

  VoxelSet raw_map;
  VoxelSet clean_map;
  VoxelSet static_truth;
  VoxelSet dynamic_trace;
  VoxelSet relocalization_query;
  std::vector<double> latencies_ms;
  double convergence_s = 0.0;
  bool converged = false;
  const auto base_static = static_environment();
  const auto wall_start = std::chrono::steady_clock::now();
  const std::clock_t cpu_start = std::clock();

  for (int frame = 0; frame < 280; ++frame) {
    auto world = base_static;
    dynamic_scene(scenario, frame, world);
    const auto origin = sensor_pose(frame, seed);
    const auto returns = measured_returns(world, origin, frame, seed);
    std::vector<Point> endpoints;
    endpoints.reserve(returns.size());
    for (const auto & point : returns) {
      endpoints.push_back(point.point);
      raw_map.insert(voxel(point.point));
      if (point.dynamic) {
        dynamic_trace.insert(voxel(point.point));
      } else {
        static_truth.insert(voxel(point.point));
        if (frame >= 220) {
          relocalization_query.insert(voxel(point.point));
        }
      }
    }
    const auto observed = observer.process(endpoints, origin);
    for (const auto & point : observed.points) {
      if (point.label != PointLabel::kDynamic) {
        clean_map.insert(voxel(point.point));
      }
    }
    const auto update_start = std::chrono::steady_clock::now();
    const auto update = refined.integrate(observed.points, origin, frame * 0.1);
    latencies_ms.push_back(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - update_start).count());
    if (!update.accepted) {
      throw std::runtime_error("deterministic long-term update rejected: " + update.reason);
    }
    if (frame >= 190 && !converged) {
      VoxelSet current;
      for (const auto & point : refined.static_confirmed_points()) {
        current.insert(voxel(point.point));
      }
      const auto metrics = map_metrics(current, static_truth, dynamic_trace, relocalization_query);
      if (metrics.contamination <= 0.03 && metrics.completeness >= 0.80) {
        convergence_s = frame * 0.1;
        converged = true;
      }
    }
  }
  VoxelSet refined_map;
  for (const auto & point : refined.static_confirmed_points()) {
    refined_map.insert(voxel(point.point));
  }
  const auto stats = refined.stats();
  const double wall_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - wall_start).count();
  const double cpu_s = static_cast<double>(std::clock() - cpu_start) /
    static_cast<double>(CLOCKS_PER_SEC);
  RunMetrics output;
  output.scenario = scenario;
  output.seed = seed;
  output.raw = map_metrics(raw_map, static_truth, dynamic_trace, relocalization_query);
  output.clean = map_metrics(clean_map, static_truth, dynamic_trace, relocalization_query);
  output.refined = map_metrics(refined_map, static_truth, dynamic_trace, relocalization_query);
  output.convergence_s = converged ? convergence_s : 28.0;
  output.admission_delay_s = stats.mean_admission_delay_s;
  output.promoted_static_ratio = stats.promoted_static_ratio;
  output.permanent_rejection_ratio = stats.permanent_rejection_ratio;
  output.removed_ghost_voxels = stats.removed_ghost_voxels;
  output.latency_p50_ms = percentile(latencies_ms, 0.50);
  output.latency_p95_ms = percentile(latencies_ms, 0.95);
  output.latency_p99_ms = percentile(latencies_ms, 0.99);
  output.cpu_percent = 100.0 * cpu_s / std::max(1.0e-9, wall_s);
  output.memory_mib = std::max(
    read_rss_mib(), static_cast<double>(stats.approximate_memory_bytes) / (1024.0 * 1024.0));
  return output;
}

MapMetrics mean_map(const std::vector<MapMetrics> & values)
{
  MapMetrics output;
  if (values.empty()) {
    return output;
  }
  for (const auto & value : values) {
    output.contamination += value.contamination;
    output.ghost_ratio += value.ghost_ratio;
    output.completeness += value.completeness;
    output.static_preservation += value.static_preservation;
    output.false_removal += value.false_removal;
    output.relocalization_contamination += value.relocalization_contamination;
    output.relocalization_overlap += value.relocalization_overlap;
    output.voxels += value.voxels;
  }
  const double denominator = static_cast<double>(values.size());
  output.contamination /= denominator;
  output.ghost_ratio /= denominator;
  output.completeness /= denominator;
  output.static_preservation /= denominator;
  output.false_removal /= denominator;
  output.relocalization_contamination /= denominator;
  output.relocalization_overlap /= denominator;
  output.voxels = static_cast<std::size_t>(std::llround(output.voxels / denominator));
  return output;
}

RunMetrics mean_runs(const std::vector<RunMetrics> & runs)
{
  RunMetrics output;
  if (runs.empty()) {
    return output;
  }
  output.scenario = runs.front().scenario;
  std::vector<MapMetrics> raw;
  std::vector<MapMetrics> clean;
  std::vector<MapMetrics> refined;
  for (const auto & run : runs) {
    raw.push_back(run.raw);
    clean.push_back(run.clean);
    refined.push_back(run.refined);
    output.convergence_s += run.convergence_s;
    output.admission_delay_s += run.admission_delay_s;
    output.promoted_static_ratio += run.promoted_static_ratio;
    output.permanent_rejection_ratio += run.permanent_rejection_ratio;
    output.removed_ghost_voxels += run.removed_ghost_voxels;
    output.latency_p50_ms += run.latency_p50_ms;
    output.latency_p95_ms += run.latency_p95_ms;
    output.latency_p99_ms += run.latency_p99_ms;
    output.cpu_percent += run.cpu_percent;
    output.memory_mib += run.memory_mib;
  }
  const double denominator = static_cast<double>(runs.size());
  output.raw = mean_map(raw);
  output.clean = mean_map(clean);
  output.refined = mean_map(refined);
  output.convergence_s /= denominator;
  output.admission_delay_s /= denominator;
  output.promoted_static_ratio /= denominator;
  output.permanent_rejection_ratio /= denominator;
  output.removed_ghost_voxels = static_cast<std::uint64_t>(
    std::llround(output.removed_ghost_voxels / denominator));
  output.latency_p50_ms /= denominator;
  output.latency_p95_ms /= denominator;
  output.latency_p99_ms /= denominator;
  output.cpu_percent /= denominator;
  output.memory_mib /= denominator;
  return output;
}

void write_map(std::ostream & output, const MapMetrics & metrics)
{
  output << "{\"contamination\":" << metrics.contamination <<
    ",\"ghost_ratio\":" << metrics.ghost_ratio <<
    ",\"static_completeness\":" << metrics.completeness <<
    ",\"static_preservation\":" << metrics.static_preservation <<
    ",\"false_removal\":" << metrics.false_removal <<
    ",\"relocalization_contamination\":" << metrics.relocalization_contamination <<
    ",\"relocalization_overlap\":" << metrics.relocalization_overlap <<
    ",\"voxel_count\":" << metrics.voxels << "}";
}

void write_run(std::ostream & output, const RunMetrics & run)
{
  output << "{\"scenario\":\"" << run.scenario << "\",\"seed\":" << run.seed <<
    ",\"raw\":";
  write_map(output, run.raw);
  output << ",\"clean\":";
  write_map(output, run.clean);
  output << ",\"refined\":";
  write_map(output, run.refined);
  output << ",\"convergence_s\":" << run.convergence_s <<
    ",\"admission_delay_s\":" << run.admission_delay_s <<
    ",\"promoted_static_ratio\":" << run.promoted_static_ratio <<
    ",\"permanent_rejection_ratio\":" << run.permanent_rejection_ratio <<
    ",\"removed_ghost_voxels\":" << run.removed_ghost_voxels <<
    ",\"latency_ms\":{\"p50\":" << run.latency_p50_ms <<
    ",\"p95\":" << run.latency_p95_ms << ",\"p99\":" << run.latency_p99_ms << "}" <<
    ",\"cpu_percent\":" << run.cpu_percent <<
    ",\"memory_mib\":" << run.memory_mib << "}";
}

}  // namespace
}  // namespace uf_dynamic_observer

int main(int argc, char ** argv)
{
  using namespace uf_dynamic_observer;
  if (argc != 2) {
    std::cerr << "usage: long_term_static_map_benchmark OUTPUT.json\n";
    return 2;
  }
  const std::vector<std::string> scenarios{
    "person_stays_then_leaves", "repeated_passes", "multiple_crossing",
    "opening_closing_door", "occlusion_reappear", "small_fast_target",
    "slow_target", "near_wall_motion", "large_dynamic_occlusion",
    "far_sparse_target", "stopped_then_moves"};
  std::vector<RunMetrics> runs;
  for (const auto & scenario : scenarios) {
    for (int seed = 0; seed < 3; ++seed) {
      const auto result = run_scenario(scenario, seed);
      runs.push_back(result);
      std::cout << scenario << " seed=" << seed <<
        " raw_contamination=" << result.raw.contamination <<
        " clean_contamination=" << result.clean.contamination <<
        " refined_contamination=" << result.refined.contamination << '\n';
    }
  }
  std::vector<RunMetrics> scenario_means;
  for (const auto & scenario : scenarios) {
    std::vector<RunMetrics> selected;
    for (const auto & run : runs) {
      if (run.scenario == scenario) {
        selected.push_back(run);
      }
    }
    scenario_means.push_back(mean_runs(selected));
  }
  auto overall = mean_runs(scenario_means);
  overall.scenario = "overall";
  std::ofstream output(argv[1]);
  output << std::fixed << std::setprecision(8);
  output << "{\"schema\":\"dyn_map_006_v1\",\"truth_role\":\"evaluator_only\"" <<
    ",\"causal_previous_state_only\":true,\"production_lidar_modified\":false" <<
    ",\"scenario_count\":" << scenarios.size() << ",\"seeds_per_scenario\":3" <<
    ",\"frames_per_run\":280,\"overall\":";
  write_run(output, overall);
  output << ",\"scenario_means\":[";
  for (std::size_t index = 0U; index < scenario_means.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    write_run(output, scenario_means[index]);
  }
  output << "],\"runs\":[";
  for (std::size_t index = 0U; index < runs.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    write_run(output, runs[index]);
  }
  output << "]}\n";
  return output ? 0 : 1;
}
