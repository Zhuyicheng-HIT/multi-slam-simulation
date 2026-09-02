#include "uf_dynamic_observer/conservative_free_space.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
using uf_dynamic_observer::Point;
using uf_dynamic_observer::PointLabel;
using uf_dynamic_observer::VisibilityAwareDynamicObserver;
constexpr double kPi = 3.14159265358979323846;

struct TruthPoint {Point point; bool dynamic{false};};
struct Metrics
{
  std::uint64_t dynamic_total{0U};
  std::uint64_t dynamic_detected{0U};
  std::uint64_t static_total{0U};
  std::uint64_t static_removed{0U};
  std::uint64_t unknown{0U};
  std::uint64_t retained_total{0U};
  std::uint64_t retained_dynamic{0U};
  std::uint64_t direct_free_dynamic{0U};
  std::uint64_t articulated_dynamic{0U};
  std::uint64_t growth_dynamic{0U};
  std::uint64_t tracked_dynamic{0U};
  double raw_information[3]{0.0, 0.0, 0.0};
  double clean_information[3]{0.0, 0.0, 0.0};
  std::vector<double> latency_ms;
};

void add_wall(std::vector<TruthPoint> & points, double x, double y0, double y1)
{
  // Sample the continuous collision surface more densely than an angular ray
  // cell at the far wall. Sparse point lattices create synthetic pinholes:
  // rays pass through the mathematical wall and later make the same voxel look
  // newly occupied, which is not a physically valid free-space contradiction.
  for (double y = y0; y <= y1 + 1.0e-6; y += 0.10) {
    for (double z = 0.04; z <= 2.8; z += 0.10) {
      points.push_back({{x, y, z, 20.0F}, false});
    }
  }
}

void add_ground(std::vector<TruthPoint> & points)
{
  for (double x = -5.0; x <= 7.0; x += 0.18) {
    for (double y = -5.0; y <= 5.0; y += 0.18) {
      points.push_back({{x, y, 0.0, 12.0F}, false});
    }
  }
}

void add_target(
  std::vector<TruthPoint> & points, double x, double y, double half,
  double height, bool dynamic)
{
  for (double dx = -half; dx <= half + 1.0e-6; dx += 0.16) {
    for (double dz = 0.08; dz <= height; dz += 0.16) {
      points.push_back({{x + dx, y - half, dz, 80.0F}, dynamic});
      points.push_back({{x + dx, y + half, dz, 80.0F}, dynamic});
      points.push_back({{x - half, y + dx, dz, 80.0F}, dynamic});
      points.push_back({{x + half, y + dx, dz, 80.0F}, dynamic});
    }
  }
}

Point motion_origin(const std::string & scenario, int frame, int seed)
{
  const double phase = 2.0 * kPi * frame / 64.0 + 0.013 * seed;
  if (scenario == "structural_hold_checkpoint4" ||
    scenario == "window_opening_passive_hold")
  {
    return {-2.2, 0.025 * std::sin(phase), 1.20, 0.0F};
  }
  if (scenario == "fast_checkpoint8_pressure") {
    return {1.0 * std::sin(2.0 * phase), 0.50 * std::sin(4.0 * phase), 1.20, 0.0F};
  }
  return {0.65 * std::sin(phase), 0.325 * std::sin(2.0 * phase), 1.20, 0.0F};
}

std::vector<TruthPoint> scene(const std::string & scenario, int frame)
{
  std::vector<TruthPoint> points;
  add_ground(points);
  add_wall(points, 7.0, -5.0, -0.9);
  add_wall(points, 7.0, 0.9, 5.0);
  add_wall(points, -5.0, -5.0, 5.0);
  add_target(points, -1.8, 2.7, 0.45, 2.2, false);
  if (scenario == "structural_hold_checkpoint4" && frame >= 12) {
    add_target(points, 6.45, -2.0 + 0.065 * (frame - 12), 0.28, 1.75, true);
  } else if (scenario == "fast_figure8_rotate_checkpoint4" && frame >= 12) {
    add_target(points, -1.2, -2.2 + 0.075 * (frame - 12), 0.28, 1.75, true);
    add_target(points, 2.4, 2.2 - 0.075 * (frame - 12), 0.28, 1.75, true);
  } else if (scenario == "window_opening_passive_hold" && frame >= 18 && frame < 50) {
    add_target(points, 6.55, -0.75 + 0.045 * (frame - 18), 0.42, 2.25, true);
  } else if (scenario == "fast_checkpoint8_pressure" && frame >= 10) {
    add_target(points, 2.3, -2.5 + 0.075 * (frame - 10), 1.05, 2.55, true);
  }
  return points;
}

std::vector<TruthPoint> visible(
  const std::vector<TruthPoint> & world, const Point & origin, int frame, int seed)
{
  struct Return {double range{1.0e9}; TruthPoint point;};
  std::map<std::pair<int, int>, Return> rays;
  for (std::size_t index = 0U; index < world.size(); ++index) {
    const auto & candidate = world[index];
    const double dx = candidate.point.x - origin.x;
    const double dy = candidate.point.y - origin.y;
    const double dz = candidate.point.z - origin.z;
    const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (range < 0.5 || range > 25.0) {
      continue;
    }
    const int azimuth = static_cast<int>(std::llround(std::atan2(dy, dx) / 0.018));
    const int elevation = static_cast<int>(std::llround(
      std::atan2(dz, std::hypot(dx, dy)) / 0.018));
    const auto key = std::make_pair(azimuth, elevation);
    auto & output = rays[key];
    if (range < output.range) {
      output = {range, candidate};
    }
  }
  std::vector<TruthPoint> output;
  for (const auto & item : rays) {
    const auto hash = static_cast<std::uint64_t>(item.first.first * 73856093) ^
      static_cast<std::uint64_t>(item.first.second * 19349663) ^
      static_cast<std::uint64_t>(frame * 83492791) ^ static_cast<std::uint64_t>(seed);
    if (hash % 100U < 84U) {
      output.push_back(item.second.point);
    }
  }
  return output;
}

void accumulate(
  Metrics & metrics, const std::vector<TruthPoint> & truth,
  const uf_dynamic_observer::FilterResult & filtered, const Point & origin,
  double latency_ms)
{
  const std::size_t count = std::min(truth.size(), filtered.points.size());
  for (std::size_t index = 0U; index < count; ++index) {
    const bool dynamic = truth[index].dynamic;
    const auto label = filtered.points[index].label;
    metrics.dynamic_total += dynamic ? 1U : 0U;
    metrics.static_total += dynamic ? 0U : 1U;
    metrics.dynamic_detected += dynamic && label == PointLabel::kDynamic ? 1U : 0U;
    metrics.static_removed += !dynamic && label == PointLabel::kDynamic ? 1U : 0U;
    metrics.unknown += label == PointLabel::kUnknown ? 1U : 0U;
    const double axis[3]{
      truth[index].point.x - origin.x,
      truth[index].point.y - origin.y,
      truth[index].point.z - origin.z};
    for (int dimension = 0; dimension < 3; ++dimension) {
      metrics.raw_information[dimension] += axis[dimension] * axis[dimension];
      if (label != PointLabel::kDynamic) {
        metrics.clean_information[dimension] += axis[dimension] * axis[dimension];
      }
    }
    if (label != PointLabel::kDynamic) {
      ++metrics.retained_total;
      metrics.retained_dynamic += dynamic ? 1U : 0U;
    }
  }
  metrics.latency_ms.push_back(latency_ms);
  metrics.direct_free_dynamic += filtered.stats.direct_free_dynamic_points;
  metrics.articulated_dynamic += filtered.stats.articulated_dynamic_points;
  metrics.growth_dynamic += filtered.stats.growth_dynamic_points;
  metrics.tracked_dynamic += filtered.stats.tracked_dynamic_points;
}

double percentile(std::vector<double> values, double q)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double position = q * (values.size() - 1U);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  return values[lower] + (values[upper] - values[lower]) * (position - lower);
}

double ratio(std::uint64_t numerator, std::uint64_t denominator)
{
  return static_cast<double>(numerator) / std::max<std::uint64_t>(1U, denominator);
}

void merge(Metrics & destination, const Metrics & source)
{
  destination.dynamic_total += source.dynamic_total;
  destination.dynamic_detected += source.dynamic_detected;
  destination.static_total += source.static_total;
  destination.static_removed += source.static_removed;
  destination.unknown += source.unknown;
  destination.retained_total += source.retained_total;
  destination.retained_dynamic += source.retained_dynamic;
  destination.direct_free_dynamic += source.direct_free_dynamic;
  destination.articulated_dynamic += source.articulated_dynamic;
  destination.growth_dynamic += source.growth_dynamic;
  destination.tracked_dynamic += source.tracked_dynamic;
  for (int index = 0; index < 3; ++index) {
    destination.raw_information[index] += source.raw_information[index];
    destination.clean_information[index] += source.clean_information[index];
  }
  destination.latency_ms.insert(
    destination.latency_ms.end(), source.latency_ms.begin(), source.latency_ms.end());
}

void write_metrics(std::ostream & output, const Metrics & metrics)
{
  const double recall = ratio(metrics.dynamic_detected, metrics.dynamic_total);
  const double precision = ratio(
    metrics.dynamic_detected, metrics.dynamic_detected + metrics.static_removed);
  output << "{\"precision\":" << precision <<
    ",\"recall\":" << recall <<
    ",\"f1\":" << (precision + recall > 0.0 ?
    2.0 * precision * recall / (precision + recall) : 0.0) <<
    ",\"static_preservation\":" << 1.0 - ratio(metrics.static_removed, metrics.static_total) <<
    ",\"unknown_ratio\":" << ratio(
    metrics.unknown, metrics.dynamic_total + metrics.static_total) <<
    ",\"raw_contamination\":" << ratio(
    metrics.dynamic_total, metrics.dynamic_total + metrics.static_total) <<
    ",\"clean_contamination\":" << ratio(
    metrics.retained_dynamic, metrics.retained_total) <<
    ",\"dynamic_reason_counts\":{\"direct_free\":" << metrics.direct_free_dynamic <<
    ",\"articulated\":" << metrics.articulated_dynamic <<
    ",\"growth\":" << metrics.growth_dynamic <<
    ",\"tracked\":" << metrics.tracked_dynamic << "}" <<
    ",\"information_ratio_xyz\":[";
  for (int index = 0; index < 3; ++index) {
    output << metrics.clean_information[index] /
      std::max(1.0e-12, metrics.raw_information[index]);
    output << (index == 2 ? "]" : ",");
  }
  output << ",\"latency_p50_ms\":" << percentile(metrics.latency_ms, 0.50) <<
    ",\"latency_p95_ms\":" << percentile(metrics.latency_ms, 0.95) <<
    ",\"latency_p99_ms\":" << percentile(metrics.latency_ms, 0.99) << "}";
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string output_path = "dynamic_active_motion_benchmark.json";
  for (int index = 1; index + 1 < argc; ++index) {
    if (std::string(argv[index]) == "--output") {
      output_path = argv[index + 1];
    }
  }
  const std::vector<std::string> scenarios{
    "structural_hold_checkpoint4", "fast_figure8_rotate_checkpoint4",
    "window_opening_passive_hold", "fast_checkpoint8_pressure"};
  std::map<std::string, Metrics> results;
  Metrics aggregate;
  for (const auto & scenario : scenarios) {
    for (int seed = 1; seed <= 3; ++seed) {
      uf_dynamic_observer::VisibilityFilterConfig config;
      config.ray_stride = 1;
      config.max_range_m = 25.0;
      VisibilityAwareDynamicObserver observer(config);
      for (int frame = 0; frame < 64; ++frame) {
        const auto origin = motion_origin(scenario, frame, seed);
        const auto scan = visible(scene(scenario, frame), origin, frame, seed);
        std::vector<Point> points;
        points.reserve(scan.size());
        for (const auto & point : scan) {
          points.push_back(point.point);
        }
        const auto start = std::chrono::steady_clock::now();
        const auto filtered = observer.process(points, origin);
        const double latency_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
        accumulate(results[scenario], scan, filtered, origin, latency_ms);
      }
    }
    merge(aggregate, results[scenario]);
  }
  std::ofstream output(output_path);
  if (!output) {
    return 2;
  }
  output << std::fixed << std::setprecision(6) <<
    "{\n  \"contract\":{\"seeds\":3,\"frames_per_seed\":64,"
    "\"truth_use\":\"evaluator_only\",\"future_pose_used\":false,"
    "\"motion_policy_modified\":false},\n  \"aggregate\":";
  write_metrics(output, aggregate);
  output << ",\n  \"scenarios\":{\n";
  for (std::size_t index = 0U; index < scenarios.size(); ++index) {
    output << "    \"" << scenarios[index] << "\":";
    write_metrics(output, results.at(scenarios[index]));
    output << (index + 1U == scenarios.size() ? "\n" : ",\n");
  }
  output << "  }\n}\n";
  std::cout << output_path << '\n';
  return 0;
}
