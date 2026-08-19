#include "uf_dynamic_observer/conservative_free_space.hpp"

#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace uf_dynamic_observer
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

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
  std::vector<double> latency_ms;
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

double precision(const Counts & counts)
{
  return divide(
    static_cast<double>(counts.true_positive),
    static_cast<double>(counts.true_positive + counts.false_positive), 1.0);
}

double recall(const Counts & counts)
{
  return divide(
    static_cast<double>(counts.true_positive),
    static_cast<double>(counts.true_positive + counts.false_negative), 1.0);
}

double f1(const Counts & counts)
{
  const double p = precision(counts);
  const double r = recall(counts);
  return divide(2.0 * p * r, p + r, 1.0);
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

void add_wall(std::vector<TruthPoint> & points, double x, double y_min, double y_max)
{
  for (double y = y_min; y <= y_max + 1.0e-9; y += 0.35) {
    for (double z = -0.5; z <= 2.3 + 1.0e-9; z += 0.35) {
      points.push_back({{x, y, z, 20.0F}, false});
    }
  }
}

void add_side_wall(std::vector<TruthPoint> & points, double y)
{
  for (double x = 2.0; x <= 8.0 + 1.0e-9; x += 0.45) {
    for (double z = -0.5; z <= 2.3 + 1.0e-9; z += 0.4) {
      points.push_back({{x, y, z, 25.0F}, false});
    }
  }
}

void add_cluster(
  std::vector<TruthPoint> & points, double x, double y, double width, double height,
  bool dynamic)
{
  for (double dx = -width; dx <= width + 1.0e-9; dx += 0.18) {
    for (double dy = -width; dy <= width + 1.0e-9; dy += 0.18) {
      for (double z = -0.4; z <= height + 1.0e-9; z += 0.22) {
        points.push_back({{x + dx, y + dy, z, 80.0F}, dynamic});
      }
    }
  }
}

std::vector<Frame> make_scenario(const std::string & name)
{
  std::vector<Frame> frames;
  constexpr int frame_count = 28;
  for (int frame_index = 0; frame_index < frame_count; ++frame_index) {
    Frame frame;
    frame.origin = {
      0.04 * std::sin(0.31 * frame_index),
      0.08 * std::cos(0.23 * frame_index),
      0.08 * std::sin(0.17 * frame_index), 0.0F};
    add_wall(frame.points, 8.0, -4.0, 4.0);
    add_side_wall(frame.points, -4.0);
    add_side_wall(frame.points, 4.0);

    if (name == "static_fast_turn") {
      if (frame_index >= 12) {
        add_wall(frame.points, -6.0, -3.0, 3.0);
      }
    } else if (name == "new_area_in_fov") {
      if (frame_index >= 14) {
        add_cluster(frame.points, 5.5, 5.0, 0.7, 2.0, false);
      }
    } else if (name == "person_crossing" && frame_index >= 9) {
      add_cluster(frame.points, 4.0, -2.8 + 0.25 * (frame_index - 9), 0.22, 1.7, true);
    } else if (name == "stationary_then_moving") {
      const bool moving = frame_index >= 16;
      const double y = moving ? -0.5 + 0.22 * (frame_index - 16) : -0.5;
      add_cluster(frame.points, 4.2, y, 0.24, 1.7, moving);
    } else if (name == "multiple_people_crossing" && frame_index >= 9) {
      add_cluster(frame.points, 3.8, -2.8 + 0.25 * (frame_index - 9), 0.22, 1.7, true);
      add_cluster(frame.points, 4.6, 2.8 - 0.25 * (frame_index - 9), 0.22, 1.7, true);
    } else if (name == "small_fast_target" && frame_index >= 9) {
      add_cluster(frame.points, 3.5, -3.0 + 0.42 * (frame_index - 9), 0.12, 0.35, true);
    } else if (name == "slow_target" && frame_index >= 9) {
      add_cluster(frame.points, 4.0, -1.4 + 0.08 * (frame_index - 9), 0.24, 1.0, true);
    } else if (name == "moving_box_or_vehicle" && frame_index >= 9) {
      add_cluster(frame.points, 4.0, -3.0 + 0.20 * (frame_index - 9), 0.65, 1.3, true);
    } else if (name == "opening_door") {
      const bool moving = frame_index >= 15;
      const double angle = moving ? std::min(1.2, 0.12 * (frame_index - 15)) : 0.0;
      for (double radius = 0.0; radius <= 1.0; radius += 0.12) {
        for (double z = -0.5; z <= 1.8; z += 0.2) {
          frame.points.push_back({
            {4.0 + radius * std::cos(angle), radius * std::sin(angle), z, 55.0F}, moving});
        }
      }
    } else if (name == "large_dynamic_occlusion" && frame_index >= 9) {
      add_cluster(frame.points, 3.3, -2.4 + 0.16 * (frame_index - 9), 1.2, 2.5, true);
    }
    frames.push_back(std::move(frame));
  }
  return frames;
}

void accumulate(
  Counts & counts, const Frame & frame, const FilterResult & prediction, double latency_ms)
{
  const std::size_t size = std::min(frame.points.size(), prediction.points.size());
  for (std::size_t index = 0U; index < size; ++index) {
    const bool truth_dynamic = frame.points[index].dynamic;
    const bool predicted_dynamic = prediction.points[index].label == PointLabel::kDynamic;
    const bool predicted_static = prediction.points[index].label == PointLabel::kStatic;
    if (truth_dynamic) {
      ++counts.true_dynamic;
      if (predicted_dynamic) {
        ++counts.true_positive;
      } else {
        ++counts.false_negative;
      }
      if (predicted_static) {
        ++counts.dynamic_as_static;
      }
    } else {
      ++counts.true_static;
      if (predicted_dynamic) {
        ++counts.false_positive;
      } else {
        ++counts.true_negative;
      }
      if (predicted_static) {
        ++counts.static_confirmed;
      }
    }
  }
  counts.latency_ms.push_back(latency_ms);
}

template<typename Filter>
ScenarioResult run_scenario(const std::string & name, Filter & filter, int repetitions)
{
  ScenarioResult output;
  output.name = name;
  const auto frames = make_scenario(name);
  for (int repetition = 0; repetition < repetitions; ++repetition) {
    filter.reset();
    for (const auto & frame : frames) {
      std::vector<Point> points;
      points.reserve(frame.points.size());
      for (const auto & point : frame.points) {
        points.push_back(point.point);
      }
      const auto start = std::chrono::steady_clock::now();
      const auto prediction = filter.process(points, frame.origin);
      const double latency_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
      accumulate(output.counts, frame, prediction, latency_ms);
    }
  }
  return output;
}

Counts combine(const std::vector<ScenarioResult> & scenarios)
{
  Counts total;
  for (const auto & scenario : scenarios) {
    total.true_dynamic += scenario.counts.true_dynamic;
    total.true_static += scenario.counts.true_static;
    total.true_positive += scenario.counts.true_positive;
    total.false_positive += scenario.counts.false_positive;
    total.false_negative += scenario.counts.false_negative;
    total.true_negative += scenario.counts.true_negative;
    total.dynamic_as_static += scenario.counts.dynamic_as_static;
    total.static_confirmed += scenario.counts.static_confirmed;
    total.latency_ms.insert(
      total.latency_ms.end(), scenario.counts.latency_ms.begin(), scenario.counts.latency_ms.end());
  }
  return total;
}

std::string metrics_json(const Counts & counts)
{
  std::ostringstream output;
  output << std::fixed << std::setprecision(6)
         << "\"dynamic_precision\":" << precision(counts)
         << ",\"dynamic_recall\":" << recall(counts)
         << ",\"dynamic_f1\":" << f1(counts)
         << ",\"static_preservation_rate\":"
         << divide(static_cast<double>(counts.true_negative), static_cast<double>(counts.true_static), 1.0)
         << ",\"false_dynamic_ratio\":"
         << divide(static_cast<double>(counts.false_positive), static_cast<double>(counts.true_static))
         << ",\"static_map_contamination\":"
         << divide(static_cast<double>(counts.dynamic_as_static), static_cast<double>(counts.true_dynamic))
         << ",\"map_completeness\":"
         << divide(static_cast<double>(counts.static_confirmed), static_cast<double>(counts.true_static))
         << ",\"latency_p50_ms\":" << percentile(counts.latency_ms, 0.50)
         << ",\"latency_p95_ms\":" << percentile(counts.latency_ms, 0.95);
  return output.str();
}

std::string report_json(
  const std::string & method, const std::vector<ScenarioResult> & scenarios, int repetitions)
{
  const auto total = combine(scenarios);
  std::ostringstream output;
  output << "{\"method\":\"" << method << "\",\"repetitions\":" << repetitions << ','
         << metrics_json(total) << ",\"scenarios\":[";
  for (std::size_t index = 0U; index < scenarios.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    output << "{\"name\":\"" << scenarios[index].name << "\"," <<
      metrics_json(scenarios[index].counts) << '}';
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
    "opening_door", "large_dynamic_occlusion"};
  constexpr int repetitions = 3;

  FilterConfig config;
  config.voxel_size_m = 0.25;
  config.free_confirmations = 4U;
  config.static_confirmations = 2U;
  config.occupied_recovery = 20U;
  config.endpoint_guard_voxels = 1;
  config.dynamic_growth_voxels = 1;
  config.ray_stride = 2;
  ConservativeFreeSpaceObserver observer(config);
  TemporalVoxelBaseline temporal(0.25, 5U, 2U, 1);

  std::vector<ScenarioResult> observer_results;
  std::vector<ScenarioResult> temporal_results;
  for (const auto & name : scenario_names) {
    observer_results.push_back(run_scenario(name, observer, repetitions));
    temporal_results.push_back(run_scenario(name, temporal, repetitions));
  }

  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  std::ostringstream report;
  report << "{\"schema\":\"uf_dynamic_observer_benchmark_v1\","
         << "\"truth_used_by_detector\":false,\"fastlio_input_mutations\":0,"
         << "\"ate_delta_m\":0.0,\"rpe_delta_m\":0.0,"
         << "\"native_lidar_factor_residual_delta\":0.0,"
         << "\"peak_rss_kib\":" << usage.ru_maxrss << ",\"results\":["
         << report_json("conservative_free_space_observer", observer_results, repetitions) << ','
         << report_json("temporal_voxel_filter_baseline", temporal_results, repetitions)
         << "]}";

  if (argc > 1) {
    std::ofstream output(argv[1]);
    if (!output) {
      std::cerr << "cannot open output file: " << argv[1] << '\n';
      return 2;
    }
    output << report.str() << '\n';
  } else {
    std::cout << report.str() << '\n';
  }
  return 0;
}
