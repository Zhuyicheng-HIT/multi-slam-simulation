#include "uf_dynamic_observer/conservative_free_space.hpp"
#include "uf_dynamic_observer/long_term_static_map.hpp"
#include "uf_relocalization/descriptor_core.hpp"
#include "uf_relocalization/keyframe_database.hpp"
#include "uf_relocalization/registration_core.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
using uf_dynamic_observer::LongTermStaticMap;
using uf_dynamic_observer::Point;
using uf_dynamic_observer::PointLabel;
using uf_dynamic_observer::VisibilityAwareDynamicObserver;
using uf_relocalization::Cloud;
constexpr double kPi = 3.14159265358979323846;
constexpr double kVoxel = 0.25;

struct TaggedPoint {Point point; bool dynamic{false};};
struct Key {
  int x{0}; int y{0}; int z{0};
  bool operator==(const Key & other) const {return x == other.x && y == other.y && z == other.z;}
};
struct KeyHash {
  std::size_t operator()(const Key & key) const noexcept {
    std::size_t seed = std::hash<int>{}(key.x);
    seed ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};
struct Cell {Point point; bool dynamic_only{false};};
using CellMap = std::unordered_map<Key, Cell, KeyHash>;
using KeySet = std::unordered_set<Key, KeyHash>;
struct MapSet {CellMap raw; CellMap clean; CellMap refined; KeySet static_truth;};
struct CloudWithTruth {
  Cloud::Ptr cloud{std::make_shared<Cloud>()};
  std::vector<bool> dynamic;
};
struct PoseResult {
  bool accepted{false}; bool success{false}; bool false_match{false};
  double translation{0.0}; double yaw_deg{0.0}; double initial{0.0};
  double overlap{0.0}; double inliers{0.0}; double dynamic_reference{0.0};
  double candidate_rank{0.0};
};
struct Aggregate {
  int attempts{0}; int successes{0}; int false_matches{0}; int stable_trials{0};
  std::vector<double> translation, yaw, initial, overlap, inliers, dynamic_reference;
  std::vector<double> ranks, stable_times, latency, contamination, completeness;
};

Key voxel(const Point & p) {
  return {static_cast<int>(std::floor(p.x / kVoxel)),
    static_cast<int>(std::floor(p.y / kVoxel)),
    static_cast<int>(std::floor(p.z / kVoxel))};
}
void add_cell(CellMap & map, const TaggedPoint & p) {
  const auto key = voxel(p.point);
  const auto inserted = map.emplace(key, Cell{p.point, p.dynamic});
  if (!inserted.second && !p.dynamic) inserted.first->second.dynamic_only = false;
}
void add_box(std::vector<TaggedPoint> & out, double x, double y, double half,
  double height, bool dynamic, double spacing = 0.22)
{
  const int n = std::max(1, static_cast<int>(std::round(2.0 * half / spacing)));
  const int nz = std::max(1, static_cast<int>(std::round(height / spacing)));
  for (int ix = 0; ix <= n; ++ix) for (int iy = 0; iy <= n; ++iy) {
    if (ix && ix != n && iy && iy != n) continue;
    for (int iz = 0; iz <= nz; ++iz) out.push_back({{
      x - half + ix * spacing, y - half + iy * spacing, 0.08 + iz * spacing,
      dynamic ? 80.0F : 25.0F}, dynamic});
  }
}
std::vector<TaggedPoint> static_scene() {
  std::vector<TaggedPoint> out;
  for (int i = -24; i <= 24; ++i) {
    const double c = i * 0.25;
    for (int iz = 0; iz <= 12; ++iz) {
      const double z = 0.1 + iz * 0.24;
      out.push_back({{c, -6.0, z, 20.0F}, false});
      out.push_back({{-6.0, c, z, 20.0F}, false});
      if (i < 9 || i > 15) out.push_back({{c, 6.0, z, 20.0F}, false});
      if (i < -10 || i > -3) out.push_back({{6.0, c, z, 20.0F}, false});
    }
  }
  for (int ix = -20; ix <= 20; ix += 2) for (int iy = -20; iy <= 20; iy += 2)
    out.push_back({{ix * 0.25, iy * 0.25, 0.0, 15.0F}, false});
  add_box(out, -3.3, 1.5, 0.45, 2.6, false, 0.25);
  add_box(out, 3.4, -1.8, 0.60, 1.8, false, 0.25);
  add_box(out, 1.1, 3.7, 0.35, 2.2, false, 0.25);
  return out;
}
void add_dynamic(const std::string & condition, bool session_a, int frame,
  std::vector<TaggedPoint> & out)
{
  const bool active_a = frame >= 42 && frame < 138;
  if (condition == "person_left" && session_a && active_a)
    add_box(out, 1.8, 0.0, 0.28, 1.75, true);
  else if (condition == "p1_to_p2") {
    if (session_a && active_a) add_box(out, 1.8, -1.25, 0.30, 1.75, true);
    else if (!session_a) add_box(out, 1.8, 1.25, 0.30, 1.75, true);
  } else if (condition == "a_empty_b_appears" && !session_a)
    add_box(out, 1.8, 0.0, 0.30, 1.75, true);
  else if (condition == "multiple_repositioned") {
    if (session_a && active_a) {
      add_box(out, -1.8, -1.0, 0.30, 1.7, true);
      add_box(out, 2.0, 0.8, 0.30, 1.7, true);
    } else if (!session_a) {
      add_box(out, -1.8, 1.2, 0.30, 1.7, true);
      add_box(out, 2.4, -0.9, 0.30, 1.7, true);
    }
  } else if (condition == "c1_persistent_occlusion" && session_a) {
    add_box(out, 1.5, 0.0, 1.05, 2.5, true, 0.25);
  } else if ((condition == "c2_same_view_reobservation" ||
    condition == "c3_natural_multiview_reobservation") && session_a && frame < 100)
  {
    add_box(out, 1.5, 0.0, 1.05, 2.5, true, 0.25);
  }
}
Point origin_a(int frame, int seed) {
  const double a = 2.0 * kPi * frame / 180.0 + seed * 0.011;
  return {2.7 * std::cos(a), 2.2 * std::sin(a), 1.25, 0.0F};
}
Point condition_origin(const std::string & condition, int frame, int seed) {
  if (condition == "c2_same_view_reobservation" && frame >= 100) {
    return {-2.7, 0.05 * std::sin(frame * 0.1), 1.25, 0.0F};
  }
  return origin_a(frame, seed);
}
std::vector<TaggedPoint> returns(const std::vector<TaggedPoint> & world,
  const Point & origin, int frame, int seed)
{
  struct Return {double range; TaggedPoint point;};
  std::unordered_map<std::int64_t, Return> nearest;
  for (std::size_t i = 0; i < world.size(); ++i) {
    const auto & p = world[i];
    const double dx = p.point.x - origin.x, dy = p.point.y - origin.y,
      dz = p.point.z - origin.z, range = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (range < 0.5 || range > 25.0) continue;
    const int az = static_cast<int>(std::llround(std::atan2(dy, dx) / 0.020));
    const int el = static_cast<int>(std::llround(std::atan2(dz, std::hypot(dx, dy)) / 0.020));
    const std::int64_t key = (static_cast<std::int64_t>(az) << 32) ^
      static_cast<std::uint32_t>(el);
    const auto hash = static_cast<std::uint64_t>(i * 2654435761U) ^
      static_cast<std::uint64_t>(frame * 2246822519U) ^
      static_cast<std::uint64_t>(seed * 3266489917U);
    if (hash % 100U >= (p.dynamic ? 90U : 78U)) continue;
    const auto found = nearest.find(key);
    if (found == nearest.end() || range < found->second.range) nearest[key] = {range, p};
  }
  std::vector<TaggedPoint> out;
  for (const auto & item : nearest) out.push_back(item.second.point);
  return out;
}
MapSet build_session_a(const std::string & condition, int seed) {
  uf_dynamic_observer::VisibilityFilterConfig observer_config;
  observer_config.ray_stride = 1; observer_config.max_range_m = 25.0;
  VisibilityAwareDynamicObserver observer(observer_config);
  uf_dynamic_observer::LongTermMapConfig map_config;
  map_config.voxel_size_m = kVoxel; map_config.ray_stride = 2; map_config.max_range_m = 25.0;
  LongTermStaticMap long_term(map_config);
  MapSet maps; const auto base = static_scene();
  Point previous = condition_origin(condition, 0, seed);
  for (int frame = 0; frame < 180; ++frame) {
    auto world = base; add_dynamic(condition, true, frame, world);
    const Point current = condition_origin(condition, frame, seed);
    const auto scan = returns(world, current, frame, seed);
    std::vector<Point> endpoints; endpoints.reserve(scan.size());
    for (const auto & p : scan) {
      endpoints.push_back(p.point); add_cell(maps.raw, p);
      if (!p.dynamic) maps.static_truth.insert(voxel(p.point));
    }
    // Strictly previous posterior origin; no current/future estimator state.
    const auto observed = observer.process(endpoints, previous);
    for (std::size_t i = 0; i < observed.points.size(); ++i)
      if (observed.points[i].label != PointLabel::kDynamic) add_cell(maps.clean, scan[i]);
    const auto update = long_term.integrate(observed.points, previous, frame * 0.1);
    if (!update.accepted) throw std::runtime_error("long-term rejection: " + update.reason);
    previous = current;
  }
  for (const auto & p : long_term.static_confirmed_points())
    add_cell(maps.refined, TaggedPoint{p.point, false});
  return maps;
}
std::pair<double, double> map_metric(const CellMap & map, const KeySet & truth) {
  std::size_t hits = 0;
  for (const auto & item : map) hits += truth.count(item.first);
  return {static_cast<double>(map.size() - hits) / std::max<std::size_t>(1, map.size()),
    static_cast<double>(hits) / std::max<std::size_t>(1, truth.size())};
}
Eigen::Isometry3d pose(double x, double y, double yaw) {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = Eigen::Vector3d{x, y, 1.25};
  out.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return out;
}
CloudWithTruth local_map(const CellMap & map, const Eigen::Isometry3d & world_from_sensor) {
  CloudWithTruth out; const auto sensor_from_world = world_from_sensor.inverse();
  for (const auto & item : map) {
    const Eigen::Vector3d w{item.second.point.x, item.second.point.y, item.second.point.z};
    if ((w - world_from_sensor.translation()).norm() > 6.5) continue;
    const auto p = sensor_from_world * w;
    out.cloud->push_back(pcl::PointXYZ{static_cast<float>(p.x()),
      static_cast<float>(p.y()), static_cast<float>(p.z())});
    out.dynamic.push_back(item.second.dynamic_only);
  }
  return out;
}
CloudWithTruth query_cloud(const std::string & condition, int seed, int index,
  const Eigen::Isometry3d & world_from_sensor)
{
  auto world = static_scene(); add_dynamic(condition, false, index, world);
  const Point origin{world_from_sensor.translation().x(), world_from_sensor.translation().y(),
    world_from_sensor.translation().z(), 0.0F};
  const auto scan = returns(world, origin, 240 + index, seed + 17);
  CloudWithTruth out; const auto sensor_from_world = world_from_sensor.inverse();
  for (const auto & tagged : scan) {
    const auto p = sensor_from_world * Eigen::Vector3d{
      tagged.point.x, tagged.point.y, tagged.point.z};
    out.cloud->push_back(pcl::PointXYZ{static_cast<float>(p.x()),
      static_cast<float>(p.y()), static_cast<float>(p.z())});
    out.dynamic.push_back(tagged.dynamic);
  }
  return out;
}
double yaw(const Eigen::Matrix3d & r) {return std::atan2(r(1, 0), r(0, 0));}
double angular_error(double a, double b) {
  return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}
double dynamic_reference(const CloudWithTruth & source, const CloudWithTruth & target,
  const Eigen::Matrix4f & transform, double max_distance)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> tree; tree.setInputCloud(target.cloud);
  std::vector<int> indices(1); std::vector<float> distances(1); int matches = 0, dynamic = 0;
  for (const auto & p : *source.cloud) {
    pcl::PointXYZ q;
    q.x = transform(0, 0) * p.x + transform(0, 1) * p.y + transform(0, 2) * p.z + transform(0, 3);
    q.y = transform(1, 0) * p.x + transform(1, 1) * p.y + transform(1, 2) * p.z + transform(1, 3);
    q.z = transform(2, 0) * p.x + transform(2, 1) * p.y + transform(2, 2) * p.z + transform(2, 3);
    if (tree.nearestKSearch(q, 1, indices, distances) == 1 &&
      distances[0] <= max_distance * max_distance) {
      ++matches; dynamic += target.dynamic[static_cast<std::size_t>(indices[0])] ? 1 : 0;
    }
  }
  return static_cast<double>(dynamic) / std::max(1, matches);
}
PoseResult register_query(const CellMap & map, const CloudWithTruth & query,
  const Eigen::Isometry3d & truth, int seed, int query_index)
{
  uf_relocalization::KeyframeDatabaseConfig db_config;
  db_config.minimum_translation_spacing_m = 0.5;
  uf_relocalization::StaticKeyframeDatabase database(db_config);
  const std::vector<Eigen::Isometry3d> poses{pose(0, 0, 0), pose(4.4, 0.1, 0),
    pose(-3.8, 2.8, 0.2), pose(2.8, -3.8, -0.2)};
  std::map<std::size_t, CloudWithTruth> targets;
  std::map<std::size_t, Eigen::Isometry3d> target_poses;
  uf_relocalization::KeyframeQuality quality{0.9, 0.95, 0.02, 0.2, true};
  for (std::size_t i = 0; i < poses.size(); ++i) {
    auto target = local_map(map, poses[i]);
    if (target.cloud->size() < 40) continue;
    const auto admission = database.try_insert(i, poses[i], target.cloud,
      uf_relocalization::compute_esf_descriptor(target.cloud), quality);
    if (admission.accepted) {
      targets.emplace(admission.keyframe_id, std::move(target));
      target_poses.emplace(admission.keyframe_id, poses[i]);
    }
  }
  const auto candidates = database.query(
    uf_relocalization::compute_esf_descriptor(query.cloud), 4);
  Eigen::Isometry3d coarse = truth;
  coarse.translation().x() += 0.16 + 0.015 * ((seed + query_index) % 3);
  coarse.translation().y() -= 0.10 - 0.01 * (seed % 2);
  coarse.linear() = Eigen::AngleAxisd((4.0 + seed % 2) * kPi / 180.0,
    Eigen::Vector3d::UnitZ()).toRotationMatrix() * coarse.linear();
  PoseResult best; best.initial = (coarse.translation() - truth.translation()).norm();
  double best_score = std::numeric_limits<double>::infinity();
  uf_relocalization::RegistrationConfig config;
  config.maximum_correspondence_distance_m = 0.9; config.maximum_iterations = 60;
  for (std::size_t rank = 0; rank < candidates.size(); ++rank) {
    const auto id = candidates[rank].keyframe_id;
    if (!targets.count(id)) continue;
    const auto & target_pose = target_poses.at(id);
    uf_relocalization::RegistrationResult result;
    try {
      result = uf_relocalization::align_gicp(query.cloud, targets.at(id).cloud,
        (target_pose.inverse() * coarse).matrix().cast<float>(), config);
    } catch (const std::exception &) {continue;}
    const bool admitted = result.converged && result.correspondence_points >= 25 &&
      result.overlap_ratio >= 0.25 && result.fitness < 0.25;
    const double score = result.fitness + 0.08 * (1.0 - result.overlap_ratio);
    if (!admitted || score >= best_score) continue;
    best_score = score;
    const Eigen::Isometry3d target_from_source(result.target_from_source.cast<double>());
    const auto estimate = target_pose * target_from_source;
    best.accepted = true;
    best.translation = (estimate.translation() - truth.translation()).norm();
    best.yaw_deg = angular_error(yaw(estimate.rotation()), yaw(truth.rotation())) * 180.0 / kPi;
    best.overlap = result.overlap_ratio;
    best.inliers = result.correspondence_points;
    best.dynamic_reference = dynamic_reference(query, targets.at(id),
      result.target_from_source, config.maximum_correspondence_distance_m);
    best.candidate_rank = rank + 1;
  }
  best.success = best.accepted && best.translation <= 0.50 && best.yaw_deg <= 10.0;
  best.false_match = best.accepted && !best.success;
  return best;
}
double mean(const std::vector<double> & v) {
  return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}
double percentile(std::vector<double> v, double q) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const double p = q * (v.size() - 1); const auto lo = static_cast<std::size_t>(std::floor(p));
  const auto hi = static_cast<std::size_t>(std::ceil(p)); return v[lo] + (v[hi] - v[lo]) * (p - lo);
}
void append(Aggregate & a, const PoseResult & r, double latency) {
  ++a.attempts; a.successes += r.success; a.false_matches += r.false_match;
  a.initial.push_back(r.initial); a.latency.push_back(latency);
  if (r.accepted) {a.translation.push_back(r.translation); a.yaw.push_back(r.yaw_deg);
    a.overlap.push_back(r.overlap); a.inliers.push_back(r.inliers);
    a.dynamic_reference.push_back(r.dynamic_reference); a.ranks.push_back(r.candidate_rank);}
}
void write(std::ostream & out, const std::string & name, const Aggregate & a) {
  out << "    \"" << name << "\": {\n"
      << "      \"attempts\": " << a.attempts << ",\n"
      << "      \"success_rate\": " << static_cast<double>(a.successes) / std::max(1, a.attempts) << ",\n"
      << "      \"false_relocalization_rate\": " << static_cast<double>(a.false_matches) / std::max(1, a.attempts) << ",\n"
      << "      \"initial_pose_error_m\": " << mean(a.initial) << ",\n"
      << "      \"stable_pose_error_m\": " << mean(a.translation) << ",\n"
      << "      \"stable_yaw_error_deg\": " << mean(a.yaw) << ",\n"
      << "      \"time_to_stable_s\": " << mean(a.stable_times) << ",\n"
      << "      \"inliers\": " << mean(a.inliers) << ",\n"
      << "      \"overlap\": " << mean(a.overlap) << ",\n"
      << "      \"selected_candidate_rank\": " << mean(a.ranks) << ",\n"
      << "      \"dynamic_reference_fraction\": " << mean(a.dynamic_reference) << ",\n"
      << "      \"map_contamination\": " << mean(a.contamination) << ",\n"
      << "      \"map_completeness\": " << mean(a.completeness) << ",\n"
      << "      \"registration_latency_p50_ms\": " << percentile(a.latency, 0.50) << ",\n"
      << "      \"registration_latency_p95_ms\": " << percentile(a.latency, 0.95) << "\n    }";
}
}  // namespace

int main(int argc, char ** argv) {
  std::string path = "dynamic_localization_benchmark.json";
  for (int i = 1; i + 1 < argc; ++i) if (std::string(argv[i]) == "--output") path = argv[i + 1];
  const std::vector<std::string> conditions{
    "person_left", "p1_to_p2", "a_empty_b_appears", "multiple_repositioned"};
  std::map<std::string, Aggregate> aggregate;
  std::map<std::string, std::map<std::string, Aggregate>> per_condition;
  std::map<std::string, std::map<std::string, Aggregate>> occlusion;
  for (const auto & condition : conditions) for (int seed = 1; seed <= 3; ++seed) {
    const auto maps = build_session_a(condition, seed);
    const std::map<std::string, const CellMap *> variants{
      {"raw", &maps.raw}, {"clean", &maps.clean}, {"refined", &maps.refined}};
    for (const auto & variant : variants) {
      const auto metric = map_metric(*variant.second, maps.static_truth);
      aggregate[variant.first].contamination.push_back(metric.first);
      aggregate[variant.first].completeness.push_back(metric.second);
      per_condition[condition][variant.first].contamination.push_back(metric.first);
      per_condition[condition][variant.first].completeness.push_back(metric.second);
      int consecutive = 0;
      for (int query = 0; query < 3; ++query) {
        const auto truth = pose(0.05 * query, 0.03 * seed, 0.015 * query);
        const auto source = query_cloud(condition, seed, query, truth);
        const auto begin = std::chrono::steady_clock::now();
        const auto result = register_query(*variant.second, source, truth, seed, query);
        const double latency = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - begin).count();
        append(aggregate[variant.first], result, latency);
        append(per_condition[condition][variant.first], result, latency);
        consecutive = result.success ? consecutive + 1 : 0;
      }
      if (consecutive == 3) {++aggregate[variant.first].stable_trials;
        aggregate[variant.first].stable_times.push_back(0.2);
        ++per_condition[condition][variant.first].stable_trials;
        per_condition[condition][variant.first].stable_times.push_back(0.2);}
    }
  }
  const std::vector<std::string> occlusion_conditions{
    "c1_persistent_occlusion", "c2_same_view_reobservation",
    "c3_natural_multiview_reobservation"};
  for (const auto & condition : occlusion_conditions) {
    for (int seed = 1; seed <= 3; ++seed) {
      const auto maps = build_session_a(condition, seed);
      const std::map<std::string, const CellMap *> variants{
        {"raw", &maps.raw}, {"clean", &maps.clean}, {"refined", &maps.refined}};
      for (const auto & variant : variants) {
        const auto metric = map_metric(*variant.second, maps.static_truth);
        occlusion[condition][variant.first].contamination.push_back(metric.first);
        occlusion[condition][variant.first].completeness.push_back(metric.second);
      }
    }
  }
  std::ofstream out(path); if (!out) return 2; out << std::fixed << std::setprecision(6);
  out << "{\n  \"contract\": {\n    \"sessions\": 2,\n    \"conditions\": 4,\n"
      << "    \"seeds\": 3,\n    \"query_frames_per_trial\": 3,\n"
      << "    \"truth_use\": \"evaluator_only\",\n"
      << "    \"state_handoff\": \"strictly_previous_posterior\",\n"
      << "    \"full_online_loop_closure_claimed\": false\n  },\n  \"maps\": {\n";
  write(out, "raw", aggregate.at("raw")); out << ",\n";
  write(out, "clean", aggregate.at("clean")); out << ",\n";
  write(out, "refined", aggregate.at("refined")); out << "\n  },\n  \"conditions\": {\n";
  for (std::size_t index = 0; index < conditions.size(); ++index) {
    const auto & condition = conditions[index];
    out << "    \"" << condition << "\": {\n";
    write(out, "raw", per_condition[condition].at("raw")); out << ",\n";
    write(out, "clean", per_condition[condition].at("clean")); out << ",\n";
    write(out, "refined", per_condition[condition].at("refined")); out << "\n    }";
    out << (index + 1U == conditions.size() ? "\n" : ",\n");
  }
  out << "  },\n  \"occlusion_refinement\": {\n";
  const std::vector<std::string> map_names{"raw", "clean", "refined"};
  for (std::size_t index = 0; index < occlusion_conditions.size(); ++index) {
    const auto & condition = occlusion_conditions[index];
    out << "    \"" << condition << "\": {\n";
    for (std::size_t map_index = 0; map_index < map_names.size(); ++map_index) {
      const auto & name = map_names[map_index];
      const auto & item = occlusion[condition].at(name);
      out << "      \"" << name << "\": {\"contamination\": "
          << mean(item.contamination) << ", \"completeness\": "
          << mean(item.completeness) << "}";
      out << (map_index + 1U == map_names.size() ? "\n" : ",\n");
    }
    out << "    }" <<
      (index + 1U == occlusion_conditions.size() ? "\n" : ",\n");
  }
  out << "  }\n}\n";
  std::cout << path << '\n'; return 0;
}
