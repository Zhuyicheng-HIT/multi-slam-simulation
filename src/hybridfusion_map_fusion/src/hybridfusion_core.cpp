#include "hybridfusion_map_fusion/hybridfusion_core.hpp"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/esf.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>

#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_set>

namespace hybridfusion_map_fusion
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
using Key = std::pair<int, int>;

struct Block
{
  Key key;
  CloudPtr cloud{new CloudT};
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  std::array<float, 640> descriptor{};
};

struct Match
{
  Key source;
  Key target;
  double descriptor_score{0.0};
  double neighbor_score{0.0};
};

struct Ndt2dCell
{
  std::size_t count{0};
  Eigen::Vector2d sum{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d sum_outer{Eigen::Matrix2d::Zero()};
  Eigen::Vector2d mean{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d information{Eigen::Matrix2d::Identity()};
};

template<typename T>
T yaml_value(const YAML::Node & node, const char * key, const T & fallback)
{
  return node && node[key] ? node[key].as<T>() : fallback;
}

double degrees(double radians)
{
  return radians * 180.0 / kPi;
}

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Isometry3d yaml_transform(const YAML::Node & node, const std::string & label)
{
  if (!node || !node.IsSequence() || node.size() != 6) {
    throw std::runtime_error(label + " must contain [x, y, z, roll, pitch, yaw]");
  }
  std::array<double, 6> values{};
  for (std::size_t index = 0; index < values.size(); ++index) {
    values[index] = node[index].as<double>();
  }
  return xyz_rpy_transform(values);
}

std::array<float, 640> compute_esf(const CloudPtr & cloud)
{
  if (!cloud || cloud->size() < 20) {
    throw std::runtime_error("ESF requires at least 20 finite points");
  }
  pcl::ESFEstimation<PointT, pcl::ESFSignature640> estimator;
  pcl::PointCloud<pcl::ESFSignature640> output;
  estimator.setInputCloud(cloud);
  estimator.compute(output);
  if (output.empty()) {
    throw std::runtime_error("PCL ESF returned an empty descriptor");
  }
  std::array<float, 640> descriptor{};
  std::copy(std::begin(output.front().histogram), std::end(output.front().histogram),
    descriptor.begin());
  return descriptor;
}

Eigen::Vector3d cloud_centroid(const CloudPtr & cloud)
{
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  std::size_t count = 0;
  for (const auto & point : cloud->points) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    sum += Eigen::Vector3d(point.x, point.y, point.z);
    ++count;
  }
  return count == 0 ? sum : sum / static_cast<double>(count);
}

std::map<Key, Block> make_blocks(
  const CloudPtr & cloud, double origin_x, double origin_y,
  double cell_size, std::size_t min_points)
{
  std::map<Key, Block> blocks;
  for (const auto & point : cloud->points) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    const int ix = static_cast<int>(std::floor((point.x - origin_x) / cell_size));
    const int iy = static_cast<int>(std::floor((point.y - origin_y) / cell_size));
    const Key key{ix, iy};
    auto & block = blocks[key];
    block.key = key;
    block.cloud->push_back(point);
  }

  for (auto iterator = blocks.begin(); iterator != blocks.end();) {
    if (iterator->second.cloud->size() < min_points) {
      iterator = blocks.erase(iterator);
      continue;
    }
    iterator->second.cloud->width = static_cast<std::uint32_t>(iterator->second.cloud->size());
    iterator->second.cloud->height = 1;
    iterator->second.cloud->is_dense = false;
    iterator->second.centroid = cloud_centroid(iterator->second.cloud);
    iterator->second.descriptor = compute_esf(iterator->second.cloud);
    ++iterator;
  }
  return blocks;
}

CloudPtr collect_neighborhood(
  const std::map<Key, Block> & blocks, const Key & center, int radius)
{
  CloudPtr output(new CloudT);
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      const auto iterator = blocks.find(Key{center.first + dx, center.second + dy});
      if (iterator != blocks.end()) {
        *output += *iterator->second.cloud;
      }
    }
  }
  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = false;
  return output;
}

double neighbor_consistency(
  const std::map<Key, Block> & source_blocks,
  const std::map<Key, Block> & target_blocks,
  const Key & source_key, const Key & target_key, int radius,
  int & comparisons)
{
  double total = 0.0;
  comparisons = 0;
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const auto source = source_blocks.find(Key{source_key.first + dx, source_key.second + dy});
      const auto target = target_blocks.find(Key{target_key.first + dx, target_key.second + dy});
      if (source == source_blocks.end() || target == target_blocks.end()) {
        continue;
      }
      total += pearson_correlation(source->second.descriptor, target->second.descriptor);
      ++comparisons;
    }
  }
  return comparisons == 0 ? 0.0 : total / static_cast<double>(comparisons);
}

double cloud_nn_score(const CloudPtr & source, const CloudPtr & target, double max_range)
{
  if (!source || !target || source->empty() || target->empty()) {
    return std::numeric_limits<double>::infinity();
  }
  pcl::KdTreeFLANN<PointT> tree;
  tree.setInputCloud(target);
  std::vector<int> indices(1);
  std::vector<float> squared(1);
  double total = 0.0;
  std::size_t count = 0;
  const double max_squared = max_range * max_range;
  for (const auto & point : source->points) {
    if (!pcl::isFinite(point) || tree.nearestKSearch(point, 1, indices, squared) == 0) {
      continue;
    }
    if (squared.front() <= max_squared) {
      total += squared.front();
      ++count;
    }
  }
  return count == 0 ? std::numeric_limits<double>::infinity() :
         total / static_cast<double>(count);
}

double cloud_nn_mean_distance(const CloudPtr & source, const CloudPtr & target, double max_range)
{
  if (!source || !target || source->empty() || target->empty()) {
    return std::numeric_limits<double>::infinity();
  }
  pcl::KdTreeFLANN<PointT> tree;
  tree.setInputCloud(target);
  std::vector<int> indices(1);
  std::vector<float> squared(1);
  double total = 0.0;
  std::size_t count = 0;
  const double max_squared = max_range * max_range;
  for (const auto & point : source->points) {
    if (pcl::isFinite(point) && tree.nearestKSearch(point, 1, indices, squared) > 0 &&
      squared.front() <= max_squared)
    {
      total += std::sqrt(squared.front());
      ++count;
    }
  }
  return count == 0 ? std::numeric_limits<double>::infinity() :
         total / static_cast<double>(count);
}

std::pair<bool, Eigen::Isometry3d> run_ndt(
  const CloudPtr & source, const CloudPtr & target, const NdtConfig & config,
  const Eigen::Isometry3d & guess, double & fitness)
{
  fitness = std::numeric_limits<double>::infinity();
  if (!source || !target || source->size() < 20 || target->size() < 20) {
    return {false, guess};
  }
  pcl::NormalDistributionsTransform<PointT, PointT> ndt;
  ndt.setResolution(config.resolution_m);
  ndt.setStepSize(config.step_size_m);
  ndt.setTransformationEpsilon(config.transformation_epsilon);
  ndt.setMaximumIterations(config.max_iterations);
  ndt.setInputSource(source);
  ndt.setInputTarget(target);
  CloudT aligned;
  ndt.align(aligned, guess.matrix().cast<float>());
  fitness = ndt.getFitnessScore(config.fitness_max_range_m);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.matrix() = ndt.getFinalTransformation().cast<double>();
  const bool valid = ndt.hasConverged() && std::isfinite(fitness) &&
    fitness <= config.max_fitness && result.matrix().allFinite();
  return {valid, result};
}

std::pair<bool, Eigen::Isometry3d> run_ndt_2d(
  const CloudPtr & source, const CloudPtr & target, const NdtConfig & config,
  const Eigen::Isometry3d & guess, double & fitness)
{
  fitness = std::numeric_limits<double>::infinity();
  if (!source || !target || source->size() < 20 || target->size() < 20) {
    return {false, guess};
  }
  std::map<Key, Ndt2dCell> cells;
  for (const auto & point : target->points) {
    const Key key{
      static_cast<int>(std::floor(point.x / config.resolution_m)),
      static_cast<int>(std::floor(point.y / config.resolution_m))};
    auto & cell = cells[key];
    const Eigen::Vector2d value(point.x, point.y);
    ++cell.count;
    cell.sum += value;
    cell.sum_outer += value * value.transpose();
  }
  for (auto iterator = cells.begin(); iterator != cells.end();) {
    if (iterator->second.count < 3) {
      iterator = cells.erase(iterator);
      continue;
    }
    auto & cell = iterator->second;
    cell.mean = cell.sum / static_cast<double>(cell.count);
    Eigen::Matrix2d covariance =
      cell.sum_outer / static_cast<double>(cell.count) - cell.mean * cell.mean.transpose();
    // PCL's 3D NDT is undefined for exactly planar z=0 boundary data.  This is
    // a true SE(2) NDT cell with explicit covariance regularization.
    const double regularizer = std::max(0.02, config.resolution_m * 0.08);
    covariance += Eigen::Matrix2d::Identity() * regularizer * regularizer;
    cell.information = covariance.inverse();
    ++iterator;
  }
  if (cells.size() < 3) {
    return {false, guess};
  }

  Eigen::Vector3d state(
    guess.translation().x(), guess.translation().y(),
    std::atan2(guess.rotation()(1, 0), guess.rotation()(0, 0)));
  bool converged = false;
  std::size_t final_matches = 0;
  double final_cost = 0.0;
  for (int iteration = 0; iteration < config.max_iterations; ++iteration) {
    Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    std::size_t matches = 0;
    double euclidean_cost = 0.0;
    const double cosine = std::cos(state.z());
    const double sine = std::sin(state.z());
    const Eigen::Matrix2d rotation = (Eigen::Matrix2d() <<
      cosine, -sine, sine, cosine).finished();
    for (const auto & point : source->points) {
      const Eigen::Vector2d original(point.x, point.y);
      const Eigen::Vector2d transformed = rotation * original + state.head<2>();
      const Key center{
        static_cast<int>(std::floor(transformed.x() / config.resolution_m)),
        static_cast<int>(std::floor(transformed.y() / config.resolution_m))};
      const Ndt2dCell * selected = nullptr;
      double selected_cost = std::numeric_limits<double>::infinity();
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          const auto candidate = cells.find(Key{center.first + dx, center.second + dy});
          if (candidate == cells.end()) {
            continue;
          }
          const Eigen::Vector2d residual = transformed - candidate->second.mean;
          const double cost = residual.transpose() * candidate->second.information * residual;
          if (cost < selected_cost) {
            selected_cost = cost;
            selected = &candidate->second;
          }
        }
      }
      if (selected == nullptr || selected_cost > 25.0) {
        continue;
      }
      const Eigen::Vector2d residual = transformed - selected->mean;
      Eigen::Matrix<double, 2, 3> jacobian;
      jacobian <<
        1.0, 0.0, -sine * original.x() - cosine * original.y(),
        0.0, 1.0, cosine * original.x() - sine * original.y();
      hessian += jacobian.transpose() * selected->information * jacobian;
      gradient += jacobian.transpose() * selected->information * residual;
      euclidean_cost += residual.squaredNorm();
      ++matches;
    }
    if (matches < 12 || std::abs(hessian.determinant()) < 1e-12) {
      return {false, guess};
    }
    Eigen::Vector3d delta = -hessian.ldlt().solve(gradient);
    const double translation_step = delta.head<2>().norm();
    if (translation_step > config.step_size_m) {
      delta.head<2>() *= config.step_size_m / translation_step;
    }
    delta.z() = std::clamp(delta.z(), -0.12, 0.12);
    state += delta;
    state.z() = wrap_angle(state.z());
    final_matches = matches;
    final_cost = euclidean_cost;
    if (delta.norm() <= config.transformation_epsilon) {
      converged = true;
      break;
    }
  }
  fitness = final_matches == 0 ? std::numeric_limits<double>::infinity() :
    final_cost / static_cast<double>(final_matches);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(state.x(), state.y(), 0.0);
  result.linear() = Eigen::AngleAxisd(state.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return {converged && std::isfinite(fitness) && fitness <= config.max_fitness, result};
}

Eigen::Isometry3d constrain_xy_yaw(const Eigen::Isometry3d & transform)
{
  const double yaw = std::atan2(transform.rotation()(1, 0), transform.rotation()(0, 0));
  Eigen::Isometry3d constrained = Eigen::Isometry3d::Identity();
  constrained.translation() = Eigen::Vector3d(
    transform.translation().x(), transform.translation().y(), 0.0);
  constrained.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return constrained;
}

std::vector<std::size_t> largest_transform_cluster(
  const std::vector<Eigen::Isometry3d> & transforms, const ClusterConfig & config)
{
  std::vector<bool> visited(transforms.size(), false);
  std::vector<std::size_t> best;
  for (std::size_t seed = 0; seed < transforms.size(); ++seed) {
    if (visited[seed]) {
      continue;
    }
    std::queue<std::size_t> pending;
    std::vector<std::size_t> cluster;
    visited[seed] = true;
    pending.push(seed);
    while (!pending.empty()) {
      const auto current = pending.front();
      pending.pop();
      cluster.push_back(current);
      for (std::size_t other = 0; other < transforms.size(); ++other) {
        if (visited[other]) {
          continue;
        }
        const double translation =
          (transforms[current].translation() - transforms[other].translation()).norm();
        const double rotation = rotation_distance_deg(
          transforms[current].rotation(), transforms[other].rotation());
        if (translation <= config.translation_threshold_m &&
          rotation <= config.rotation_threshold_deg)
        {
          visited[other] = true;
          pending.push(other);
        }
      }
    }
    if (cluster.size() > best.size()) {
      best = std::move(cluster);
    }
  }
  return best;
}

std::string json_escape(const std::string & value)
{
  std::ostringstream output;
  for (const char character : value) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default: output << character; break;
    }
  }
  return output.str();
}

std::string finite_json(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream output;
  output << std::setprecision(10) << value;
  return output.str();
}

long peak_rss_kib()
{
  struct rusage usage {};
  return getrusage(RUSAGE_SELF, &usage) == 0 ? usage.ru_maxrss : 0;
}

std::tuple<int, int, int> voxel_key(const PointT & point, double leaf)
{
  return {
    static_cast<int>(std::floor(point.x / leaf)),
    static_cast<int>(std::floor(point.y / leaf)),
    static_cast<int>(std::floor(point.z / leaf))};
}

}  // namespace

Eigen::Isometry3d xyz_rpy_transform(const std::array<double, 6> & values)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(values[0], values[1], values[2]);
  transform.linear() = (
    Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
  return transform;
}

std::array<double, 6> transform_xyz_rpy(const Eigen::Isometry3d & transform)
{
  const Eigen::Vector3d rpy = transform.rotation().eulerAngles(0, 1, 2);
  return {
    transform.translation().x(), transform.translation().y(), transform.translation().z(),
    rpy.x(), rpy.y(), rpy.z()};
}

Config load_config(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node node = root["hybridfusion"];
  if (!node) {
    throw std::runtime_error("configuration requires a hybridfusion root");
  }
  Config config;
  const auto preprocess = node["preprocess"];
  config.visual_voxel_leaf_m = yaml_value(preprocess, "visual_voxel_leaf_m", config.visual_voxel_leaf_m);
  config.lidar_voxel_leaf_m = yaml_value(preprocess, "lidar_voxel_leaf_m", config.lidar_voxel_leaf_m);

  const auto grid = node["grid"];
  config.grid.cell_size_m = yaml_value(grid, "cell_size_m", config.grid.cell_size_m);
  config.grid.auto_scene_divisor = yaml_value(grid, "auto_scene_divisor", config.grid.auto_scene_divisor);
  config.grid.min_points = yaml_value(grid, "min_points", config.grid.min_points);
  config.grid.neighbor_radius_cells = yaml_value(grid, "neighbor_radius_cells", config.grid.neighbor_radius_cells);

  const auto candidate = node["candidate"];
  config.candidate.radial_scale_lambda = yaml_value(candidate, "radial_scale_lambda", config.candidate.radial_scale_lambda);
  config.candidate.angular_tolerance_deg = yaml_value(candidate, "angular_tolerance_deg", config.candidate.angular_tolerance_deg);
  config.candidate.max_centroid_distance_m = yaml_value(candidate, "max_centroid_distance_m", config.candidate.max_centroid_distance_m);
  config.candidate.esf_correlation_threshold = yaml_value(candidate, "esf_correlation_threshold", config.candidate.esf_correlation_threshold);
  config.candidate.neighbor_correlation_threshold = yaml_value(candidate, "neighbor_correlation_threshold", config.candidate.neighbor_correlation_threshold);
  config.candidate.min_consistent_neighbors = yaml_value(candidate, "min_consistent_neighbors", config.candidate.min_consistent_neighbors);
  config.candidate.max_candidates_per_block = yaml_value(candidate, "max_candidates_per_block", config.candidate.max_candidates_per_block);
  config.candidate.max_local_registrations = yaml_value(candidate, "max_local_registrations", config.candidate.max_local_registrations);

  const auto boundary = node["boundary"];
  config.boundary.ground_quantile = yaml_value(boundary, "ground_quantile", config.boundary.ground_quantile);
  config.boundary.ground_clearance_m = yaml_value(boundary, "ground_clearance_m", config.boundary.ground_clearance_m);
  config.boundary.raster_resolution_m = yaml_value(boundary, "raster_resolution_m", config.boundary.raster_resolution_m);
  config.boundary.max_occupied_neighbors = yaml_value(boundary, "max_occupied_neighbors", config.boundary.max_occupied_neighbors);
  config.boundary.min_boundary_points = yaml_value(boundary, "min_boundary_points", config.boundary.min_boundary_points);

  const auto read_ndt = [](const YAML::Node & value, NdtConfig & output) {
      output.resolution_m = yaml_value(value, "resolution_m", output.resolution_m);
      output.step_size_m = yaml_value(value, "step_size_m", output.step_size_m);
      output.transformation_epsilon = yaml_value(value, "transformation_epsilon", output.transformation_epsilon);
      output.max_iterations = yaml_value(value, "max_iterations", output.max_iterations);
      output.max_fitness = yaml_value(value, "max_fitness", output.max_fitness);
      output.fitness_max_range_m = yaml_value(value, "fitness_max_range_m", output.fitness_max_range_m);
    };
  read_ndt(node["ndt_2d"], config.ndt_2d);
  read_ndt(node["ndt_3d"], config.ndt_3d);

  const auto gicp = node["gicp"];
  config.gicp.max_iterations = yaml_value(gicp, "max_iterations", config.gicp.max_iterations);
  config.gicp.max_correspondence_distance_m = yaml_value(gicp, "max_correspondence_distance_m", config.gicp.max_correspondence_distance_m);
  config.gicp.transformation_epsilon = yaml_value(gicp, "transformation_epsilon", config.gicp.transformation_epsilon);
  config.gicp.fitness_epsilon = yaml_value(gicp, "fitness_epsilon", config.gicp.fitness_epsilon);

  const auto cluster = node["cluster"];
  config.cluster.translation_threshold_m = yaml_value(cluster, "translation_threshold_m", config.cluster.translation_threshold_m);
  config.cluster.rotation_threshold_deg = yaml_value(cluster, "rotation_threshold_deg", config.cluster.rotation_threshold_deg);
  config.cluster.min_cluster_size = yaml_value(cluster, "min_cluster_size", config.cluster.min_cluster_size);
  config.cluster.enable_global_ndt_refine = yaml_value(cluster, "enable_global_ndt_refine", config.cluster.enable_global_ndt_refine);
  config.cluster.max_refine_fitness_ratio = yaml_value(cluster, "max_refine_fitness_ratio", config.cluster.max_refine_fitness_ratio);

  const auto metrics = node["metrics"];
  config.metrics.inlier_distance_m = yaml_value(metrics, "inlier_distance_m", config.metrics.inlier_distance_m);
  config.metrics.overlap_max_distance_m = yaml_value(metrics, "overlap_max_distance_m", config.metrics.overlap_max_distance_m);
  config.metrics.volume_voxel_m = yaml_value(metrics, "volume_voxel_m", config.metrics.volume_voxel_m);

  if (config.candidate.esf_correlation_threshold < 0.60) {
    throw std::runtime_error("ESF correlation threshold must preserve the paper's >= 0.60 criterion");
  }
  if (config.grid.auto_scene_divisor <= 0.0 || config.grid.min_points < 20 ||
    config.visual_voxel_leaf_m <= 0.0 || config.lidar_voxel_leaf_m <= 0.0)
  {
    throw std::runtime_error("invalid positive preprocessing/grid configuration");
  }
  return config;
}

Dataset load_dataset(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node node = root["dataset"];
  if (!node) {
    throw std::runtime_error("dataset manifest requires a dataset root");
  }
  const std::filesystem::path parent = std::filesystem::absolute(path).parent_path();
  const auto resolve = [&parent](const std::string & value) {
      const std::filesystem::path candidate(value);
      return (candidate.is_absolute() ? candidate : parent / candidate).lexically_normal().string();
    };
  Dataset dataset;
  dataset.dataset_id = yaml_value(node, "id", std::string("unnamed"));
  dataset.visual_map_path = resolve(node["visual_map"].as<std::string>());
  dataset.lidar_map_path = resolve(node["lidar_map"].as<std::string>());
  dataset.visual_frame = yaml_value(node, "visual_frame", std::string("rtabmap_map"));
  dataset.lidar_frame = yaml_value(node, "lidar_frame", std::string("camera_init"));
  dataset.initial_lidar_to_visual = yaml_transform(node["initial_lidar_to_visual"], "initial_lidar_to_visual");
  if (node["truth_lidar_to_visual"]) {
    dataset.truth_lidar_to_visual = yaml_transform(
      node["truth_lidar_to_visual"], "truth_lidar_to_visual");
    dataset.has_truth = true;
  }
  return dataset;
}

CloudPtr load_cloud(const std::string & path)
{
  CloudPtr cloud(new CloudT);
  if (pcl::io::loadPCDFile<PointT>(path, *cloud) != 0) {
    throw std::runtime_error("failed to load PCD: " + path);
  }
  CloudPtr finite(new CloudT);
  finite->reserve(cloud->size());
  for (const auto & point : cloud->points) {
    if (pcl::isFinite(point)) {
      finite->push_back(point);
    }
  }
  finite->width = static_cast<std::uint32_t>(finite->size());
  finite->height = 1;
  finite->is_dense = true;
  if (finite->empty()) {
    throw std::runtime_error("PCD contains no finite points: " + path);
  }
  return finite;
}

CloudPtr voxel_downsample(const CloudPtr & cloud, double leaf_m)
{
  pcl::VoxelGrid<PointT> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(leaf_m, leaf_m, leaf_m);
  CloudPtr output(new CloudT);
  filter.filter(*output);
  return output;
}

CloudPtr transform_cloud(const CloudPtr & cloud, const Eigen::Isometry3d & transform)
{
  CloudPtr output(new CloudT);
  pcl::transformPointCloud(*cloud, *output, transform.matrix().cast<float>());
  return output;
}

CloudPtr extract_xy_boundary(const CloudPtr & cloud, const BoundaryConfig & config)
{
  CloudPtr output(new CloudT);
  if (!cloud || cloud->empty()) {
    return output;
  }
  std::vector<float> heights;
  heights.reserve(cloud->size());
  for (const auto & point : cloud->points) {
    if (pcl::isFinite(point)) {
      heights.push_back(point.z);
    }
  }
  if (heights.empty()) {
    return output;
  }
  const auto quantile_index = static_cast<std::size_t>(std::clamp(
      config.ground_quantile, 0.0, 0.95) * static_cast<double>(heights.size() - 1));
  std::nth_element(heights.begin(), heights.begin() + quantile_index, heights.end());
  const float ground = heights[quantile_index];

  using Cell = std::pair<int, int>;
  std::map<Cell, std::vector<const PointT *>> cells;
  for (const auto & point : cloud->points) {
    if (!pcl::isFinite(point) || point.z <= ground + config.ground_clearance_m) {
      continue;
    }
    const Cell cell{
      static_cast<int>(std::floor(point.x / config.raster_resolution_m)),
      static_cast<int>(std::floor(point.y / config.raster_resolution_m))};
    cells[cell].push_back(&point);
  }
  for (const auto & entry : cells) {
    int occupied_neighbors = 0;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        occupied_neighbors += cells.count(Cell{entry.first.first + dx, entry.first.second + dy}) > 0 ? 1 : 0;
      }
    }
    if (occupied_neighbors > config.max_occupied_neighbors) {
      continue;
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto * point : entry.second) {
      mean += Eigen::Vector3d(point->x, point->y, point->z);
    }
    mean /= static_cast<double>(entry.second.size());
    PointT boundary;
    boundary.x = static_cast<float>(mean.x());
    boundary.y = static_cast<float>(mean.y());
    boundary.z = 0.0F;
    boundary.r = entry.second.front()->r;
    boundary.g = entry.second.front()->g;
    boundary.b = entry.second.front()->b;
    output->push_back(boundary);
  }
  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = true;
  return output;
}

double pearson_correlation(const std::array<float, 640> & a, const std::array<float, 640> & b)
{
  const double mean_a = std::accumulate(a.begin(), a.end(), 0.0) / a.size();
  const double mean_b = std::accumulate(b.begin(), b.end(), 0.0) / b.size();
  double numerator = 0.0;
  double denominator_a = 0.0;
  double denominator_b = 0.0;
  for (std::size_t index = 0; index < a.size(); ++index) {
    const double da = a[index] - mean_a;
    const double db = b[index] - mean_b;
    numerator += da * db;
    denominator_a += da * da;
    denominator_b += db * db;
  }
  const double denominator = std::sqrt(denominator_a * denominator_b);
  return denominator <= 1e-12 ? 0.0 : numerator / denominator;
}

double rotation_distance_deg(const Eigen::Matrix3d & a, const Eigen::Matrix3d & b)
{
  const Eigen::AngleAxisd difference(a.transpose() * b);
  return degrees(std::abs(difference.angle()));
}

Eigen::Isometry3d average_transforms(const std::vector<Eigen::Isometry3d> & transforms)
{
  if (transforms.empty()) {
    throw std::runtime_error("cannot average an empty transformation set");
  }
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rotation(transforms.front().rotation());
  rotation.normalize();
  for (std::size_t index = 0; index < transforms.size(); ++index) {
    translation += transforms[index].translation();
    if (index == 0) {
      continue;
    }
    Eigen::Quaterniond candidate(transforms[index].rotation());
    candidate.normalize();
    if (rotation.dot(candidate) < 0.0) {
      candidate.coeffs() *= -1.0;
    }
    rotation = rotation.slerp(1.0 / static_cast<double>(index + 1), candidate).normalized();
  }
  Eigen::Isometry3d average = Eigen::Isometry3d::Identity();
  average.translation() = translation / static_cast<double>(transforms.size());
  average.linear() = rotation.toRotationMatrix();
  return average;
}

Metrics evaluate_alignment(
  const CloudPtr & visual, const CloudPtr & lidar,
  const Eigen::Isometry3d & estimate, const Eigen::Isometry3d & truth,
  const Config & config)
{
  Metrics metrics;
  metrics.translation_error_m = (estimate.translation() - truth.translation()).norm();
  metrics.rotation_error_deg = rotation_distance_deg(estimate.rotation(), truth.rotation());
  const CloudPtr aligned = transform_cloud(lidar, estimate);
  pcl::KdTreeFLANN<PointT> tree;
  tree.setInputCloud(visual);
  std::vector<int> indices(1);
  std::vector<float> squared(1);
  double total_distance = 0.0;
  double total_squared = 0.0;
  std::size_t inliers = 0;
  const double overlap_squared = config.metrics.overlap_max_distance_m *
    config.metrics.overlap_max_distance_m;
  const double inlier_squared = config.metrics.inlier_distance_m * config.metrics.inlier_distance_m;
  for (const auto & point : aligned->points) {
    if (tree.nearestKSearch(point, 1, indices, squared) == 0) {
      continue;
    }
    if (squared.front() <= overlap_squared) {
      total_distance += std::sqrt(squared.front());
      total_squared += squared.front();
      ++metrics.overlap_pairs;
    }
    if (squared.front() <= inlier_squared) {
      ++inliers;
    }
  }
  if (metrics.overlap_pairs > 0) {
    metrics.overlap_mean_nn_m = total_distance / static_cast<double>(metrics.overlap_pairs);
    metrics.overlap_rmse_m = std::sqrt(total_squared / static_cast<double>(metrics.overlap_pairs));
  } else {
    metrics.overlap_mean_nn_m = std::numeric_limits<double>::infinity();
    metrics.overlap_rmse_m = std::numeric_limits<double>::infinity();
  }
  metrics.inlier_ratio = aligned->empty() ? 0.0 :
    static_cast<double>(inliers) / static_cast<double>(aligned->size());

  const CloudPtr visual_boundary = extract_xy_boundary(visual, config.boundary);
  const CloudPtr lidar_boundary = extract_xy_boundary(aligned, config.boundary);
  metrics.boundary_mean_nn_m = cloud_nn_mean_distance(
    lidar_boundary, visual_boundary, config.metrics.overlap_max_distance_m);

  std::set<std::tuple<int, int, int>> visual_voxels;
  std::set<std::tuple<int, int, int>> union_voxels;
  for (const auto & point : visual->points) {
    const auto key = voxel_key(point, config.metrics.volume_voxel_m);
    visual_voxels.insert(key);
    union_voxels.insert(key);
  }
  for (const auto & point : aligned->points) {
    union_voxels.insert(voxel_key(point, config.metrics.volume_voxel_m));
  }
  metrics.supplement_voxel_growth_ratio = visual_voxels.empty() ? 0.0 :
    static_cast<double>(union_voxels.size() - visual_voxels.size()) /
    static_cast<double>(visual_voxels.size());
  return metrics;
}

RegistrationResult run_registration(
  const std::string & method, const CloudPtr & visual_input, const CloudPtr & lidar_input,
  const Dataset & dataset, const Config & config)
{
  const auto started = std::chrono::steady_clock::now();
  RegistrationResult result;
  result.method = method;
  result.registration_fitness = -1.0;
  const CloudPtr visual = voxel_downsample(visual_input, config.visual_voxel_leaf_m);
  const CloudPtr lidar = voxel_downsample(lidar_input, config.lidar_voxel_leaf_m);
  result.transform_lidar_to_visual = dataset.initial_lidar_to_visual;

  if (method == "initial") {
    result.converged = true;
    result.registration_fitness = cloud_nn_score(
      transform_cloud(lidar, result.transform_lidar_to_visual), visual,
      config.metrics.overlap_max_distance_m);
  } else if (method == "gicp") {
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setMaximumIterations(config.gicp.max_iterations);
    gicp.setMaxCorrespondenceDistance(config.gicp.max_correspondence_distance_m);
    gicp.setTransformationEpsilon(config.gicp.transformation_epsilon);
    gicp.setEuclideanFitnessEpsilon(config.gicp.fitness_epsilon);
    gicp.setInputSource(lidar);
    gicp.setInputTarget(visual);
    CloudT aligned;
    gicp.align(aligned, dataset.initial_lidar_to_visual.matrix().cast<float>());
    result.converged = gicp.hasConverged();
    result.registration_fitness = gicp.getFitnessScore(config.metrics.overlap_max_distance_m);
    result.transform_lidar_to_visual.matrix() = gicp.getFinalTransformation().cast<double>();
    if (!result.converged) {
      result.failure_reason = "GICP did not converge";
    }
  } else if (method == "hybrid") {
    const CloudPtr lidar_initial = transform_cloud(lidar, dataset.initial_lidar_to_visual);
    PointT visual_min, visual_max, lidar_min, lidar_max;
    pcl::getMinMax3D(*visual, visual_min, visual_max);
    pcl::getMinMax3D(*lidar_initial, lidar_min, lidar_max);
    const double origin_x = std::min(visual_min.x, lidar_min.x);
    const double origin_y = std::min(visual_min.y, lidar_min.y);
    const double scene_size = std::max({
      static_cast<double>(std::max(visual_max.x, lidar_max.x) - origin_x),
      static_cast<double>(std::max(visual_max.y, lidar_max.y) - origin_y), 1.0});
    const double cell_size = config.grid.cell_size_m > 0.0 ? config.grid.cell_size_m :
      scene_size / config.grid.auto_scene_divisor;
    const auto source_blocks = make_blocks(
      lidar_initial, origin_x, origin_y, cell_size, config.grid.min_points);
    const auto target_blocks = make_blocks(
      visual, origin_x, origin_y, cell_size, config.grid.min_points);
    result.source_blocks = static_cast<int>(source_blocks.size());
    result.target_blocks = static_cast<int>(target_blocks.size());

    std::vector<Match> matches;
    for (const auto & source_entry : source_blocks) {
      const auto & source = source_entry.second;
      const double source_radius = source.centroid.head<2>().norm();
      const double source_angle = std::atan2(source.centroid.y(), source.centroid.x());
      std::vector<Match> candidates;
      for (const auto & target_entry : target_blocks) {
        const auto & target = target_entry.second;
        const double target_radius = target.centroid.head<2>().norm();
        const double radial_difference = source_radius < 0.5 ?
          std::abs(target_radius - source_radius) :
          std::abs(target_radius / source_radius - 1.0);
        const double angle_difference = degrees(std::abs(wrap_angle(
            std::atan2(target.centroid.y(), target.centroid.x()) - source_angle)));
        const double centroid_distance =
          (source.centroid.head<2>() - target.centroid.head<2>()).norm();
        if (radial_difference > config.candidate.radial_scale_lambda ||
          angle_difference > config.candidate.angular_tolerance_deg ||
          centroid_distance > config.candidate.max_centroid_distance_m)
        {
          continue;
        }
        const double correlation = pearson_correlation(source.descriptor, target.descriptor);
        if (correlation >= config.candidate.esf_correlation_threshold) {
          candidates.push_back(Match{source.key, target.key, correlation, 0.0});
        }
      }
      std::sort(candidates.begin(), candidates.end(), [](const Match & left, const Match & right) {
          return left.descriptor_score > right.descriptor_score;
        });
      if (static_cast<int>(candidates.size()) > config.candidate.max_candidates_per_block) {
        candidates.resize(config.candidate.max_candidates_per_block);
      }
      matches.insert(matches.end(), candidates.begin(), candidates.end());
    }
    result.descriptor_candidates = static_cast<int>(matches.size());

    std::vector<Match> consistent_all;
    for (auto match : matches) {
      int comparisons = 0;
      match.neighbor_score = neighbor_consistency(
        source_blocks, target_blocks, match.source, match.target,
        config.grid.neighbor_radius_cells, comparisons);
      if (comparisons >= config.candidate.min_consistent_neighbors &&
        match.neighbor_score >= config.candidate.neighbor_correlation_threshold)
      {
        consistent_all.push_back(match);
      }
    }
    result.neighbor_consistent_candidates = static_cast<int>(consistent_all.size());

    // Multiple visual candidates for one source block are useful during
    // descriptor screening, but running NDT for every one makes runtime grow
    // quadratically.  Retain the best neighborhood-consistent hypothesis for
    // each source block, then apply the configured deterministic global cap.
    std::map<Key, Match> best_by_source;
    for (const auto & match : consistent_all) {
      const double score = match.descriptor_score + match.neighbor_score;
      const auto existing = best_by_source.find(match.source);
      if (existing == best_by_source.end() || score >
        existing->second.descriptor_score + existing->second.neighbor_score)
      {
        best_by_source[match.source] = match;
      }
    }
    std::vector<Match> consistent;
    consistent.reserve(best_by_source.size());
    for (const auto & entry : best_by_source) {
      consistent.push_back(entry.second);
    }
    std::sort(consistent.begin(), consistent.end(), [](const Match & left, const Match & right) {
        return left.descriptor_score + left.neighbor_score >
               right.descriptor_score + right.neighbor_score;
      });
    if (static_cast<int>(consistent.size()) > config.candidate.max_local_registrations) {
      consistent.resize(config.candidate.max_local_registrations);
    }

    std::vector<Eigen::Isometry3d> local_transforms;
    std::vector<double> local_fitness;
    for (const auto & match : consistent) {
      const CloudPtr source_local = collect_neighborhood(
        source_blocks, match.source, config.grid.neighbor_radius_cells);
      const CloudPtr target_local = collect_neighborhood(
        target_blocks, match.target, config.grid.neighbor_radius_cells);
      const CloudPtr source_boundary = extract_xy_boundary(source_local, config.boundary);
      const CloudPtr target_boundary = extract_xy_boundary(target_local, config.boundary);
      if (source_boundary->size() < config.boundary.min_boundary_points ||
        target_boundary->size() < config.boundary.min_boundary_points)
      {
        ++result.failed_blocks;
        continue;
      }
      double fitness_2d = 0.0;
      const auto [ok_2d, raw_2d] = run_ndt_2d(
        source_boundary, target_boundary, config.ndt_2d,
        Eigen::Isometry3d::Identity(), fitness_2d);
      if (!ok_2d) {
        ++result.failed_blocks;
        continue;
      }
      const Eigen::Isometry3d transform_2d = constrain_xy_yaw(raw_2d);
      const CloudPtr source_after_2d = transform_cloud(source_local, transform_2d);
      double fitness_3d = 0.0;
      const auto [ok_3d, transform_3d] = run_ndt(
        source_after_2d, target_local, config.ndt_3d,
        Eigen::Isometry3d::Identity(), fitness_3d);
      if (!ok_3d) {
        ++result.failed_blocks;
        continue;
      }
      local_transforms.push_back(
        transform_3d * transform_2d * dataset.initial_lidar_to_visual);
      local_fitness.push_back(fitness_3d);
      ++result.successful_blocks;
    }

    const auto cluster = largest_transform_cluster(local_transforms, config.cluster);
    result.selected_cluster_size = static_cast<int>(cluster.size());
    if (static_cast<int>(cluster.size()) < config.cluster.min_cluster_size) {
      result.converged = false;
      result.failure_reason = "no transformation cluster reached min_cluster_size";
    } else {
      std::vector<Eigen::Isometry3d> selected;
      double selected_fitness = 0.0;
      for (const auto index : cluster) {
        selected.push_back(local_transforms[index]);
        selected_fitness += local_fitness[index];
      }
      result.transform_lidar_to_visual = average_transforms(selected);
      result.registration_fitness = selected_fitness / static_cast<double>(selected.size());
      result.converged = true;

      if (config.cluster.enable_global_ndt_refine) {
        const double before = cloud_nn_score(
          transform_cloud(lidar, result.transform_lidar_to_visual), visual,
          config.ndt_3d.fitness_max_range_m);
        double refine_fitness = 0.0;
        const auto [refined, refine_transform] = run_ndt(
          lidar, visual, config.ndt_3d, result.transform_lidar_to_visual, refine_fitness);
        if (refined && refine_fitness <= before * config.cluster.max_refine_fitness_ratio) {
          result.transform_lidar_to_visual = refine_transform;
          result.registration_fitness = refine_fitness;
        }
      }
    }
  } else {
    throw std::runtime_error("method must be initial, gicp, or hybrid");
  }

  result.metrics = evaluate_alignment(
    visual, lidar, result.transform_lidar_to_visual,
    dataset.truth_lidar_to_visual, config);
  if (!dataset.has_truth) {
    result.metrics.translation_error_m = std::numeric_limits<double>::quiet_NaN();
    result.metrics.rotation_error_deg = std::numeric_limits<double>::quiet_NaN();
  }
  result.runtime_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  result.peak_rss_kib = peak_rss_kib();
  return result;
}

void write_result_artifacts(
  const std::string & output_dir, const RegistrationResult & result,
  const CloudPtr & visual, const CloudPtr & lidar,
  const Dataset & dataset, const Config & config)
{
  const std::filesystem::path directory(output_dir);
  std::filesystem::create_directories(directory);
  const CloudPtr aligned = transform_cloud(lidar, result.transform_lidar_to_visual);
  CloudPtr fused(new CloudT(*visual));
  *fused += *aligned;
  const CloudPtr fused_downsampled = voxel_downsample(fused, config.metrics.volume_voxel_m);
  if (pcl::io::savePCDFileBinaryCompressed((directory / "aligned_lidar.pcd").string(), *aligned) != 0 ||
    pcl::io::savePCDFileBinaryCompressed((directory / "fused_map.pcd").string(), *fused_downsampled) != 0)
  {
    throw std::runtime_error("failed to write output PCD artifacts");
  }

  const auto xyz_rpy = transform_xyz_rpy(result.transform_lidar_to_visual);
  std::ofstream transform_file(directory / "transform.yaml");
  transform_file << std::setprecision(12)
                 << "transform_lidar_to_visual:\n"
                 << "  parent_frame: " << dataset.visual_frame << "\n"
                 << "  child_frame: " << dataset.lidar_frame << "\n"
                 << "  xyz_rpy_rad: [";
  for (std::size_t index = 0; index < xyz_rpy.size(); ++index) {
    transform_file << (index == 0 ? "" : ", ") << xyz_rpy[index];
  }
  transform_file << "]\n  published_as_tf: false\n";

  std::ofstream manifest(directory / "run_manifest.yaml");
  manifest << "dataset_id: " << dataset.dataset_id << "\n"
           << "method: " << result.method << "\n"
           << "visual_map_input: " << dataset.visual_map_path << "\n"
           << "lidar_map_input: " << dataset.lidar_map_path << "\n"
           << "inputs_modified: false\n"
           << "outputs:\n"
           << "  aligned_lidar: aligned_lidar.pcd\n"
           << "  fused_map: fused_map.pcd\n"
           << "  transform: transform.yaml\n";

  std::ofstream json(directory / "result.json");
  json << std::boolalpha << std::setprecision(10)
       << "{\n"
       << "  \"dataset_id\": \"" << json_escape(dataset.dataset_id) << "\",\n"
       << "  \"ground_truth_available\": " << dataset.has_truth << ",\n"
       << "  \"method\": \"" << json_escape(result.method) << "\",\n"
       << "  \"converged\": " << result.converged << ",\n"
       << "  \"failure_reason\": \"" << json_escape(result.failure_reason) << "\",\n"
       << "  \"registration_fitness\": " << finite_json(result.registration_fitness) << ",\n"
       << "  \"runtime_ms\": " << finite_json(result.runtime_ms) << ",\n"
       << "  \"peak_rss_kib\": " << result.peak_rss_kib << ",\n"
       << "  \"blocks\": {\n"
       << "    \"source\": " << result.source_blocks << ",\n"
       << "    \"target\": " << result.target_blocks << ",\n"
       << "    \"descriptor_candidates\": " << result.descriptor_candidates << ",\n"
       << "    \"neighbor_consistent_candidates\": " << result.neighbor_consistent_candidates << ",\n"
       << "    \"successful\": " << result.successful_blocks << ",\n"
       << "    \"failed\": " << result.failed_blocks << ",\n"
       << "    \"selected_cluster_size\": " << result.selected_cluster_size << "\n"
       << "  },\n"
       << "  \"metrics\": {\n"
       << "    \"translation_error_m\": " << finite_json(result.metrics.translation_error_m) << ",\n"
       << "    \"rotation_error_deg\": " << finite_json(result.metrics.rotation_error_deg) << ",\n"
       << "    \"overlap_mean_nn_m\": " << finite_json(result.metrics.overlap_mean_nn_m) << ",\n"
       << "    \"overlap_rmse_m\": " << finite_json(result.metrics.overlap_rmse_m) << ",\n"
       << "    \"boundary_mean_nn_m\": " << finite_json(result.metrics.boundary_mean_nn_m) << ",\n"
       << "    \"inlier_ratio\": " << finite_json(result.metrics.inlier_ratio) << ",\n"
       << "    \"supplement_voxel_growth_ratio\": " << finite_json(result.metrics.supplement_voxel_growth_ratio) << ",\n"
       << "    \"overlap_pairs\": " << result.metrics.overlap_pairs << "\n"
       << "  },\n"
       << "  \"transform_lidar_to_visual_xyz_rpy_rad\": [";
  for (std::size_t index = 0; index < xyz_rpy.size(); ++index) {
    json << (index == 0 ? "" : ", ") << finite_json(xyz_rpy[index]);
  }
  json << "]\n}\n";
}

}  // namespace hybridfusion_map_fusion
