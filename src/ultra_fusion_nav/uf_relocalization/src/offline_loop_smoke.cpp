#include "uf_relocalization/descriptor_core.hpp"
#include "uf_relocalization/keyframe_database.hpp"
#include "uf_relocalization/offline_esf_seed.hpp"
#include "uf_relocalization/offline_loop_edge.hpp"
#include "uf_relocalization/registration_core.hpp"

#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

struct KeyframeRow
{
  std::size_t keyframe_id{0};
  std::size_t scan_id{0};
  double stamp_s{0.0};
  Eigen::Isometry3d map_from_body{Eigen::Isometry3d::Identity()};
  std::filesystem::path descriptor_pcd;
  std::filesystem::path map_pcd;
};

std::vector<std::string> split_csv(const std::string & line)
{
  std::vector<std::string> values;
  std::stringstream stream(line);
  std::string value;
  while (std::getline(stream, value, ',')) {
    values.push_back(value);
  }
  return values;
}

std::vector<KeyframeRow> read_metadata(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open keyframe metadata: " + path.string());
  }
  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("keyframe metadata is empty");
  }
  const auto header = split_csv(line);
  std::unordered_map<std::string, std::size_t> columns;
  for (std::size_t index = 0; index < header.size(); ++index) {
    columns.emplace(header[index], index);
  }
  const std::vector<std::string> required{
    "keyframe_id", "scan_id", "stamp_ns", "tx", "ty", "tz",
    "qx", "qy", "qz", "qw", "descriptor_pcd", "map_pcd"};
  for (const auto & name : required) {
    if (columns.count(name) == 0U) {
      throw std::runtime_error("metadata column missing: " + name);
    }
  }
  std::vector<KeyframeRow> rows;
  const auto root = path.parent_path();
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    const auto values = split_csv(line);
    if (values.size() != header.size()) {
      throw std::runtime_error("metadata row width mismatch");
    }
    KeyframeRow row;
    row.keyframe_id = static_cast<std::size_t>(std::stoull(values[columns.at("keyframe_id")]));
    row.scan_id = static_cast<std::size_t>(std::stoull(values[columns.at("scan_id")]));
    row.stamp_s = std::stod(values[columns.at("stamp_ns")]) * 1.0e-9;
    row.map_from_body.translation() = Eigen::Vector3d{
      std::stod(values[columns.at("tx")]),
      std::stod(values[columns.at("ty")]),
      std::stod(values[columns.at("tz")])};
    Eigen::Quaterniond quaternion{
      std::stod(values[columns.at("qw")]),
      std::stod(values[columns.at("qx")]),
      std::stod(values[columns.at("qy")]),
      std::stod(values[columns.at("qz")])};
    if (!std::isfinite(row.stamp_s) || !quaternion.coeffs().allFinite() ||
      quaternion.norm() <= 1.0e-12 || !row.map_from_body.translation().allFinite())
    {
      throw std::runtime_error("metadata contains invalid pose or timestamp");
    }
    row.map_from_body.linear() = quaternion.normalized().toRotationMatrix();
    row.descriptor_pcd = root / values[columns.at("descriptor_pcd")];
    row.map_pcd = root / values[columns.at("map_pcd")];
    rows.push_back(row);
  }
  if (rows.size() < 2U) {
    throw std::runtime_error("loop smoke requires at least two keyframes");
  }
  return rows;
}

uf_relocalization::Cloud::Ptr load_cloud(
  const std::filesystem::path & path, const float voxel_size_m)
{
  auto raw = std::make_shared<uf_relocalization::Cloud>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path.string(), *raw) != 0) {
    throw std::runtime_error("cannot load PCD: " + path.string());
  }
  auto output = std::make_shared<uf_relocalization::Cloud>();
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(voxel_size_m, voxel_size_m, voxel_size_m);
  filter.setInputCloud(raw);
  filter.filter(*output);
  if (output->size() < 100U) {
    throw std::runtime_error("downsampled keyframe has fewer than 100 points");
  }
  return output;
}

uf_relocalization::KeyframeQuality healthy_quality()
{
  uf_relocalization::KeyframeQuality quality;
  quality.map_quality = 1.0;
  quality.feature_repeatability = 1.0;
  quality.dynamic_ratio = 0.0;
  quality.lidar_degradation = 0.0;
  quality.scheduler_lidar_enabled = true;
  return quality;
}

struct BestResult
{
  bool accepted{false};
  std::size_t query_id{0};
  std::size_t candidate_id{0};
  double temporal_separation_s{0.0};
  double descriptor_distance{std::numeric_limits<double>::infinity()};
  uf_relocalization::RegistrationResult registration;
};

void write_json(
  const std::filesystem::path & output, const std::size_t keyframes,
  const std::size_t retrieved, const std::size_t verified,
  const std::uint32_t descriptor_seed,
  const BestResult & best,
  const std::vector<uf_relocalization::OfflineLoopEdge> & verified_edges)
{
  std::ofstream stream(output);
  if (!stream) {
    throw std::runtime_error("cannot write loop smoke output");
  }
  stream << std::setprecision(12) << "{\n"
         << "  \"schema_version\": 2,\n"
         << "  \"measurement_convention\": \"candidate_from_query\",\n"
         << "  \"descriptor_sampling\": \"pcl_esf_fixed_seed_offline_only\",\n"
         << "  \"descriptor_sampling_seed\": " << descriptor_seed << ",\n"
         << "  \"keyframes\": " << keyframes << ",\n"
         << "  \"retrieved_candidates\": " << retrieved << ",\n"
         << "  \"geometrically_verified\": " << verified << ",\n"
         << "  \"accepted\": " << (best.accepted ? "true" : "false") << ",\n"
         << "  \"query_keyframe\": " << best.query_id << ",\n"
         << "  \"candidate_keyframe\": " << best.candidate_id << ",\n"
         << "  \"temporal_separation_s\": " << best.temporal_separation_s << ",\n"
         << "  \"descriptor_distance\": " << best.descriptor_distance << ",\n"
         << "  \"registration_converged\": " << (best.registration.converged ? "true" : "false") << ",\n"
         << "  \"correspondence_points\": " << best.registration.correspondence_points << ",\n"
         << "  \"overlap_ratio\": " << best.registration.overlap_ratio << ",\n"
         << "  \"reciprocal_ratio\": " << best.registration.reciprocal_ratio << ",\n"
         << "  \"inlier_rmse_m\": " << best.registration.inlier_rmse << ",\n"
         << "  \"absolute_plane_error_p90_m\": " << best.registration.absolute_plane_error_p90_m << ",\n"
         << "  \"effective_rank\": " << best.registration.effective_rank << ",\n"
         << "  \"condition_number\": " << best.registration.condition_number << ",\n"
         << "  \"target_from_source\": [";
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      if (row != 0 || column != 0) {
        stream << ", ";
      }
      stream << best.registration.target_from_source(row, column);
    }
  }
  stream << "],\n"
         << "  \"verified_edges\": [";
  for (std::size_t edge_index = 0; edge_index < verified_edges.size(); ++edge_index) {
    const auto & edge = verified_edges[edge_index];
    if (edge_index != 0U) {
      stream << ",";
    }
    stream << "\n    {\n"
           << "      \"candidate_keyframe\": " << edge.candidate_keyframe << ",\n"
           << "      \"query_keyframe\": " << edge.query_keyframe << ",\n"
           << "      \"candidate_stamp_s\": " << edge.candidate_stamp_s << ",\n"
           << "      \"query_stamp_s\": " << edge.query_stamp_s << ",\n"
           << "      \"temporal_separation_s\": " << edge.temporal_separation_s << ",\n"
           << "      \"descriptor_distance\": " << edge.descriptor_distance << ",\n"
           << "      \"correspondence_points\": " << edge.registration.correspondence_points << ",\n"
           << "      \"overlap_ratio\": " << edge.registration.overlap_ratio << ",\n"
           << "      \"reciprocal_ratio\": " << edge.registration.reciprocal_ratio << ",\n"
           << "      \"inlier_rmse_m\": " << edge.registration.inlier_rmse << ",\n"
           << "      \"absolute_plane_error_p90_m\": " <<
      edge.registration.absolute_plane_error_p90_m << ",\n"
           << "      \"effective_rank\": " << edge.registration.effective_rank << ",\n"
           << "      \"condition_number\": " << edge.registration.condition_number << ",\n"
           << "      \"candidate_from_query\": [";
    for (int row = 0; row < 4; ++row) {
      for (int column = 0; column < 4; ++column) {
        if (row != 0 || column != 0) {
          stream << ", ";
        }
        stream << edge.candidate_from_query.matrix()(row, column);
      }
    }
    stream << "]\n    }";
  }
  if (!verified_edges.empty()) {
    stream << "\n  ";
  }
  stream << "]\n}\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    std::filesystem::path metadata;
    std::filesystem::path output;
    double minimum_separation_s = 20.0;
    std::size_t maximum_candidates = 5U;
    std::size_t exclude_recent = 3U;
    std::uint32_t descriptor_seed = 1731U;
    float voxel_size_m = 0.15F;
    for (int index = 1; index < argc; ++index) {
      const std::string argument(argv[index]);
      if (index + 1 >= argc) {
        throw std::runtime_error("every option requires a value");
      }
      const std::string value(argv[++index]);
      if (argument == "--metadata") {
        metadata = value;
      } else if (argument == "--output") {
        output = value;
      } else if (argument == "--minimum-separation-s") {
        minimum_separation_s = std::stod(value);
      } else if (argument == "--maximum-candidates") {
        maximum_candidates = static_cast<std::size_t>(std::stoull(value));
      } else if (argument == "--exclude-recent") {
        exclude_recent = static_cast<std::size_t>(std::stoull(value));
      } else if (argument == "--descriptor-seed") {
        descriptor_seed = static_cast<std::uint32_t>(std::stoul(value));
      } else if (argument == "--voxel-size-m") {
        voxel_size_m = std::stof(value);
      } else {
        throw std::runtime_error("unknown option: " + argument);
      }
    }
    if (metadata.empty() || output.empty() || minimum_separation_s <= 0.0 ||
      maximum_candidates == 0U || voxel_size_m <= 0.0F)
    {
      throw std::runtime_error("invalid or missing command-line options");
    }

    const auto rows = read_metadata(metadata);
    uf_relocalization::set_offline_esf_seed(descriptor_seed);
    uf_relocalization::KeyframeDatabaseConfig database_config;
    database_config.minimum_translation_spacing_m = 0.0;
    database_config.minimum_rotation_spacing_rad = 0.0;
    database_config.maximum_keyframes = rows.size() + 1U;
    uf_relocalization::StaticKeyframeDatabase database(database_config);
    std::unordered_map<std::size_t, double> candidate_stamps;
    std::size_t retrieved = 0U;
    std::size_t verified = 0U;
    std::vector<uf_relocalization::OfflineLoopEdge> verified_edges;
    BestResult best;
    best.registration.inlier_rmse = std::numeric_limits<double>::infinity();

    uf_relocalization::RegistrationConfig registration_config;
    registration_config.maximum_correspondence_distance_m = 0.8;
    registration_config.maximum_iterations = 60;
    registration_config.normal_k_neighbors = 20;

    for (const auto & row : rows) {
      const auto descriptor_cloud = load_cloud(row.descriptor_pcd, voxel_size_m);
      const auto descriptor = uf_relocalization::compute_esf_descriptor(descriptor_cloud);
      for (const auto & candidate : database.query(descriptor, maximum_candidates, exclude_recent)) {
        ++retrieved;
        const auto * keyframe = database.find(candidate.keyframe_id);
        if (keyframe == nullptr) {
          continue;
        }
        const double separation = row.stamp_s - candidate_stamps.at(candidate.keyframe_id);
        if (separation < minimum_separation_s) {
          continue;
        }
        const auto result = uf_relocalization::align_point_to_plane(
          descriptor_cloud, keyframe->cloud,
          row.map_from_body.matrix().cast<float>(), registration_config);
        const bool accepted = result.converged && result.correspondence_points >= 100U &&
          result.overlap_ratio >= 0.35 && result.reciprocal_ratio >= 0.15 &&
          result.inlier_rmse <= 0.20 && result.effective_rank >= 5;
        if (accepted) {
          ++verified;
          verified_edges.push_back(uf_relocalization::make_offline_loop_edge(
            candidate.keyframe_id, row.keyframe_id,
            candidate_stamps.at(candidate.keyframe_id), row.stamp_s,
            keyframe->world_from_sensor, candidate.descriptor_distance, result));
        }
        if (accepted && (!best.accepted || result.inlier_rmse < best.registration.inlier_rmse)) {
          best.accepted = true;
          best.query_id = row.keyframe_id;
          best.candidate_id = candidate.keyframe_id;
          best.temporal_separation_s = separation;
          best.descriptor_distance = candidate.descriptor_distance;
          best.registration = result;
        }
      }
      const auto map_cloud = load_cloud(row.map_pcd, voxel_size_m);
      const auto admission = database.try_insert(
        row.stamp_s, row.map_from_body, map_cloud, descriptor, healthy_quality());
      if (!admission.accepted) {
        throw std::runtime_error("keyframe admission failed: " + admission.reason);
      }
      candidate_stamps[admission.keyframe_id] = row.stamp_s;
    }
    write_json(
      output, rows.size(), retrieved, verified, descriptor_seed, best, verified_edges);
    std::cout << output << std::endl;
    return best.accepted ? 0 : 2;
  } catch (const std::exception & error) {
    std::cerr << "offline_loop_smoke: " << error.what() << std::endl;
    return 1;
  }
}
