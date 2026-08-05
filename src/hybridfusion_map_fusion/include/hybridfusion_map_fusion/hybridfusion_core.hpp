#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cstddef>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace hybridfusion_map_fusion
{

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;

struct GridConfig
{
  double cell_size_m{0.0};
  double auto_scene_divisor{10.0};
  std::size_t min_points{80};
  int neighbor_radius_cells{1};
};

struct CandidateConfig
{
  double radial_scale_lambda{0.35};
  double angular_tolerance_deg{55.0};
  double max_centroid_distance_m{4.5};
  double esf_correlation_threshold{0.60};
  double neighbor_correlation_threshold{0.52};
  int min_consistent_neighbors{1};
  int max_candidates_per_block{6};
  int max_local_registrations{18};
};

struct BoundaryConfig
{
  double ground_quantile{0.12};
  double ground_clearance_m{0.18};
  double raster_resolution_m{0.18};
  int max_occupied_neighbors{7};
  std::size_t min_boundary_points{24};
};

struct NdtConfig
{
  double resolution_m{0.55};
  double step_size_m{0.20};
  double transformation_epsilon{0.001};
  int max_iterations{50};
  double max_fitness{1.50};
  double fitness_max_range_m{2.0};
};

struct GicpConfig
{
  int max_iterations{80};
  double max_correspondence_distance_m{2.0};
  double transformation_epsilon{1e-5};
  double fitness_epsilon{1e-5};
};

struct ClusterConfig
{
  double translation_threshold_m{0.65};
  double rotation_threshold_deg{9.0};
  int min_cluster_size{2};
  bool enable_global_ndt_refine{true};
  double max_refine_fitness_ratio{1.08};
};

struct MetricConfig
{
  double inlier_distance_m{0.35};
  double overlap_max_distance_m{1.2};
  double volume_voxel_m{0.20};
};

struct Config
{
  double visual_voxel_leaf_m{0.12};
  double lidar_voxel_leaf_m{0.12};
  GridConfig grid;
  CandidateConfig candidate;
  BoundaryConfig boundary;
  NdtConfig ndt_2d;
  NdtConfig ndt_3d;
  GicpConfig gicp;
  ClusterConfig cluster;
  MetricConfig metrics;
};

struct Dataset
{
  std::string visual_map_path;
  std::string lidar_map_path;
  std::string visual_frame;
  std::string lidar_frame;
  std::string dataset_id;
  Eigen::Isometry3d initial_lidar_to_visual{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d truth_lidar_to_visual{Eigen::Isometry3d::Identity()};
  bool has_truth{false};
};

struct Metrics
{
  double translation_error_m{0.0};
  double rotation_error_deg{0.0};
  double overlap_mean_nn_m{0.0};
  double overlap_rmse_m{0.0};
  double boundary_mean_nn_m{0.0};
  double inlier_ratio{0.0};
  double supplement_voxel_growth_ratio{0.0};
  std::size_t overlap_pairs{0};
};

struct RegistrationResult
{
  std::string method;
  bool converged{false};
  std::string failure_reason;
  Eigen::Isometry3d transform_lidar_to_visual{Eigen::Isometry3d::Identity()};
  double registration_fitness{0.0};
  double runtime_ms{0.0};
  long peak_rss_kib{0};
  int source_blocks{0};
  int target_blocks{0};
  int descriptor_candidates{0};
  int neighbor_consistent_candidates{0};
  int successful_blocks{0};
  int failed_blocks{0};
  int selected_cluster_size{0};
  Metrics metrics;
};

Config load_config(const std::string & path);
Dataset load_dataset(const std::string & path);
CloudPtr load_cloud(const std::string & path);
CloudPtr voxel_downsample(const CloudPtr & cloud, double leaf_m);
CloudPtr transform_cloud(const CloudPtr & cloud, const Eigen::Isometry3d & transform);
CloudPtr extract_xy_boundary(const CloudPtr & cloud, const BoundaryConfig & config);
double pearson_correlation(const std::array<float, 640> & a, const std::array<float, 640> & b);
Eigen::Isometry3d average_transforms(const std::vector<Eigen::Isometry3d> & transforms);
double rotation_distance_deg(const Eigen::Matrix3d & a, const Eigen::Matrix3d & b);
Metrics evaluate_alignment(
  const CloudPtr & visual, const CloudPtr & lidar,
  const Eigen::Isometry3d & estimate, const Eigen::Isometry3d & truth,
  const Config & config);
RegistrationResult run_registration(
  const std::string & method, const CloudPtr & visual, const CloudPtr & lidar,
  const Dataset & dataset, const Config & config);
void write_result_artifacts(
  const std::string & output_dir, const RegistrationResult & result,
  const CloudPtr & visual, const CloudPtr & lidar,
  const Dataset & dataset, const Config & config);

Eigen::Isometry3d xyz_rpy_transform(const std::array<double, 6> & values);
std::array<double, 6> transform_xyz_rpy(const Eigen::Isometry3d & transform);

}  // namespace hybridfusion_map_fusion
