#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <array>
#include <string>

namespace uf_relocalization
{

struct RegistrationConfig
{
  double maximum_correspondence_distance_m{1.5};
  int maximum_iterations{80};
  double transformation_epsilon{1.0e-8};
  double euclidean_fitness_epsilon{1.0e-7};
  double ndt_resolution_m{0.6};
  double ndt_step_size{0.1};
  int normal_k_neighbors{20};
};

struct RegistrationResult
{
  bool converged{false};
  double fitness{0.0};
  Eigen::Matrix4f target_from_source{Eigen::Matrix4f::Identity()};
  std::size_t source_points{0};
  std::size_t target_points{0};
  std::size_t correspondence_points{0};
  double overlap_ratio{0.0};
  std::size_t reciprocal_correspondence_points{0};
  // Fraction of forward correspondences whose target nearest neighbor maps
  // back to the same transformed source point. This does not use target size,
  // so unseen portions of a rolling submap do not penalize a single scan.
  double reciprocal_ratio{0.0};
  // Method-specific residual used by the registration gate. For point-to-
  // plane ICP this is the normal-direction RMSE.
  double inlier_rmse{0.0};
  double absolute_plane_error_p90_m{0.0};
  double euclidean_inlier_rmse{0.0};
  int effective_rank{0};
  double condition_number{0.0};
  std::array<double, 6> normalized_information_eigenvalues{};
  std::string method;
};

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

RegistrationResult align_icp(
  const Cloud::ConstPtr & source,
  const Cloud::ConstPtr & target,
  const Eigen::Matrix4f & initial_target_from_source,
  const RegistrationConfig & config = RegistrationConfig{});

RegistrationResult align_gicp(
  const Cloud::ConstPtr & source,
  const Cloud::ConstPtr & target,
  const Eigen::Matrix4f & initial_target_from_source,
  const RegistrationConfig & config = RegistrationConfig{});

RegistrationResult align_point_to_plane(
  const Cloud::ConstPtr & source,
  const Cloud::ConstPtr & target,
  const Eigen::Matrix4f & initial_target_from_source,
  const RegistrationConfig & config = RegistrationConfig{});

RegistrationResult align_ndt(
  const Cloud::ConstPtr & source,
  const Cloud::ConstPtr & target,
  const Eigen::Matrix4f & initial_target_from_source,
  const RegistrationConfig & config = RegistrationConfig{});

}  // namespace uf_relocalization
