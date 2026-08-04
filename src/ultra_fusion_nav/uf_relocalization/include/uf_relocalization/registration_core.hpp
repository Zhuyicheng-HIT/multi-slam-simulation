#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
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
};

struct RegistrationResult
{
  bool converged{false};
  double fitness{0.0};
  Eigen::Matrix4f target_from_source{Eigen::Matrix4f::Identity()};
  std::size_t source_points{0};
  std::size_t target_points{0};
  std::string method;
};

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

RegistrationResult align_icp(
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
