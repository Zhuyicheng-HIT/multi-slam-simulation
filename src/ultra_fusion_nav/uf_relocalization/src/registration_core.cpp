#include "uf_relocalization/registration_core.hpp"

#include <pcl/common/transforms.h>
#include <pcl/common/point_tests.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace uf_relocalization
{
namespace
{

Cloud::Ptr finite_cloud(const Cloud::ConstPtr & input, const char * label)
{
  if (!input) {
    throw std::invalid_argument(std::string(label) + " cloud is null");
  }
  auto output = std::make_shared<Cloud>();
  output->reserve(input->size());
  for (const auto & point : *input) {
    if (pcl::isFinite(point)) {
      output->push_back(point);
    }
  }
  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1U;
  output->is_dense = true;
  if (output->size() < 3U) {
    throw std::invalid_argument(std::string(label) + " cloud has fewer than three finite points");
  }
  return output;
}

int spatial_rank(const Cloud::ConstPtr & cloud)
{
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto & point : *cloud) {
    mean += Eigen::Vector3d(point.x, point.y, point.z);
  }
  mean /= static_cast<double>(cloud->size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto & point : *cloud) {
    const Eigen::Vector3d delta = Eigen::Vector3d(point.x, point.y, point.z) - mean;
    covariance.noalias() += delta * delta.transpose();
  }
  covariance /= static_cast<double>(cloud->size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    return 0;
  }
  const double maximum = solver.eigenvalues().maxCoeff();
  if (maximum <= 1.0e-12) {
    return 0;
  }
  const double threshold = std::max(1.0e-12, maximum * 1.0e-5);
  int rank = 0;
  for (const double eigenvalue : solver.eigenvalues()) {
    rank += eigenvalue > threshold ? 1 : 0;
  }
  return rank;
}

void validate_spatial_rank(
  const Cloud::ConstPtr & source,
  const Cloud::ConstPtr & target,
  const int minimum_rank,
  const char * method)
{
  if (spatial_rank(source) < minimum_rank || spatial_rank(target) < minimum_rank) {
    throw std::invalid_argument(
            std::string(method) + " clouds do not have sufficient spatial rank");
  }
}

void validate_inputs(
  const Cloud::ConstPtr & source,
  const Cloud::ConstPtr & target,
  const RegistrationConfig & config)
{
  if (!source || !target || source->empty() || target->empty()) {
    throw std::invalid_argument("registration clouds must be non-empty");
  }
  if (config.maximum_correspondence_distance_m <= 0.0 || config.maximum_iterations < 1) {
    throw std::invalid_argument("registration limits must be positive");
  }
  if (config.ndt_resolution_m <= 0.0 || config.ndt_step_size <= 0.0 ||
    config.normal_k_neighbors < 3)
  {
    throw std::invalid_argument("registration resolutions and neighborhood must be positive");
  }
}

pcl::PointCloud<pcl::PointNormal>::Ptr estimate_normals(
  const Cloud::ConstPtr & cloud, const int k_neighbors)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> estimation;
  estimation.setInputCloud(cloud);
  estimation.setSearchMethod(
    pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>());
  estimation.setKSearch(k_neighbors);
  pcl::PointCloud<pcl::Normal> normals;
  estimation.compute(normals);

  auto output = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  output->reserve(cloud->size());
  for (std::size_t index = 0; index < cloud->size() && index < normals.size(); ++index) {
    const auto & point = (*cloud)[index];
    const auto & normal = normals[index];
    Eigen::Vector3f direction(normal.normal_x, normal.normal_y, normal.normal_z);
    if (!direction.allFinite() || direction.norm() <= 1.0e-6F) {
      continue;
    }
    direction.normalize();
    pcl::PointNormal point_normal;
    point_normal.x = point.x;
    point_normal.y = point.y;
    point_normal.z = point.z;
    point_normal.normal_x = direction.x();
    point_normal.normal_y = direction.y();
    point_normal.normal_z = direction.z();
    point_normal.curvature = normal.curvature;
    output->push_back(point_normal);
  }
  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1U;
  output->is_dense = true;
  if (output->size() < 6U) {
    throw std::invalid_argument("target cloud has insufficient finite normals");
  }
  return output;
}

RegistrationResult base_result(
  const Cloud::ConstPtr & source,
  const Cloud::ConstPtr & target,
  const char * method)
{
  RegistrationResult result;
  result.fitness = std::numeric_limits<double>::infinity();
  result.inlier_rmse = std::numeric_limits<double>::infinity();
  result.absolute_plane_error_p90_m = std::numeric_limits<double>::infinity();
  result.euclidean_inlier_rmse = std::numeric_limits<double>::infinity();
  result.condition_number = std::numeric_limits<double>::infinity();
  result.source_points = source->size();
  result.target_points = target->size();
  result.method = method;
  return result;
}

void characterize_alignment(
  RegistrationResult & result,
  const Cloud::ConstPtr & source,
  const Cloud::ConstPtr & target,
  const double maximum_correspondence_distance_m)
{
  if (!result.target_from_source.allFinite()) {
    return;
  }
  Cloud transformed;
  pcl::transformPointCloud(*source, transformed, result.target_from_source);
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target);
  const float maximum_squared_distance = static_cast<float>(
    maximum_correspondence_distance_m * maximum_correspondence_distance_m);
  double squared_error_sum = 0.0;
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  for (const auto & point : transformed) {
    if (!pcl::isFinite(point) ||
      tree.nearestKSearch(point, 1, indices, squared_distances) != 1 ||
      squared_distances[0] > maximum_squared_distance)
    {
      continue;
    }
    ++result.correspondence_points;
    squared_error_sum += squared_distances[0];
  }
  result.overlap_ratio = static_cast<double>(result.correspondence_points) /
    static_cast<double>(std::max<std::size_t>(1U, source->size()));
  if (result.correspondence_points > 0U) {
    result.inlier_rmse = std::sqrt(
      squared_error_sum / static_cast<double>(result.correspondence_points));
    result.euclidean_inlier_rmse = result.inlier_rmse;
  }
}

void characterize_point_to_plane_alignment(
  RegistrationResult & result,
  const Cloud::ConstPtr & source,
  const pcl::PointCloud<pcl::PointNormal>::ConstPtr & target,
  const double maximum_correspondence_distance_m)
{
  if (!result.target_from_source.allFinite()) {
    return;
  }
  Cloud transformed;
  pcl::transformPointCloud(*source, transformed, result.target_from_source);
  pcl::KdTreeFLANN<pcl::PointNormal> tree;
  tree.setInputCloud(target);
  const auto transformed_cloud = std::make_shared<Cloud>(transformed);
  pcl::KdTreeFLANN<pcl::PointXYZ> transformed_tree;
  transformed_tree.setInputCloud(transformed_cloud);
  const float maximum_squared_distance = static_cast<float>(
    maximum_correspondence_distance_m * maximum_correspondence_distance_m);
  double euclidean_squared_sum = 0.0;
  double plane_squared_sum = 0.0;
  std::vector<double> absolute_plane_errors;
  absolute_plane_errors.reserve(transformed.size());
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  std::vector<int> reciprocal_indices(1);
  std::vector<float> reciprocal_squared_distances(1);
  for (std::size_t point_index = 0U; point_index < transformed.size(); ++point_index) {
    const auto & point = transformed[point_index];
    if (!pcl::isFinite(point)) {
      continue;
    }
    pcl::PointNormal query;
    query.x = point.x;
    query.y = point.y;
    query.z = point.z;
    if (tree.nearestKSearch(query, 1, indices, squared_distances) != 1 ||
      squared_distances[0] > maximum_squared_distance)
    {
      continue;
    }
    const auto & match = (*target)[static_cast<std::size_t>(indices[0])];
    Eigen::Vector3d normal(match.normal_x, match.normal_y, match.normal_z);
    if (!normal.allFinite() || normal.norm() <= 1.0e-9) {
      continue;
    }
    normal.normalize();
    const Eigen::Vector3d transformed_point(point.x, point.y, point.z);
    const Eigen::Vector3d matched_point(match.x, match.y, match.z);
    const double plane_error = normal.dot(transformed_point - matched_point);
    Eigen::Matrix<double, 6, 1> jacobian;
    jacobian.head<3>() = transformed_point.cross(normal);
    jacobian.tail<3>() = normal;
    information.noalias() += jacobian * jacobian.transpose();
    ++result.correspondence_points;
    pcl::PointXYZ matched_query;
    matched_query.x = match.x;
    matched_query.y = match.y;
    matched_query.z = match.z;
    if (transformed_tree.nearestKSearch(
        matched_query, 1, reciprocal_indices, reciprocal_squared_distances) == 1 &&
      reciprocal_indices[0] == static_cast<int>(point_index))
    {
      ++result.reciprocal_correspondence_points;
    }
    euclidean_squared_sum += squared_distances[0];
    plane_squared_sum += plane_error * plane_error;
    absolute_plane_errors.push_back(std::abs(plane_error));
  }
  result.overlap_ratio = static_cast<double>(result.correspondence_points) /
    static_cast<double>(std::max<std::size_t>(1U, source->size()));
  if (result.correspondence_points == 0U) {
    return;
  }
  result.inlier_rmse = std::sqrt(
    plane_squared_sum / static_cast<double>(result.correspondence_points));
  result.reciprocal_ratio =
    static_cast<double>(result.reciprocal_correspondence_points) /
    static_cast<double>(result.correspondence_points);
  result.euclidean_inlier_rmse = std::sqrt(
    euclidean_squared_sum / static_cast<double>(result.correspondence_points));
  std::sort(absolute_plane_errors.begin(), absolute_plane_errors.end());
  const std::size_t p90_index = std::min(
    absolute_plane_errors.size() - 1U,
    static_cast<std::size_t>(
      std::ceil(0.90 * static_cast<double>(absolute_plane_errors.size()))) - 1U);
  result.absolute_plane_error_p90_m = absolute_plane_errors[p90_index];

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(information);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    return;
  }
  const double maximum = solver.eigenvalues().maxCoeff();
  if (maximum <= 1.0e-12) {
    return;
  }
  const double threshold = maximum * 1.0e-6;
  double minimum_observable = std::numeric_limits<double>::infinity();
  for (int index = 0; index < 6; ++index) {
    const double eigenvalue = solver.eigenvalues()[index];
    result.normalized_information_eigenvalues[static_cast<std::size_t>(index)] =
      eigenvalue / maximum;
    if (eigenvalue > threshold) {
      ++result.effective_rank;
      minimum_observable = std::min(minimum_observable, eigenvalue);
    }
  }
  if (result.effective_rank > 0 && std::isfinite(minimum_observable)) {
    result.condition_number = maximum / minimum_observable;
  }
}

}  // namespace

RegistrationResult align_icp(
  const Cloud::ConstPtr & source,
  const Cloud::ConstPtr & target,
  const Eigen::Matrix4f & initial_target_from_source,
  const RegistrationConfig & config)
{
  const auto clean_source = finite_cloud(source, "source");
  const auto clean_target = finite_cloud(target, "target");
  validate_inputs(clean_source, clean_target, config);
  validate_spatial_rank(clean_source, clean_target, 2, "ICP");
  if (!initial_target_from_source.allFinite()) {
    throw std::invalid_argument("ICP initial transform must be finite");
  }
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;
  registration.setInputSource(clean_source);
  registration.setInputTarget(clean_target);
  registration.setMaxCorrespondenceDistance(config.maximum_correspondence_distance_m);
  registration.setMaximumIterations(config.maximum_iterations);
  registration.setTransformationEpsilon(config.transformation_epsilon);
  registration.setEuclideanFitnessEpsilon(config.euclidean_fitness_epsilon);

  Cloud aligned;
  registration.align(aligned, initial_target_from_source);
  auto result = base_result(clean_source, clean_target, "icp");
  result.converged = registration.hasConverged();
  result.fitness = registration.getFitnessScore(config.maximum_correspondence_distance_m);
  result.target_from_source = registration.getFinalTransformation();
  characterize_alignment(
    result, clean_source, clean_target, config.maximum_correspondence_distance_m);
  return result;
}

RegistrationResult align_ndt(
  const Cloud::ConstPtr & source,
  const Cloud::ConstPtr & target,
  const Eigen::Matrix4f & initial_target_from_source,
  const RegistrationConfig & config)
{
  const auto clean_source = finite_cloud(source, "source");
  const auto clean_target = finite_cloud(target, "target");
  validate_inputs(clean_source, clean_target, config);
  validate_spatial_rank(clean_source, clean_target, 3, "NDT");
  if (!initial_target_from_source.allFinite()) {
    throw std::invalid_argument("NDT initial transform must be finite");
  }
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> registration;
  registration.setInputSource(clean_source);
  registration.setInputTarget(clean_target);
  registration.setMaximumIterations(config.maximum_iterations);
  registration.setTransformationEpsilon(config.transformation_epsilon);
  registration.setResolution(config.ndt_resolution_m);
  registration.setStepSize(config.ndt_step_size);

  Cloud aligned;
  registration.align(aligned, initial_target_from_source);
  auto result = base_result(clean_source, clean_target, "ndt");
  result.converged = registration.hasConverged();
  result.fitness = registration.getFitnessScore(config.maximum_correspondence_distance_m);
  result.target_from_source = registration.getFinalTransformation();
  characterize_alignment(
    result, clean_source, clean_target, config.maximum_correspondence_distance_m);
  return result;
}

RegistrationResult align_gicp(
  const Cloud::ConstPtr & source,
  const Cloud::ConstPtr & target,
  const Eigen::Matrix4f & initial_target_from_source,
  const RegistrationConfig & config)
{
  const auto clean_source = finite_cloud(source, "source");
  const auto clean_target = finite_cloud(target, "target");
  validate_inputs(clean_source, clean_target, config);
  validate_spatial_rank(clean_source, clean_target, 2, "GICP");
  if (!initial_target_from_source.allFinite()) {
    throw std::invalid_argument("GICP initial transform must be finite");
  }
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;
  registration.setInputSource(clean_source);
  registration.setInputTarget(clean_target);
  registration.setMaxCorrespondenceDistance(config.maximum_correspondence_distance_m);
  registration.setMaximumIterations(config.maximum_iterations);
  registration.setTransformationEpsilon(config.transformation_epsilon);
  registration.setEuclideanFitnessEpsilon(config.euclidean_fitness_epsilon);

  Cloud aligned;
  registration.align(aligned, initial_target_from_source);
  auto result = base_result(clean_source, clean_target, "gicp");
  result.converged = registration.hasConverged();
  result.fitness = registration.getFitnessScore(config.maximum_correspondence_distance_m);
  result.target_from_source = registration.getFinalTransformation();
  characterize_alignment(
    result, clean_source, clean_target, config.maximum_correspondence_distance_m);
  return result;
}

RegistrationResult align_point_to_plane(
  const Cloud::ConstPtr & source,
  const Cloud::ConstPtr & target,
  const Eigen::Matrix4f & initial_target_from_source,
  const RegistrationConfig & config)
{
  const auto clean_source = finite_cloud(source, "source");
  const auto clean_target = finite_cloud(target, "target");
  validate_inputs(clean_source, clean_target, config);
  validate_spatial_rank(clean_source, clean_target, 2, "point-to-plane ICP");
  if (!initial_target_from_source.allFinite()) {
    throw std::invalid_argument("point-to-plane ICP initial transform must be finite");
  }
  const auto source_with_normals = estimate_normals(
    clean_source, config.normal_k_neighbors);
  const auto target_with_normals = estimate_normals(
    clean_target, config.normal_k_neighbors);
  pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> registration;
  registration.setUseSymmetricObjective(false);
  registration.setInputSource(source_with_normals);
  registration.setInputTarget(target_with_normals);
  registration.setMaxCorrespondenceDistance(config.maximum_correspondence_distance_m);
  registration.setMaximumIterations(config.maximum_iterations);
  registration.setTransformationEpsilon(config.transformation_epsilon);
  registration.setEuclideanFitnessEpsilon(config.euclidean_fitness_epsilon);

  pcl::PointCloud<pcl::PointNormal> aligned;
  registration.align(aligned, initial_target_from_source);
  auto result = base_result(clean_source, clean_target, "point_to_plane");
  result.converged = registration.hasConverged();
  result.fitness = registration.getFitnessScore(config.maximum_correspondence_distance_m);
  result.target_from_source = registration.getFinalTransformation();
  characterize_point_to_plane_alignment(
    result, clean_source, target_with_normals,
    config.maximum_correspondence_distance_m);
  return result;
}

}  // namespace uf_relocalization
