#include "uf_relocalization/registration_core.hpp"

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <limits>
#include <stdexcept>

namespace uf_relocalization
{
namespace
{

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
  if (config.ndt_resolution_m <= 0.0 || config.ndt_step_size <= 0.0) {
    throw std::invalid_argument("NDT resolution and step size must be positive");
  }
}

RegistrationResult base_result(
  const Cloud::ConstPtr & source,
  const Cloud::ConstPtr & target,
  const char * method)
{
  RegistrationResult result;
  result.fitness = std::numeric_limits<double>::infinity();
  result.source_points = source->size();
  result.target_points = target->size();
  result.method = method;
  return result;
}

}  // namespace

RegistrationResult align_icp(
  const Cloud::ConstPtr & source,
  const Cloud::ConstPtr & target,
  const Eigen::Matrix4f & initial_target_from_source,
  const RegistrationConfig & config)
{
  validate_inputs(source, target, config);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;
  registration.setInputSource(source);
  registration.setInputTarget(target);
  registration.setMaxCorrespondenceDistance(config.maximum_correspondence_distance_m);
  registration.setMaximumIterations(config.maximum_iterations);
  registration.setTransformationEpsilon(config.transformation_epsilon);
  registration.setEuclideanFitnessEpsilon(config.euclidean_fitness_epsilon);

  Cloud aligned;
  registration.align(aligned, initial_target_from_source);
  auto result = base_result(source, target, "icp");
  result.converged = registration.hasConverged();
  result.fitness = registration.getFitnessScore(config.maximum_correspondence_distance_m);
  result.target_from_source = registration.getFinalTransformation();
  return result;
}

RegistrationResult align_ndt(
  const Cloud::ConstPtr & source,
  const Cloud::ConstPtr & target,
  const Eigen::Matrix4f & initial_target_from_source,
  const RegistrationConfig & config)
{
  validate_inputs(source, target, config);
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> registration;
  registration.setInputSource(source);
  registration.setInputTarget(target);
  registration.setMaximumIterations(config.maximum_iterations);
  registration.setTransformationEpsilon(config.transformation_epsilon);
  registration.setResolution(config.ndt_resolution_m);
  registration.setStepSize(config.ndt_step_size);

  Cloud aligned;
  registration.align(aligned, initial_target_from_source);
  auto result = base_result(source, target, "ndt");
  result.converged = registration.hasConverged();
  result.fitness = registration.getFitnessScore(config.maximum_correspondence_distance_m);
  result.target_from_source = registration.getFinalTransformation();
  return result;
}

}  // namespace uf_relocalization
