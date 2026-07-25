#include "uf_relocalization/descriptor_core.hpp"

#include <pcl/common/point_tests.h>
#include <pcl/features/esf.h>

#include <cmath>
#include <memory>
#include <stdexcept>

namespace uf_relocalization
{

std::vector<float> compute_esf_descriptor(const Cloud::ConstPtr & cloud)
{
  if (!cloud || cloud->empty()) {
    throw std::invalid_argument("ESF input cloud must be non-empty");
  }
  auto finite_cloud = std::make_shared<Cloud>();
  finite_cloud->reserve(cloud->size());
  for (const auto & point : *cloud) {
    if (pcl::isFinite(point)) {
      finite_cloud->push_back(point);
    }
  }
  if (finite_cloud->size() < 10) {
    throw std::invalid_argument("ESF input requires at least 10 finite points");
  }

  pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> estimator;
  estimator.setInputCloud(finite_cloud);
  pcl::PointCloud<pcl::ESFSignature640> output;
  estimator.compute(output);
  if (output.size() != 1) {
    throw std::runtime_error("PCL ESF did not produce one descriptor");
  }

  std::vector<float> descriptor(640);
  for (std::size_t index = 0; index < descriptor.size(); ++index) {
    const float value = output.front().histogram[index];
    if (!std::isfinite(value)) {
      throw std::runtime_error("PCL ESF produced a non-finite descriptor");
    }
    descriptor[index] = value;
  }
  return descriptor;
}

}  // namespace uf_relocalization
