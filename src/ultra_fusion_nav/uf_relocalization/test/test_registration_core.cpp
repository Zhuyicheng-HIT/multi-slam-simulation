#include "uf_relocalization/registration_core.hpp"

#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

#include <cmath>
#include <memory>

namespace
{

uf_relocalization::Cloud::Ptr make_asymmetric_cloud()
{
  auto cloud = std::make_shared<uf_relocalization::Cloud>();
  cloud->reserve(1200);
  for (int i = 0; i < 1200; ++i) {
    const float x = 0.08F * static_cast<float>(i % 31);
    const float y = 0.09F * static_cast<float>((i * 7) % 29);
    const float z = 0.35F * std::sin(0.17F * static_cast<float>(i)) +
      0.04F * static_cast<float>((i * 11) % 13);
    cloud->push_back(pcl::PointXYZ{x, y, z});
  }
  return cloud;
}

Eigen::Matrix4f known_transform()
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(0.09F, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  transform.block<3, 1>(0, 3) = Eigen::Vector3f{0.32F, -0.18F, 0.07F};
  return transform;
}

double translation_error(const Eigen::Matrix4f & estimate, const Eigen::Matrix4f & truth)
{
  return (estimate.block<3, 1>(0, 3) - truth.block<3, 1>(0, 3)).norm();
}

double rotation_error(const Eigen::Matrix4f & estimate, const Eigen::Matrix4f & truth)
{
  const Eigen::Matrix3f delta =
    estimate.block<3, 3>(0, 0) * truth.block<3, 3>(0, 0).transpose();
  return Eigen::AngleAxisf(delta).angle();
}

}  // namespace

TEST(RegistrationCore, IcpRecoversKnownRigidTransform)
{
  const auto target = make_asymmetric_cloud();
  auto source = std::make_shared<uf_relocalization::Cloud>();
  const Eigen::Matrix4f truth = known_transform();
  pcl::transformPointCloud(*target, *source, truth.inverse());

  uf_relocalization::RegistrationConfig config;
  config.maximum_correspondence_distance_m = 0.8;
  const auto result = uf_relocalization::align_icp(
    source, target, Eigen::Matrix4f::Identity(), config);

  ASSERT_TRUE(result.converged);
  EXPECT_LT(result.fitness, 1.0e-4);
  EXPECT_LT(translation_error(result.target_from_source, truth), 0.01);
  EXPECT_LT(rotation_error(result.target_from_source, truth), 0.01);
}

TEST(RegistrationCore, NdtRefinesAReasonableCandidate)
{
  const auto target = make_asymmetric_cloud();
  auto source = std::make_shared<uf_relocalization::Cloud>();
  const Eigen::Matrix4f truth = known_transform();
  pcl::transformPointCloud(*target, *source, truth.inverse());
  Eigen::Matrix4f initial = truth;
  initial(0, 3) += 0.08F;
  initial(1, 3) -= 0.05F;

  uf_relocalization::RegistrationConfig config;
  config.maximum_correspondence_distance_m = 1.0;
  const auto result = uf_relocalization::align_ndt(source, target, initial, config);

  ASSERT_TRUE(result.converged);
  EXPECT_LT(translation_error(result.target_from_source, truth), 0.12);
  EXPECT_LT(rotation_error(result.target_from_source, truth), 0.08);
}

TEST(RegistrationCore, EmptyCloudIsRejected)
{
  auto empty = std::make_shared<uf_relocalization::Cloud>();
  const auto target = make_asymmetric_cloud();
  EXPECT_THROW(
    uf_relocalization::align_icp(
      empty, target, Eigen::Matrix4f::Identity()),
    std::invalid_argument);
}
