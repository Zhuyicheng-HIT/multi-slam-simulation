#include "uf_relocalization/keyframe_database.hpp"
#include "uf_relocalization/registration_core.hpp"

#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

#include <cmath>
#include <memory>

namespace
{

uf_relocalization::Cloud::Ptr make_cloud()
{
  auto cloud = std::make_shared<uf_relocalization::Cloud>();
  cloud->reserve(1200);
  for (int index = 0; index < 1200; ++index) {
    const float x = 0.08F * static_cast<float>(index % 31);
    const float y = 0.09F * static_cast<float>((index * 7) % 29);
    const float z = 0.35F * std::sin(0.17F * static_cast<float>(index)) +
      0.04F * static_cast<float>((index * 11) % 13);
    cloud->push_back(pcl::PointXYZ{x, y, z});
  }
  return cloud;
}

uf_relocalization::KeyframeQuality healthy_quality()
{
  uf_relocalization::KeyframeQuality quality;
  quality.map_quality = 0.8;
  quality.feature_repeatability = 0.9;
  quality.dynamic_ratio = 0.02;
  quality.lidar_degradation = 0.3;
  quality.scheduler_lidar_enabled = true;
  return quality;
}

}  // namespace

TEST(RelocalizationPipeline, RetrievedStaticKeyframeFeedsIcpVerification)
{
  auto target = make_cloud();
  uf_relocalization::StaticKeyframeDatabase database;
  const auto admission = database.try_insert(
    1.0,
    Eigen::Isometry3d::Identity(),
    target,
    {0.9F, 0.2F, 0.1F},
    healthy_quality());
  ASSERT_TRUE(admission.accepted);

  const auto candidates = database.query({0.91F, 0.19F, 0.1F}, 1);
  ASSERT_EQ(candidates.size(), 1U);
  const auto * keyframe = database.find(candidates.front().keyframe_id);
  ASSERT_NE(keyframe, nullptr);

  Eigen::Matrix4f truth = Eigen::Matrix4f::Identity();
  truth.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(0.08F, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  truth.block<3, 1>(0, 3) = Eigen::Vector3f{0.28F, -0.16F, 0.06F};
  auto source = std::make_shared<uf_relocalization::Cloud>();
  pcl::transformPointCloud(*target, *source, truth.inverse());

  uf_relocalization::RegistrationConfig config;
  config.maximum_correspondence_distance_m = 0.8;
  const auto result = uf_relocalization::align_icp(
    source, keyframe->cloud, Eigen::Matrix4f::Identity(), config);

  ASSERT_TRUE(result.converged);
  EXPECT_LT(result.fitness, 1.0e-4);
  EXPECT_LT(
    (result.target_from_source.block<3, 1>(0, 3) -
    truth.block<3, 1>(0, 3)).norm(),
    0.01);
  const Eigen::Matrix3f rotation_delta =
    result.target_from_source.block<3, 3>(0, 0) *
    truth.block<3, 3>(0, 0).transpose();
  EXPECT_LT(Eigen::AngleAxisf(rotation_delta).angle(), 0.01);
}
