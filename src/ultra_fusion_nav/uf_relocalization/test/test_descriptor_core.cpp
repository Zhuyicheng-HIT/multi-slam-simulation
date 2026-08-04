#include "uf_relocalization/descriptor_core.hpp"
#include "uf_relocalization/keyframe_database.hpp"

#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <memory>

namespace
{

uf_relocalization::Cloud::Ptr asymmetric_cloud()
{
  auto cloud = std::make_shared<uf_relocalization::Cloud>();
  cloud->reserve(1600);
  for (int index = 0; index < 1600; ++index) {
    const float x = 0.06F * static_cast<float>(index % 37);
    const float y = 0.08F * static_cast<float>((index * 7) % 31);
    const float z = 0.45F * std::sin(0.13F * static_cast<float>(index)) +
      0.03F * static_cast<float>((index * 11) % 17);
    cloud->push_back(pcl::PointXYZ{x, y, z});
  }
  return cloud;
}

uf_relocalization::Cloud::Ptr planar_cloud()
{
  auto cloud = std::make_shared<uf_relocalization::Cloud>();
  for (int x = 0; x < 40; ++x) {
    for (int y = 0; y < 40; ++y) {
      cloud->push_back(pcl::PointXYZ{
        0.05F * static_cast<float>(x),
        0.05F * static_cast<float>(y),
        0.0F});
    }
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

Eigen::Isometry3d pose(const double x)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation().x() = x;
  return result;
}

}  // namespace

TEST(DescriptorCore, EsfRetrievesRigidlyTransformedShape)
{
  const auto target = asymmetric_cloud();
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(0.7F, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  transform.block<3, 1>(0, 3) = Eigen::Vector3f{2.0F, -1.0F, 0.5F};
  auto transformed = std::make_shared<uf_relocalization::Cloud>();
  pcl::transformPointCloud(*target, *transformed, transform);

  const auto target_descriptor = uf_relocalization::compute_esf_descriptor(target);
  const auto query_descriptor = uf_relocalization::compute_esf_descriptor(transformed);
  const auto distractor_descriptor =
    uf_relocalization::compute_esf_descriptor(planar_cloud());
  EXPECT_EQ(target_descriptor.size(), 640U);
  EXPECT_TRUE(std::all_of(
    query_descriptor.begin(), query_descriptor.end(),
    [](const float value) {return std::isfinite(value);}));

  uf_relocalization::KeyframeDatabaseConfig config;
  config.minimum_translation_spacing_m = 0.5;
  uf_relocalization::StaticKeyframeDatabase database(config);
  const auto target_admission = database.try_insert(
    1.0, pose(0.0), target, target_descriptor, healthy_quality());
  const auto distractor_admission = database.try_insert(
    2.0, pose(1.0), planar_cloud(), distractor_descriptor, healthy_quality());
  ASSERT_TRUE(target_admission.accepted && distractor_admission.accepted);

  const auto candidates = database.query(query_descriptor, 2);
  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(candidates.front().keyframe_id, target_admission.keyframe_id);
  EXPECT_LT(candidates[0].descriptor_distance, candidates[1].descriptor_distance);
}

TEST(DescriptorCore, EsfRejectsInsufficientCloud)
{
  auto empty = std::make_shared<uf_relocalization::Cloud>();
  EXPECT_THROW(
    uf_relocalization::compute_esf_descriptor(empty),
    std::invalid_argument);
}
