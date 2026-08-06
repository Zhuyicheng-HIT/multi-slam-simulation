#include "uf_relocalization/keyframe_database.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <vector>

namespace
{

uf_relocalization::Cloud::Ptr cloud()
{
  auto result = std::make_shared<uf_relocalization::Cloud>();
  result->push_back(pcl::PointXYZ{0.0F, 0.0F, 0.0F});
  result->push_back(pcl::PointXYZ{1.0F, 0.0F, 0.0F});
  return result;
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

Eigen::Isometry3d pose(const double x, const double yaw = 0.0)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation().x() = x;
  result.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

}  // namespace

TEST(KeyframeDatabase, RejectsUnhealthyObservations)
{
  uf_relocalization::StaticKeyframeDatabase disabled_database;
  auto quality = healthy_quality();
  quality.scheduler_lidar_enabled = false;
  EXPECT_EQ(
    disabled_database.try_insert(
      1.0, pose(0.0), cloud(), {1.0F, 0.0F}, quality).reason,
    "scheduler_lidar_disabled");

  uf_relocalization::StaticKeyframeDatabase diagnostic_database;
  quality = healthy_quality();
  quality.map_quality = 0.2;
  EXPECT_TRUE(diagnostic_database.try_insert(
    1.0, pose(0.0), cloud(), {1.0F, 0.0F}, quality).accepted);

  uf_relocalization::StaticKeyframeDatabase dynamic_database;
  quality = healthy_quality();
  quality.dynamic_ratio = 0.4;
  EXPECT_EQ(
    dynamic_database.try_insert(
      1.0, pose(0.0), cloud(), {1.0F, 0.0F}, quality).reason,
    "high_dynamic_ratio");
  EXPECT_TRUE(dynamic_database.keyframes().empty());
}

TEST(KeyframeDatabase, EnforcesPoseSpacingAndBoundedStorage)
{
  uf_relocalization::KeyframeDatabaseConfig config;
  config.maximum_keyframes = 2;
  uf_relocalization::StaticKeyframeDatabase database(config);
  auto source_cloud = cloud();

  const auto first = database.try_insert(
    1.0, pose(0.0), source_cloud, {1.0F, 0.0F}, healthy_quality());
  ASSERT_TRUE(first.accepted);
  source_cloud->front().x = 99.0F;
  ASSERT_NE(database.find(first.keyframe_id), nullptr);
  EXPECT_FLOAT_EQ(database.find(first.keyframe_id)->cloud->front().x, 0.0F);

  const auto too_close = database.try_insert(
    2.0, pose(0.2), cloud(), {0.9F, 0.1F}, healthy_quality());
  EXPECT_FALSE(too_close.accepted);
  EXPECT_EQ(too_close.reason, "insufficient_pose_spacing");

  const auto rotated = database.try_insert(
    3.0, pose(0.2, 0.4), cloud(), {0.8F, 0.2F}, healthy_quality());
  ASSERT_TRUE(rotated.accepted);
  const auto translated = database.try_insert(
    4.0, pose(2.0, 0.4), cloud(), {0.0F, 1.0F}, healthy_quality());
  ASSERT_TRUE(translated.accepted);
  EXPECT_EQ(database.keyframes().size(), 2U);
  EXPECT_EQ(database.find(first.keyframe_id), nullptr);
}

TEST(KeyframeDatabase, RanksDescriptorsAndExcludesRecentFrames)
{
  uf_relocalization::KeyframeDatabaseConfig config;
  config.minimum_translation_spacing_m = 0.5;
  uf_relocalization::StaticKeyframeDatabase database(config);
  const auto first = database.try_insert(
    1.0, pose(0.0), cloud(), {1.0F, 0.0F, 0.0F}, healthy_quality());
  const auto second = database.try_insert(
    2.0, pose(1.0), cloud(), {0.8F, 0.2F, 0.0F}, healthy_quality());
  const auto recent = database.try_insert(
    3.0, pose(2.0), cloud(), {0.0F, 1.0F, 0.0F}, healthy_quality());
  ASSERT_TRUE(first.accepted && second.accepted && recent.accepted);

  const auto matches = database.query({1.0F, 0.0F, 0.0F}, 2, 1);
  ASSERT_EQ(matches.size(), 2U);
  EXPECT_EQ(matches[0].keyframe_id, first.keyframe_id);
  EXPECT_EQ(matches[1].keyframe_id, second.keyframe_id);
  EXPECT_LT(matches[0].descriptor_distance, matches[1].descriptor_distance);

  const auto mismatch = database.try_insert(
    4.0, pose(3.0), cloud(), {1.0F, 0.0F}, healthy_quality());
  EXPECT_FALSE(mismatch.accepted);
  EXPECT_EQ(mismatch.reason, "descriptor_dimension_mismatch");
  EXPECT_THROW(database.query({0.0F, 0.0F, 0.0F}, 1), std::invalid_argument);
}
