#include "hybridfusion_map_fusion/hybridfusion_core.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <vector>

namespace hf = hybridfusion_map_fusion;

TEST(HybridFusionCore, PearsonCorrelationPreservesPaperCriterion)
{
  std::array<float, 640> first{};
  std::array<float, 640> same{};
  std::array<float, 640> inverse{};
  for (std::size_t index = 0; index < first.size(); ++index) {
    first[index] = static_cast<float>((index * 17) % 97);
    same[index] = 2.0F * first[index] + 3.0F;
    inverse[index] = -first[index];
  }
  EXPECT_NEAR(hf::pearson_correlation(first, same), 1.0, 1e-6);
  EXPECT_NEAR(hf::pearson_correlation(first, inverse), -1.0, 1e-6);
}

TEST(HybridFusionCore, TransformAverageClustersTranslationAndYaw)
{
  std::vector<Eigen::Isometry3d> transforms;
  transforms.push_back(hf::xyz_rpy_transform({1.0, 2.0, 0.2, 0.0, 0.0, 0.10}));
  transforms.push_back(hf::xyz_rpy_transform({1.2, 1.8, 0.3, 0.0, 0.0, 0.12}));
  transforms.push_back(hf::xyz_rpy_transform({0.8, 2.2, 0.1, 0.0, 0.0, 0.08}));
  const auto average = hf::average_transforms(transforms);
  const auto values = hf::transform_xyz_rpy(average);
  EXPECT_NEAR(values[0], 1.0, 1e-9);
  EXPECT_NEAR(values[1], 2.0, 1e-9);
  EXPECT_NEAR(values[2], 0.2, 1e-9);
  EXPECT_NEAR(values[5], 0.10, 2e-3);
}

TEST(HybridFusionCore, BoundaryProjectionRejectsGroundInterior)
{
  hf::CloudPtr cloud(new hf::CloudT);
  for (int x = -10; x <= 10; ++x) {
    for (int y = -10; y <= 10; ++y) {
      hf::PointT ground;
      ground.x = x * 0.1F;
      ground.y = y * 0.1F;
      ground.z = 0.0F;
      cloud->push_back(ground);
    }
  }
  for (int index = -20; index <= 20; ++index) {
    hf::PointT wall;
    wall.x = index * 0.05F;
    wall.y = -0.8F;
    wall.z = 1.2F;
    cloud->push_back(wall);
  }
  hf::BoundaryConfig config;
  config.ground_clearance_m = 0.15;
  config.raster_resolution_m = 0.10;
  const auto boundary = hf::extract_xy_boundary(cloud, config);
  EXPECT_GT(boundary->size(), 10U);
  EXPECT_LT(boundary->size(), cloud->size());
  for (const auto & point : boundary->points) {
    EXPECT_FLOAT_EQ(point.z, 0.0F);
  }
}

TEST(HybridFusionCore, TruthTransformProducesZeroPoseError)
{
  hf::CloudPtr visual(new hf::CloudT);
  hf::CloudPtr lidar(new hf::CloudT);
  for (int x = 0; x < 20; ++x) {
    for (int y = 0; y < 20; ++y) {
      hf::PointT point;
      point.x = x * 0.1F;
      point.y = y * 0.1F;
      point.z = (x % 3) * 0.1F;
      visual->push_back(point);
      lidar->push_back(point);
    }
  }
  hf::Config config;
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  const auto metrics = hf::evaluate_alignment(visual, lidar, identity, identity, config);
  EXPECT_NEAR(metrics.translation_error_m, 0.0, 1e-12);
  EXPECT_NEAR(metrics.rotation_error_deg, 0.0, 1e-12);
  EXPECT_GT(metrics.inlier_ratio, 0.99);
}

TEST(HybridFusionCore, DatasetAllowsLiveMapWithoutGroundTruth)
{
  const auto manifest = std::filesystem::path(__FILE__).parent_path() /
    "data" / "dataset_without_truth.yaml";
  const auto dataset = hf::load_dataset(manifest.string());
  EXPECT_EQ(dataset.dataset_id, "live_without_truth");
  EXPECT_FALSE(dataset.has_truth);
  EXPECT_EQ(dataset.visual_frame, "odom");
  EXPECT_EQ(dataset.lidar_frame, "camera_init");
}

TEST(HybridFusionCore, LiveMapMetricsDoNotInventGroundTruthError)
{
  hf::CloudPtr visual(new hf::CloudT);
  hf::CloudPtr lidar(new hf::CloudT);
  for (int x = 0; x < 20; ++x) {
    for (int y = 0; y < 20; ++y) {
      hf::PointT point;
      point.x = x * 0.1F;
      point.y = y * 0.1F;
      point.z = (x % 3) * 0.1F;
      visual->push_back(point);
      lidar->push_back(point);
    }
  }
  hf::Dataset dataset;
  dataset.has_truth = false;
  hf::Config config;
  const auto result = hf::run_registration("initial", visual, lidar, dataset, config);
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(std::isnan(result.metrics.translation_error_m));
  EXPECT_TRUE(std::isnan(result.metrics.rotation_error_deg));
  EXPECT_GT(result.metrics.inlier_ratio, 0.99);
}
