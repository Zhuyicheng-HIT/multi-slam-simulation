#include "uf_relocalization/registration_core.hpp"

#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

#include <cmath>
#include <limits>
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

uf_relocalization::Cloud::Ptr make_planar_cloud()
{
  auto cloud = std::make_shared<uf_relocalization::Cloud>();
  cloud->reserve(400);
  for (int row = 0; row < 20; ++row) {
    for (int column = 0; column < 20; ++column) {
      cloud->push_back(pcl::PointXYZ{
        0.10F * static_cast<float>(column),
        0.10F * static_cast<float>(row),
        0.0F});
    }
  }
  return cloud;
}

uf_relocalization::Cloud::Ptr make_collinear_cloud()
{
  auto cloud = std::make_shared<uf_relocalization::Cloud>();
  cloud->reserve(200);
  for (int index = 0; index < 200; ++index) {
    cloud->push_back(pcl::PointXYZ{0.05F * static_cast<float>(index), 0.0F, 0.0F});
  }
  return cloud;
}

uf_relocalization::Cloud::Ptr make_three_plane_cloud(const bool shifted_sampling)
{
  auto cloud = std::make_shared<uf_relocalization::Cloud>();
  cloud->reserve(1200);
  const float first_shift = shifted_sampling ? 0.045F : 0.0F;
  const float second_shift = shifted_sampling ? 0.065F : 0.0F;
  for (int row = 0; row < 20; ++row) {
    for (int column = 0; column < 20; ++column) {
      const float first = 0.25F + 0.14F * static_cast<float>(column);
      const float second = 0.25F + 0.13F * static_cast<float>(row);
      cloud->push_back(pcl::PointXYZ{
        first + first_shift, second + second_shift, 0.0F});
      cloud->push_back(pcl::PointXYZ{
        0.0F, first + second_shift, second + first_shift});
      cloud->push_back(pcl::PointXYZ{
        first + second_shift, 0.0F, second + first_shift});
    }
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
  EXPECT_EQ(result.method, "icp");
  EXPECT_LT(result.fitness, 1.0e-4);
  EXPECT_GT(result.overlap_ratio, 0.95);
  EXPECT_GT(result.correspondence_points, 1100U);
  EXPECT_LT(result.inlier_rmse, 0.01);
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
  EXPECT_EQ(result.method, "ndt");
  EXPECT_GT(result.overlap_ratio, 0.90);
  EXPECT_LT(translation_error(result.target_from_source, truth), 0.12);
  EXPECT_LT(rotation_error(result.target_from_source, truth), 0.08);
}

TEST(RegistrationCore, GicpRecoversRawScanRelativeMotion)
{
  const auto target = make_asymmetric_cloud();
  auto source = std::make_shared<uf_relocalization::Cloud>();
  const Eigen::Matrix4f truth = known_transform();
  pcl::transformPointCloud(*target, *source, truth.inverse());

  uf_relocalization::RegistrationConfig config;
  config.maximum_correspondence_distance_m = 0.8;
  config.maximum_iterations = 40;
  const auto result = uf_relocalization::align_gicp(
    source, target, Eigen::Matrix4f::Identity(), config);

  ASSERT_TRUE(result.converged);
  EXPECT_EQ(result.method, "gicp");
  EXPECT_LT(result.fitness, 1.0e-4);
  EXPECT_GT(result.overlap_ratio, 0.95);
  EXPECT_LT(translation_error(result.target_from_source, truth), 0.01);
  EXPECT_LT(rotation_error(result.target_from_source, truth), 0.01);
}

TEST(RegistrationCore, PointToPlaneHandlesTangentialResampling)
{
  const auto target = make_three_plane_cloud(false);
  const auto shifted_surface = make_three_plane_cloud(true);
  auto source = std::make_shared<uf_relocalization::Cloud>();
  const Eigen::Matrix4f truth = known_transform();
  pcl::transformPointCloud(*shifted_surface, *source, truth.inverse());

  uf_relocalization::RegistrationConfig config;
  config.maximum_correspondence_distance_m = 0.8;
  config.maximum_iterations = 60;
  config.normal_k_neighbors = 16;
  const auto result = uf_relocalization::align_point_to_plane(
    source, target, truth, config);

  ASSERT_TRUE(result.converged);
  EXPECT_EQ(result.method, "point_to_plane");
  EXPECT_GT(result.overlap_ratio, 0.95);
  EXPECT_LT(result.inlier_rmse, 0.02);
  EXPECT_GT(result.reciprocal_correspondence_points, 500U);
  EXPECT_GT(result.reciprocal_ratio, 0.40);
  EXPECT_LT(result.absolute_plane_error_p90_m, 0.03);
  EXPECT_GT(result.euclidean_inlier_rmse, result.inlier_rmse);
  EXPECT_EQ(result.effective_rank, 6);
  EXPECT_LT(translation_error(result.target_from_source, truth), 0.03);
  EXPECT_LT(rotation_error(result.target_from_source, truth), 0.03);
}

TEST(RegistrationCore, ReciprocalSupportIgnoresUnseenMapExtent)
{
  const auto visible_target = make_three_plane_cloud(false);
  auto expanded_target = std::make_shared<uf_relocalization::Cloud>(*visible_target);
  const auto unseen = make_three_plane_cloud(false);
  Eigen::Matrix4f far_transform = Eigen::Matrix4f::Identity();
  far_transform.block<3, 1>(0, 3) = Eigen::Vector3f{40.0F, 35.0F, 12.0F};
  uf_relocalization::Cloud far_cloud;
  pcl::transformPointCloud(*unseen, far_cloud, far_transform);
  *expanded_target += far_cloud;

  const auto shifted_surface = make_three_plane_cloud(true);
  auto source = std::make_shared<uf_relocalization::Cloud>();
  const Eigen::Matrix4f truth = known_transform();
  pcl::transformPointCloud(*shifted_surface, *source, truth.inverse());

  uf_relocalization::RegistrationConfig config;
  config.maximum_correspondence_distance_m = 0.8;
  config.maximum_iterations = 60;
  config.normal_k_neighbors = 16;
  const auto visible = uf_relocalization::align_point_to_plane(
    source, visible_target, truth, config);
  const auto expanded = uf_relocalization::align_point_to_plane(
    source, expanded_target, truth, config);

  ASSERT_TRUE(visible.converged);
  ASSERT_TRUE(expanded.converged);
  EXPECT_NEAR(expanded.reciprocal_ratio, visible.reciprocal_ratio, 0.03);
  EXPECT_NEAR(expanded.overlap_ratio, visible.overlap_ratio, 0.03);
  EXPECT_LT(translation_error(expanded.target_from_source, truth), 0.03);
}

TEST(RegistrationCore, ReciprocalSupportDetectsManyToOneSampling)
{
  const auto target = make_three_plane_cloud(false);
  auto unique_source = std::make_shared<uf_relocalization::Cloud>();
  const Eigen::Matrix4f truth = known_transform();
  pcl::transformPointCloud(*target, *unique_source, truth.inverse());
  auto duplicated_source = std::make_shared<uf_relocalization::Cloud>();
  duplicated_source->reserve(unique_source->size() * 4U);
  for (const auto & point : *unique_source) {
    for (int duplicate = 0; duplicate < 4; ++duplicate) {
      duplicated_source->push_back(point);
    }
  }

  uf_relocalization::RegistrationConfig config;
  config.maximum_correspondence_distance_m = 0.8;
  config.maximum_iterations = 30;
  config.normal_k_neighbors = 20;
  const auto result = uf_relocalization::align_point_to_plane(
    duplicated_source, target, truth, config);

  ASSERT_TRUE(result.converged);
  EXPECT_GT(result.overlap_ratio, 0.95);
  EXPECT_LT(result.inlier_rmse, 0.02);
  EXPECT_LT(result.reciprocal_ratio, 0.35);
  EXPECT_LT(
    result.reciprocal_correspondence_points,
    result.correspondence_points / 2U);
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

TEST(RegistrationCore, IcpFiltersNonFinitePoints)
{
  const auto target = make_asymmetric_cloud();
  auto source = std::make_shared<uf_relocalization::Cloud>();
  const Eigen::Matrix4f truth = known_transform();
  pcl::transformPointCloud(*target, *source, truth.inverse());
  const float nan = std::numeric_limits<float>::quiet_NaN();
  target->push_back(pcl::PointXYZ{nan, 0.0F, 0.0F});
  source->push_back(pcl::PointXYZ{0.0F, nan, 0.0F});

  uf_relocalization::RegistrationConfig config;
  config.maximum_correspondence_distance_m = 0.8;
  const auto result = uf_relocalization::align_icp(
    source, target, Eigen::Matrix4f::Identity(), config);

  ASSERT_TRUE(result.converged);
  EXPECT_EQ(result.source_points, 1200U);
  EXPECT_EQ(result.target_points, 1200U);
  EXPECT_LT(translation_error(result.target_from_source, truth), 0.01);
}

TEST(RegistrationCore, NdtRejectsCollinearCloudBeforeNativeRegistration)
{
  const auto collinear = make_collinear_cloud();
  EXPECT_THROW(
    uf_relocalization::align_ndt(
      collinear, collinear, Eigen::Matrix4f::Identity()),
    std::invalid_argument);
}

TEST(RegistrationCore, IcpHandlesPlanarCloudWithoutNativeCrash)
{
  const auto target = make_planar_cloud();
  auto source = std::make_shared<uf_relocalization::Cloud>();
  const Eigen::Matrix4f truth = known_transform();
  pcl::transformPointCloud(*target, *source, truth.inverse());

  uf_relocalization::RegistrationConfig config;
  config.maximum_correspondence_distance_m = 0.8;
  const auto result = uf_relocalization::align_icp(source, target, truth, config);
  EXPECT_TRUE(result.target_from_source.allFinite());
  EXPECT_GT(result.correspondence_points, 300U);
}

TEST(RegistrationCore, IcpPoorInitialOverlapReturnsSafely)
{
  const auto target = make_asymmetric_cloud();
  const auto source = make_asymmetric_cloud();
  Eigen::Matrix4f initial = Eigen::Matrix4f::Identity();
  initial(0, 3) = 100.0F;

  uf_relocalization::RegistrationConfig config;
  config.maximum_correspondence_distance_m = 0.5;
  for (int attempt = 0; attempt < 10; ++attempt) {
    const auto result = uf_relocalization::align_icp(source, target, initial, config);
    EXPECT_TRUE(result.target_from_source.allFinite());
    EXPECT_EQ(result.correspondence_points, 0U);
    EXPECT_EQ(result.overlap_ratio, 0.0);
  }
}
