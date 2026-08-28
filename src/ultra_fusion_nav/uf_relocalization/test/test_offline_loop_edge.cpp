#include "uf_relocalization/offline_loop_edge.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <limits>
#include <stdexcept>

TEST(OfflineLoopEdge, ConvertsRegisteredMapPoseToCandidateRelativeMeasurement)
{
  Eigen::Isometry3d map_from_candidate = Eigen::Isometry3d::Identity();
  map_from_candidate.translation() = Eigen::Vector3d{1.0, 2.0, 0.5};
  uf_relocalization::RegistrationResult registration;
  registration.converged = true;
  registration.target_from_source = Eigen::Matrix4f::Identity();
  registration.target_from_source.block<3, 1>(0, 3) = Eigen::Vector3f{2.0F, 2.5F, 0.5F};
  registration.correspondence_points = 500U;
  registration.overlap_ratio = 0.8;
  registration.reciprocal_ratio = 0.6;
  registration.inlier_rmse = 0.04;
  registration.effective_rank = 6;
  registration.condition_number = 25.0;

  const auto edge = uf_relocalization::make_offline_loop_edge(
    3U, 12U, 10.0, 42.0, map_from_candidate, 0.02, registration);

  EXPECT_EQ(edge.candidate_keyframe, 3U);
  EXPECT_EQ(edge.query_keyframe, 12U);
  EXPECT_DOUBLE_EQ(edge.temporal_separation_s, 32.0);
  EXPECT_TRUE(edge.candidate_from_query.matrix().allFinite());
  EXPECT_NEAR(edge.candidate_from_query.translation().x(), 1.0, 1.0e-12);
  EXPECT_NEAR(edge.candidate_from_query.translation().y(), 0.5, 1.0e-12);
  EXPECT_NEAR(edge.candidate_from_query.translation().z(), 0.0, 1.0e-12);
}

TEST(OfflineLoopEdge, RejectsNonFiniteRegistration)
{
  uf_relocalization::RegistrationResult registration;
  registration.target_from_source = Eigen::Matrix4f::Identity();
  registration.target_from_source(0, 0) = std::numeric_limits<double>::quiet_NaN();

  EXPECT_THROW(
    uf_relocalization::make_offline_loop_edge(
      0U, 1U, 0.0, 1.0, Eigen::Isometry3d::Identity(), 0.1, registration),
    std::invalid_argument);
}
