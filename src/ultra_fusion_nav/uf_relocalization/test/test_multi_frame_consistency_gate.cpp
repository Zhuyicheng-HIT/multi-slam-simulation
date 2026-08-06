#include "uf_relocalization/multi_frame_consistency_gate.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <limits>

namespace
{

Eigen::Isometry3d hypothesis(const double x, const double yaw_rad = 0.0)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().x() = x;
  transform.linear() =
    Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return transform;
}

}  // namespace

TEST(MultiFrameConsistencyGate, ConfirmsThreePairwiseConsistentQueries)
{
  uf_relocalization::MultiFrameConsistencyGate gate;

  const auto first = gate.observe(100, hypothesis(0.00, 0.00));
  const auto second = gate.observe(200, hypothesis(0.07, 0.02));
  const auto third = gate.observe(300, hypothesis(0.14, 0.04));

  EXPECT_EQ(first.status, uf_relocalization::MultiFrameConsistencyStatus::ACCUMULATING);
  EXPECT_EQ(second.status, uf_relocalization::MultiFrameConsistencyStatus::ACCUMULATING);
  EXPECT_EQ(third.status, uf_relocalization::MultiFrameConsistencyStatus::CONFIRMED);
  EXPECT_EQ(third.consistent_queries, 3U);
}

TEST(MultiFrameConsistencyGate, RejectsChainedDriftThatExceedsPairwiseClusterLimit)
{
  uf_relocalization::MultiFrameConsistencyGate gate;

  EXPECT_EQ(
    gate.observe(100, hypothesis(0.00)).status,
    uf_relocalization::MultiFrameConsistencyStatus::ACCUMULATING);
  EXPECT_EQ(
    gate.observe(200, hypothesis(0.10)).status,
    uf_relocalization::MultiFrameConsistencyStatus::ACCUMULATING);
  const auto restarted = gate.observe(300, hypothesis(0.20));

  EXPECT_EQ(restarted.status, uf_relocalization::MultiFrameConsistencyStatus::RESTARTED);
  EXPECT_EQ(restarted.consistent_queries, 1U);
  EXPECT_GT(restarted.maximum_translation_delta_m, 0.15);
}

TEST(MultiFrameConsistencyGate, RotationDisagreementRestartsAtCurrentQuery)
{
  uf_relocalization::MultiFrameConsistencyGate gate;
  gate.observe(100, hypothesis(0.0, 0.00));

  const auto restarted = gate.observe(200, hypothesis(0.0, 0.06));

  EXPECT_EQ(restarted.status, uf_relocalization::MultiFrameConsistencyStatus::RESTARTED);
  EXPECT_EQ(restarted.consistent_queries, 1U);
  EXPECT_GT(restarted.maximum_rotation_delta_rad, 0.05);
  EXPECT_EQ(
    gate.observe(300, hypothesis(0.01, 0.065)).status,
    uf_relocalization::MultiFrameConsistencyStatus::ACCUMULATING);
  EXPECT_EQ(
    gate.observe(400, hypothesis(0.02, 0.070)).status,
    uf_relocalization::MultiFrameConsistencyStatus::CONFIRMED);
}

TEST(MultiFrameConsistencyGate, DuplicateAndStaleQueriesDoNotAdvanceSequence)
{
  uf_relocalization::MultiFrameConsistencyGate gate;
  gate.observe(100, hypothesis(0.0));

  const auto duplicate = gate.observe(100, hypothesis(0.01));
  const auto stale = gate.observe(99, hypothesis(0.01));

  EXPECT_EQ(
    duplicate.status, uf_relocalization::MultiFrameConsistencyStatus::STALE_OR_DUPLICATE);
  EXPECT_EQ(stale.status, uf_relocalization::MultiFrameConsistencyStatus::STALE_OR_DUPLICATE);
  EXPECT_EQ(gate.consistent_queries(), 1U);
}

TEST(MultiFrameConsistencyGate, ResetClearsPartialSequenceAndQueryToken)
{
  uf_relocalization::MultiFrameConsistencyGate gate;
  gate.observe(100, hypothesis(0.0));
  gate.observe(200, hypothesis(0.01));

  gate.reset();
  const auto first_after_reset = gate.observe(50, hypothesis(0.02));

  EXPECT_EQ(first_after_reset.status, uf_relocalization::MultiFrameConsistencyStatus::ACCUMULATING);
  EXPECT_EQ(first_after_reset.consistent_queries, 1U);
}

TEST(MultiFrameConsistencyGate, RejectsInvalidConfigurationAndTransform)
{
  uf_relocalization::MultiFrameConsistencyConfig invalid_config;
  invalid_config.required_queries = 0U;
  EXPECT_THROW(
    {
      const uf_relocalization::MultiFrameConsistencyGate invalid_gate{invalid_config};
      static_cast<void>(invalid_gate);
    },
    std::invalid_argument);

  uf_relocalization::MultiFrameConsistencyGate gate;
  auto invalid_transform = Eigen::Isometry3d::Identity();
  invalid_transform.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(gate.observe(1, invalid_transform), std::invalid_argument);
}
