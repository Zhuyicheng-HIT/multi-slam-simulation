#include "uf_relocalization/keyframe_synchronization.hpp"

#include <gtest/gtest.h>

#include <optional>

namespace
{

using uf_relocalization::KeyframeSynchronizationState;
using uf_relocalization::TimestampEvidence;
using uf_relocalization::decide_keyframe_synchronization;

TimestampEvidence evidence(
  const bool matched, const std::optional<double> latest_stamp_s,
  const double tolerance_s)
{
  return TimestampEvidence{matched, latest_stamp_s, tolerance_s};
}

}  // namespace

TEST(KeyframeSynchronization, AuxiliaryOdometryCannotConsumeLiDARCloud)
{
  constexpr double cloud_stamp_s = 10.0;
  const std::optional<double> latest_auxiliary_odom_stamp_s = 10.2;
  std::optional<double> latest_optimized_map_pose_stamp_s;

  ASSERT_TRUE(latest_auxiliary_odom_stamp_s.has_value());
  EXPECT_GT(*latest_auxiliary_odom_stamp_s, cloud_stamp_s + 0.12);

  // Generic auxiliary odometry is deliberately absent from this contract.
  // Until the LiDAR-gated optimized map pose arrives, the cloud must wait even
  // if an unrelated fused state has already advanced beyond its timestamp.
  const auto waiting = decide_keyframe_synchronization(
    cloud_stamp_s,
    evidence(false, latest_optimized_map_pose_stamp_s, 0.12),
    evidence(true, 10.0, 0.12),
    evidence(true, 10.0, 0.08));
  EXPECT_EQ(waiting.state, KeyframeSynchronizationState::WAITING);
  EXPECT_EQ(waiting.reason, "awaiting_optimized_map_pose");

  latest_optimized_map_pose_stamp_s = 10.0;
  const auto matched = decide_keyframe_synchronization(
    cloud_stamp_s,
    evidence(true, latest_optimized_map_pose_stamp_s, 0.12),
    evidence(true, 10.0, 0.12),
    evidence(true, 10.0, 0.08));
  EXPECT_EQ(matched.state, KeyframeSynchronizationState::MATCHED);
}

TEST(KeyframeSynchronization, ExpiresOnlyAfterRequiredStreamPassesTolerance)
{
  const auto boundary = decide_keyframe_synchronization(
    10.0,
    evidence(false, 10.12, 0.12),
    evidence(true, 10.0, 0.12),
    evidence(true, 10.0, 0.08));
  EXPECT_EQ(boundary.state, KeyframeSynchronizationState::WAITING);

  const auto expired = decide_keyframe_synchronization(
    10.0,
    evidence(false, 10.121, 0.12),
    evidence(true, 10.0, 0.12),
    evidence(true, 10.0, 0.08));
  EXPECT_EQ(expired.state, KeyframeSynchronizationState::EXPIRED);
  EXPECT_EQ(expired.reason, "optimized_map_pose_advanced_past_tolerance");
}

TEST(KeyframeSynchronization, WaitsForAllThreeLiDARTimestampedInputs)
{
  const auto waiting = decide_keyframe_synchronization(
    10.0,
    evidence(true, 10.0, 0.12),
    evidence(true, 10.0, 0.12),
    evidence(false, 10.04, 0.08));
  EXPECT_EQ(waiting.state, KeyframeSynchronizationState::WAITING);
  EXPECT_EQ(waiting.reason, "awaiting_body_cloud");

  const auto expired = decide_keyframe_synchronization(
    10.0,
    evidence(true, 10.0, 0.12),
    evidence(true, 10.0, 0.12),
    evidence(false, 10.09, 0.08));
  EXPECT_EQ(expired.state, KeyframeSynchronizationState::EXPIRED);
  EXPECT_EQ(expired.reason, "body_cloud_advanced_past_tolerance");
}

TEST(KeyframeSynchronization, MissingStreamExpiresByRosTime)
{
  const auto waiting = decide_keyframe_synchronization(
    10.0,
    evidence(true, 10.0, 0.12),
    evidence(true, 10.0, 0.12),
    evidence(false, std::nullopt, 0.08),
    0.49, 0.50);
  EXPECT_EQ(waiting.state, KeyframeSynchronizationState::WAITING);

  const auto expired = decide_keyframe_synchronization(
    10.0,
    evidence(true, 10.0, 0.12),
    evidence(true, 10.0, 0.12),
    evidence(false, std::nullopt, 0.08),
    0.50, 0.50);
  EXPECT_EQ(expired.state, KeyframeSynchronizationState::EXPIRED);
  EXPECT_EQ(expired.reason, "waiting_timeout");

  const auto matched_at_timeout = decide_keyframe_synchronization(
    10.0,
    evidence(true, 10.0, 0.12),
    evidence(true, 10.0, 0.12),
    evidence(true, 10.0, 0.08),
    0.50, 0.50);
  EXPECT_EQ(matched_at_timeout.state, KeyframeSynchronizationState::MATCHED);
}
