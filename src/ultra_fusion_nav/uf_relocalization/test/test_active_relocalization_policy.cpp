#include "uf_relocalization/active_relocalization_policy.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace
{

uf_relocalization::ActiveRelocalizationEvidence stable_request()
{
  uf_relocalization::ActiveRelocalizationEvidence evidence;
  evidence.request_active = true;
  evidence.attitude_healthy = true;
  evidence.altitude_healthy = true;
  return evidence;
}

}  // namespace

TEST(ActiveRelocalizationPolicy, RemainsIdleWithoutRequest)
{
  const uf_relocalization::ActiveRelocalizationPolicy policy;
  const auto decision = policy.decide({});

  EXPECT_EQ(decision.action, uf_relocalization::ActiveRelocalizationAction::IDLE);
  EXPECT_EQ(decision.reason, "request_inactive");
}

TEST(ActiveRelocalizationPolicy, UsesPassiveBudgetBeforeCommandingMotion)
{
  const uf_relocalization::ActiveRelocalizationPolicy policy;
  auto evidence = stable_request();
  evidence.passive_attempts = 2U;

  const auto decision = policy.decide(evidence);

  EXPECT_EQ(
    decision.action, uf_relocalization::ActiveRelocalizationAction::PASSIVE_SEARCH);
}

TEST(ActiveRelocalizationPolicy, UsesYawScanBeforeEgoMotion)
{
  uf_relocalization::ActiveRelocalizationPolicyConfig config;
  config.ego_motion_enabled = true;
  const uf_relocalization::ActiveRelocalizationPolicy policy(config);
  auto evidence = stable_request();
  evidence.passive_attempts = 3U;
  evidence.yaw_scan_views_completed = 3U;
  evidence.local_odometry_healthy = true;
  evidence.obstacle_map_fresh = true;

  const auto decision = policy.decide(evidence);

  EXPECT_EQ(decision.action, uf_relocalization::ActiveRelocalizationAction::YAW_SCAN);
}

TEST(ActiveRelocalizationPolicy, AllowsEgoOnlyAfterAllSafetyGatesPass)
{
  uf_relocalization::ActiveRelocalizationPolicyConfig config;
  config.ego_motion_enabled = true;
  const uf_relocalization::ActiveRelocalizationPolicy policy(config);
  auto evidence = stable_request();
  evidence.passive_attempts = 3U;
  evidence.yaw_scan_views_completed = 4U;
  evidence.local_odometry_healthy = true;
  evidence.obstacle_map_fresh = true;

  const auto decision = policy.decide(evidence);

  EXPECT_EQ(
    decision.action, uf_relocalization::ActiveRelocalizationAction::EGO_SAFE_MOTION);
  EXPECT_STREQ(uf_relocalization::to_string(decision.action), "EGO_SAFE_MOTION");
}

TEST(ActiveRelocalizationPolicy, HoldsWhenEgoSafetyEvidenceIsMissing)
{
  uf_relocalization::ActiveRelocalizationPolicyConfig config;
  config.ego_motion_enabled = true;
  const uf_relocalization::ActiveRelocalizationPolicy policy(config);
  auto evidence = stable_request();
  evidence.passive_attempts = 3U;
  evidence.yaw_scan_views_completed = 4U;
  evidence.local_odometry_healthy = true;

  const auto stale_map = policy.decide(evidence);
  evidence.local_odometry_healthy = false;
  evidence.obstacle_map_fresh = true;
  const auto lost_odometry = policy.decide(evidence);

  EXPECT_EQ(
    stale_map.action, uf_relocalization::ActiveRelocalizationAction::HOLD_POSITION);
  EXPECT_EQ(stale_map.reason, "obstacle_map_stale");
  EXPECT_EQ(
    lost_odometry.action, uf_relocalization::ActiveRelocalizationAction::HOLD_POSITION);
  EXPECT_EQ(lost_odometry.reason, "local_odometry_unhealthy");
}

TEST(ActiveRelocalizationPolicy, EntersFailsafeWithoutAttitudeOrAltitude)
{
  const uf_relocalization::ActiveRelocalizationPolicy policy;
  auto evidence = stable_request();
  evidence.attitude_healthy = false;
  EXPECT_EQ(
    policy.decide(evidence).action,
    uf_relocalization::ActiveRelocalizationAction::FAILSAFE);

  evidence.attitude_healthy = true;
  evidence.altitude_healthy = false;
  EXPECT_EQ(
    policy.decide(evidence).action,
    uf_relocalization::ActiveRelocalizationAction::FAILSAFE);
}

TEST(ActiveRelocalizationPolicy, KeepsEgoDisabledByDefault)
{
  const uf_relocalization::ActiveRelocalizationPolicy policy;
  auto evidence = stable_request();
  evidence.passive_attempts = 3U;
  evidence.yaw_scan_views_completed = 4U;
  evidence.local_odometry_healthy = true;
  evidence.obstacle_map_fresh = true;

  const auto decision = policy.decide(evidence);

  EXPECT_EQ(
    decision.action, uf_relocalization::ActiveRelocalizationAction::HOLD_POSITION);
  EXPECT_EQ(decision.reason, "ego_experiment_disabled");
}

TEST(ActiveRelocalizationPolicy, RejectsZeroAttemptLimits)
{
  uf_relocalization::ActiveRelocalizationPolicyConfig invalid;
  invalid.passive_attempt_limit = 0U;
  EXPECT_THROW(
    uf_relocalization::ActiveRelocalizationPolicy{invalid}, std::invalid_argument);

  invalid.passive_attempt_limit = 3U;
  invalid.yaw_scan_view_count = 0U;
  EXPECT_THROW(
    uf_relocalization::ActiveRelocalizationPolicy{invalid}, std::invalid_argument);
}
