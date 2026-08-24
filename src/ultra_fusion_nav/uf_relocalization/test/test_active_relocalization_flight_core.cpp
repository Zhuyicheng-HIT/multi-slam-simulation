#include "uf_relocalization/active_relocalization_flight_core.hpp"

#include <gtest/gtest.h>

using namespace uf_relocalization;

namespace
{
ActiveFlightEvent healthy(const double now)
{
  ActiveFlightEvent event;
  event.now_s = now;
  event.request_active = true;
  event.pose_healthy = true;
  event.stabilization_healthy = true;
  event.action_available = true;
  event.action_safe = true;
  return event;
}

ActiveRelocalizationFlightCore active_core()
{
  ActiveFlightConfig config;
  config.initial_hold_s = 1.0;
  config.active_timeout_s = 10.0;
  config.recovery_dwell_s = 0.5;
  config.resume_dwell_s = 0.2;
  return ActiveRelocalizationFlightCore(config);
}
}  // namespace

TEST(ActiveRelocalizationFlightCore, CompletesHoldActiveEpochRecoveryResume)
{
  auto core = active_core();
  EXPECT_EQ(core.update(healthy(1.0)).state, ActiveFlightState::HOLD);
  EXPECT_EQ(core.update(healthy(2.1)).state, ActiveFlightState::ACTIVE_RELOCALIZATION);
  EXPECT_TRUE(core.update(healthy(2.2)).motion_authorized);

  auto accepted = healthy(3.0);
  accepted.relocalization_success = true;
  accepted.result_transaction_id = 44U;
  accepted.result_candidate_id = 7U;
  EXPECT_EQ(core.update(accepted).state, ActiveFlightState::RECOVERY_VALIDATION);
  EXPECT_FALSE(core.decision().motion_authorized);

  auto committed = healthy(3.1);
  committed.epoch_applied = true;
  committed.epoch_transaction_id = 44U;
  committed.epoch_candidate_id = 7U;
  committed.recovery_healthy = true;
  EXPECT_TRUE(core.update(committed).epoch_committed);
  EXPECT_EQ(core.decision().reason, "matching_epoch_awaiting_request_release");
  committed.request_active = false;
  committed.now_s = 3.2;
  EXPECT_EQ(core.update(committed).state, ActiveFlightState::RECOVERY_VALIDATION);
  committed.now_s = 3.8;
  EXPECT_EQ(core.update(committed).state, ActiveFlightState::RESUME);
  committed.now_s = 4.1;
  EXPECT_EQ(core.update(committed).state, ActiveFlightState::NORMAL_NAVIGATION);
}

TEST(ActiveRelocalizationFlightCore, MatchingEpochCannotResumeWhileRequestRemainsLatched)
{
  auto core = active_core();
  core.update(healthy(1.0));
  core.update(healthy(2.1));
  auto accepted = healthy(2.2);
  accepted.relocalization_success = true;
  accepted.result_transaction_id = 19U;
  accepted.result_candidate_id = 5U;
  core.update(accepted);
  auto committed = healthy(2.3);
  committed.epoch_applied = true;
  committed.epoch_transaction_id = 19U;
  committed.epoch_candidate_id = 5U;
  committed.recovery_healthy = true;
  core.update(committed);
  committed.now_s = 4.0;
  EXPECT_EQ(core.update(committed).state, ActiveFlightState::RECOVERY_VALIDATION);
}

TEST(ActiveRelocalizationFlightCore, CandidateAloneCannotResumeBeforeMatchingEpoch)
{
  auto core = active_core();
  core.update(healthy(1.0));
  core.update(healthy(2.1));
  auto accepted = healthy(2.2);
  accepted.relocalization_success = true;
  accepted.result_transaction_id = 12U;
  accepted.result_candidate_id = 3U;
  core.update(accepted);
  auto wrong = healthy(2.3);
  wrong.epoch_applied = true;
  wrong.epoch_transaction_id = 13U;
  wrong.epoch_candidate_id = 3U;
  wrong.recovery_healthy = true;
  EXPECT_EQ(core.update(wrong).state, ActiveFlightState::RECOVERY_VALIDATION);
  EXPECT_FALSE(core.decision().epoch_committed);
}

TEST(ActiveRelocalizationFlightCore, RawObstacleVetoPreventsMotionAuthorization)
{
  auto core = active_core();
  core.update(healthy(1.0));
  core.update(healthy(2.1));
  auto blocked = healthy(2.2);
  blocked.action_safe = false;
  const auto decision = core.update(blocked);
  EXPECT_EQ(decision.state, ActiveFlightState::ACTIVE_RELOCALIZATION);
  EXPECT_FALSE(decision.motion_authorized);
  EXPECT_EQ(decision.reason, "raw_obstacle_veto");
}

TEST(ActiveRelocalizationFlightCore, FailedOrTimedOutSearchLatchesFailsafeHold)
{
  auto core = active_core();
  core.update(healthy(1.0));
  core.update(healthy(2.1));
  auto failed = healthy(2.2);
  failed.relocalization_failure = true;
  auto decision = core.update(failed);
  EXPECT_EQ(decision.state, ActiveFlightState::FAILSAFE);
  EXPECT_TRUE(decision.localization_hold);
  EXPECT_FALSE(decision.motion_authorized);

  core.reset(0.0);
  core.update(healthy(1.0));
  core.update(healthy(2.1));
  decision = core.update(healthy(11.1));
  EXPECT_EQ(decision.state, ActiveFlightState::FAILSAFE);
  EXPECT_EQ(decision.reason, "active_relocalization_timeout");
}

TEST(ActiveRelocalizationFlightCore, MissingPoseAtRequestFailsClosed)
{
  auto core = active_core();
  auto event = healthy(1.0);
  event.pose_healthy = false;
  const auto decision = core.update(event);
  EXPECT_EQ(decision.state, ActiveFlightState::FAILSAFE);
  EXPECT_TRUE(decision.localization_hold);
}
