#include "uf_dynamic_observer/epoch_reseed_guard.hpp"

#include <gtest/gtest.h>

namespace uf_dynamic_observer
{

TEST(EpochReseedGuard, InitialLioStateDoesNotInventReset)
{
  EpochReseedGuard guard(3U);
  const auto decision = guard.observe_lio_state(7U);
  EXPECT_TRUE(decision.accepted);
  EXPECT_FALSE(decision.reset_local_history);
  EXPECT_FALSE(decision.fail_open);
  EXPECT_EQ(guard.state(), DynamicEpochState::kReady);
}

TEST(EpochReseedGuard, LioLocalEpochChangeClearsAndRequiresBoundedReseed)
{
  EpochReseedGuard guard(3U);
  guard.observe_lio_state(2U);
  const auto changed = guard.observe_lio_state(3U);
  EXPECT_TRUE(changed.reset_local_history);
  EXPECT_TRUE(changed.fail_open);
  EXPECT_TRUE(guard.fail_open_required());
  EXPECT_FALSE(guard.observe_reseed_scan(true));
  EXPECT_FALSE(guard.observe_reseed_scan(false));
  EXPECT_FALSE(guard.observe_reseed_scan(true));
  EXPECT_TRUE(guard.observe_reseed_scan(true));
  EXPECT_FALSE(guard.fail_open_required());
}

TEST(EpochReseedGuard, StaleLioStateCannotUndoNewEpoch)
{
  EpochReseedGuard guard(2U);
  guard.observe_lio_state(5U);
  guard.observe_lio_state(6U);
  const auto stale = guard.observe_lio_state(5U);
  EXPECT_FALSE(stale.accepted);
  EXPECT_FALSE(stale.reset_local_history);
  EXPECT_EQ(guard.lio_reset_counter(), 6U);
}

TEST(EpochReseedGuard, SmallBackendCorrectionRetainsLioLocalHistory)
{
  EpochReseedGuard guard;
  guard.observe_lio_state(0U);
  const auto epoch = guard.observe_backend_epoch(true, 1U, 10U, 1U);
  EXPECT_TRUE(epoch.accepted);
  EXPECT_FALSE(epoch.reset_local_history);
  EXPECT_EQ(guard.state(), DynamicEpochState::kReady);
}

TEST(EpochReseedGuard, LargeTranslationAndYawBackendCorrectionsHaveSameFrameOwnership)
{
  EpochReseedGuard guard;
  guard.observe_lio_state(0U);
  const auto translation_epoch = guard.observe_backend_epoch(true, 1U, 11U, 1U);
  const auto yaw_epoch = guard.observe_backend_epoch(true, 1U, 12U, 2U);
  EXPECT_FALSE(translation_epoch.reset_local_history);
  EXPECT_FALSE(yaw_epoch.reset_local_history);
  EXPECT_EQ(guard.backend_epoch_count(), 2U);
  EXPECT_FALSE(guard.fail_open_required());
}

TEST(EpochReseedGuard, RepeatedAndFailedBackendReinitCannotMutateDynamicState)
{
  EpochReseedGuard guard;
  guard.observe_lio_state(0U);
  EXPECT_FALSE(guard.observe_backend_epoch(false, 1U, 20U, 1U).accepted);
  EXPECT_TRUE(guard.observe_backend_epoch(true, 1U, 20U, 1U).accepted);
  EXPECT_FALSE(guard.observe_backend_epoch(true, 1U, 20U, 1U).accepted);
  EXPECT_FALSE(guard.observe_backend_epoch(true, 1U, 19U, 1U).accepted);
  EXPECT_EQ(guard.backend_epoch_count(), 1U);
  EXPECT_EQ(guard.ignored_backend_epoch_count(), 3U);
  EXPECT_EQ(guard.state(), DynamicEpochState::kReady);
}

TEST(EpochReseedGuard, BackendReinitDuringDynamicOcclusionDoesNotDiscardLocalEvidence)
{
  EpochReseedGuard guard(4U);
  guard.observe_lio_state(8U);
  const auto epoch = guard.observe_backend_epoch(true, 4U, 99U, 12U);
  EXPECT_TRUE(epoch.accepted);
  EXPECT_FALSE(epoch.reset_local_history);
  EXPECT_FALSE(epoch.fail_open);
  EXPECT_EQ(guard.reseed_scans(), 0U);
}

TEST(EpochReseedGuard, InvalidConfigurationIsRejected)
{
  EXPECT_THROW(EpochReseedGuard guard(0U), std::invalid_argument);
}

}  // namespace uf_dynamic_observer
