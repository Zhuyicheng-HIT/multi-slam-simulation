#include "uf_relocalization/automatic_loop_closure_policy.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace
{

uf_relocalization::AutomaticLoopClosureEvidence ready_evidence()
{
  uf_relocalization::AutomaticLoopClosureEvidence evidence;
  evidence.scheduler_healthy = true;
  evidence.lidar_enabled = true;
  evidence.database_keyframes = 8U;
  evidence.nearby_historical_keyframes = 2U;
  evidence.query_stamp_s = 40.0;
  return evidence;
}

}  // namespace

TEST(AutomaticLoopClosurePolicy, StartsOnlyWithHealthyHistoricalDatabase)
{
  const uf_relocalization::AutomaticLoopClosureConfig config;
  const auto decision = uf_relocalization::evaluate_automatic_loop_closure(
    config, ready_evidence());
  EXPECT_TRUE(decision.start_search);
  EXPECT_EQ(decision.reason, "historical_place_search_due");
}

TEST(AutomaticLoopClosurePolicy, ManualAndActiveSearchesHavePriority)
{
  const uf_relocalization::AutomaticLoopClosureConfig config;
  auto evidence = ready_evidence();
  evidence.manual_request_asserted = true;
  EXPECT_FALSE(
    uf_relocalization::evaluate_automatic_loop_closure(config, evidence).start_search);

  evidence.manual_request_asserted = false;
  evidence.request_active = true;
  EXPECT_FALSE(
    uf_relocalization::evaluate_automatic_loop_closure(config, evidence).start_search);
}

TEST(AutomaticLoopClosurePolicy, EnforcesHealthDatabaseAndCooldown)
{
  const uf_relocalization::AutomaticLoopClosureConfig config;
  auto evidence = ready_evidence();
  evidence.scheduler_healthy = false;
  EXPECT_FALSE(
    uf_relocalization::evaluate_automatic_loop_closure(config, evidence).start_search);

  evidence.scheduler_healthy = true;
  evidence.database_keyframes = 5U;
  EXPECT_FALSE(
    uf_relocalization::evaluate_automatic_loop_closure(config, evidence).start_search);

  evidence.database_keyframes = 8U;
  evidence.nearby_historical_keyframes = 0U;
  EXPECT_FALSE(
    uf_relocalization::evaluate_automatic_loop_closure(config, evidence).start_search);

  evidence.nearby_historical_keyframes = 2U;
  evidence.last_search_started_s = 30.0;
  EXPECT_FALSE(
    uf_relocalization::evaluate_automatic_loop_closure(config, evidence).start_search);
  evidence.query_stamp_s = 45.0;
  EXPECT_TRUE(
    uf_relocalization::evaluate_automatic_loop_closure(config, evidence).start_search);
  evidence.query_stamp_s = 5.0;
  EXPECT_TRUE(
    uf_relocalization::evaluate_automatic_loop_closure(config, evidence).start_search);
}

TEST(AutomaticLoopClosurePolicy, HistoricalCandidateUsesSourceTime)
{
  const uf_relocalization::AutomaticLoopClosureConfig config;
  EXPECT_TRUE(uf_relocalization::automatic_loop_candidate_is_historical(
    config, 50.0, 30.0));
  EXPECT_FALSE(uf_relocalization::automatic_loop_candidate_is_historical(
    config, 49.9, 30.0));
  EXPECT_FALSE(uf_relocalization::automatic_loop_candidate_is_historical(
    config, 50.0, 55.0));
  EXPECT_FALSE(uf_relocalization::automatic_loop_candidate_is_historical(
    config, std::numeric_limits<double>::quiet_NaN(), 30.0));
}

TEST(AutomaticLoopClosurePolicy, RoutineCorrectionHasIndependentSafetyBound)
{
  const uf_relocalization::AutomaticLoopClosureConfig config;
  EXPECT_TRUE(uf_relocalization::automatic_loop_correction_is_safe(
    config, 0.50, 0.10));
  EXPECT_FALSE(uf_relocalization::automatic_loop_correction_is_safe(
    config, 0.80, 0.10));
  EXPECT_FALSE(uf_relocalization::automatic_loop_correction_is_safe(
    config, 0.50, 0.30));
  EXPECT_FALSE(uf_relocalization::automatic_loop_correction_is_safe(
    config, std::numeric_limits<double>::quiet_NaN(), 0.10));
}

TEST(AutomaticLoopClosurePolicy, SpatialPrefilterRejectsNovelAreas)
{
  const uf_relocalization::AutomaticLoopClosureConfig config;
  EXPECT_TRUE(uf_relocalization::automatic_loop_candidate_is_spatially_near(
    config, 2.99));
  EXPECT_FALSE(uf_relocalization::automatic_loop_candidate_is_spatially_near(
    config, 3.01));
  EXPECT_FALSE(uf_relocalization::automatic_loop_candidate_is_spatially_near(
    config, std::numeric_limits<double>::quiet_NaN()));
}

TEST(AutomaticLoopClosurePolicy, RejectsInvalidLimits)
{
  uf_relocalization::AutomaticLoopClosureConfig config;
  config.search_cooldown_s = 0.0;
  EXPECT_THROW(
    uf_relocalization::evaluate_automatic_loop_closure(config, ready_evidence()),
    std::invalid_argument);
}
