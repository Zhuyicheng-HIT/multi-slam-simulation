#include "uf_safety_supervisor/local_avoidance_core.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

using uf_safety_supervisor::AvoidanceEvent;
using uf_safety_supervisor::AvoidanceState;
using uf_safety_supervisor::ConservativeLocalPlanner;
using uf_safety_supervisor::LocalAvoidanceStateMachine;
using uf_safety_supervisor::LocalPlannerConfig;

namespace
{

std::vector<Eigen::Vector3d> vertical_wall(
  const double x, const double y_min, const double y_max)
{
  std::vector<Eigen::Vector3d> result;
  for (double y = y_min; y <= y_max + 1.0e-9; y += 0.15) {
    result.emplace_back(x, y, 0.0);
  }
  return result;
}

void horizontal_wall(
  std::vector<Eigen::Vector3d> & points, const double y,
  const double x_min, const double x_max)
{
  for (double x = x_min; x <= x_max + 1.0e-9; x += 0.15) {
    points.emplace_back(x, y, 0.0);
  }
}

void expect_safe(const ConservativeLocalPlanner & planner,
  const uf_safety_supervisor::LocalPlanResult & result,
  const std::vector<Eigen::Vector3d> & obstacles)
{
  ASSERT_TRUE(result.success) << result.reason;
  ASSERT_TRUE(result.verified);
  ASSERT_GE(result.path.size(), 2U);
  double clearance = 0.0;
  EXPECT_TRUE(planner.verify(result.path, obstacles, &clearance));
  EXPECT_GE(clearance, 0.65 - 1.0e-9);
}

}  // namespace

TEST(LocalAvoidance, DirectGoalStaysStraight)
{
  ConservativeLocalPlanner planner;
  const auto result = planner.plan({0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, {});
  expect_safe(planner, result, {});
  EXPECT_NEAR(result.path.back().y(), 0.0, 1.0e-9);
}

TEST(LocalAvoidance, FrontalWallProducesVerifiedDetour)
{
  ConservativeLocalPlanner planner;
  const auto obstacles = vertical_wall(3.0, -1.5, 1.5);
  const auto result = planner.plan({0.0, 0.0, 0.0}, {6.0, 0.0, 0.0}, obstacles);
  expect_safe(planner, result, obstacles);
  bool left_direct_line = false;
  for (const auto & point : result.path) {left_direct_line |= std::abs(point.y()) > 1.5;}
  EXPECT_TRUE(left_direct_line);
}

TEST(LocalAvoidance, SingleColumnProducesVerifiedDetour)
{
  ConservativeLocalPlanner planner;
  std::vector<Eigen::Vector3d> obstacles;
  for (double x = 2.8; x <= 3.2; x += 0.1) {
    for (double y = -0.2; y <= 0.2; y += 0.1) {obstacles.emplace_back(x, y, 0.0);}
  }
  expect_safe(planner, planner.plan({0.0, 0.0, 0.0}, {6.0, 0.0, 0.0}, obstacles), obstacles);
}

TEST(LocalAvoidance, LCornerProducesContinuousPath)
{
  ConservativeLocalPlanner planner;
  auto obstacles = vertical_wall(2.5, -0.5, 2.5);
  horizontal_wall(obstacles, 2.5, 2.5, 5.0);
  const auto result = planner.plan({0.0, 0.0, 0.0}, {5.5, 1.0, 0.0}, obstacles);
  expect_safe(planner, result, obstacles);
  for (std::size_t i = 1; i < result.path.size(); ++i) {
    EXPECT_LE((result.path[i] - result.path[i - 1]).norm(), 0.76);
  }
}

TEST(LocalAvoidance, NarrowPassagePreservesVehicleClearance)
{
  ConservativeLocalPlanner planner;
  std::vector<Eigen::Vector3d> obstacles;
  horizontal_wall(obstacles, 1.1, 0.5, 6.5);
  horizontal_wall(obstacles, -1.1, 0.5, 6.5);
  expect_safe(planner, planner.plan({0.0, 0.0, 0.0}, {6.0, 0.0, 0.0}, obstacles), obstacles);
}

TEST(LocalAvoidance, SuddenObstacleInvalidatesOldPathAndReplans)
{
  ConservativeLocalPlanner planner;
  const auto clear = planner.plan({0.0, 0.0, 0.0}, {6.0, 0.0, 0.0}, {});
  ASSERT_TRUE(clear.success);
  const auto obstacles = vertical_wall(2.0, -1.0, 1.0);
  EXPECT_FALSE(planner.verify(clear.path, obstacles));
  expect_safe(planner, planner.plan({0.0, 0.0, 0.0}, {6.0, 0.0, 0.0}, obstacles), obstacles);
}

TEST(LocalAvoidance, ConsecutiveReplansRemainCollisionFree)
{
  ConservativeLocalPlanner planner;
  for (int pass = 0; pass < 4; ++pass) {
    const auto obstacles = vertical_wall(1.8 + 0.45 * pass, -0.7 - 0.2 * pass, 0.7 + 0.2 * pass);
    const auto result = planner.plan({0.0, 0.0, 0.0}, {6.0, 0.0, 0.0}, obstacles);
    expect_safe(planner, result, obstacles);
  }
}

TEST(LocalAvoidance, FullyBlockedGoalFailsClosed)
{
  ConservativeLocalPlanner planner;
  std::vector<Eigen::Vector3d> obstacles;
  for (double angle = 0.0; angle < 2.0 * M_PI; angle += 0.08) {
    obstacles.emplace_back(0.9 * std::cos(angle), 0.9 * std::sin(angle), 0.0);
  }
  const auto result = planner.plan({0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, obstacles);
  EXPECT_FALSE(result.success);
}

TEST(LocalAvoidance, NonfiniteObstacleIsRejected)
{
  ConservativeLocalPlanner planner;
  const std::vector<Eigen::Vector3d> obstacles{{
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}};
  EXPECT_FALSE(planner.plan({0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, obstacles).success);
}

TEST(LocalAvoidance, PlannerLatencyIsBoundedForRepresentativeWall)
{
  ConservativeLocalPlanner planner;
  const auto obstacles = vertical_wall(3.0, -1.5, 1.5);
  const auto started = std::chrono::steady_clock::now();
  for (int run = 0; run < 50; ++run) {
    ASSERT_TRUE(planner.plan({0.0, 0.0, 0.0}, {6.0, 0.0, 0.0}, obstacles).success);
  }
  const double average_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count() / 50.0;
  EXPECT_LT(average_ms, 80.0);
}

TEST(LocalAvoidanceFSM, CompletesBrakeReplanVerifyResumeSequence)
{
  LocalAvoidanceStateMachine fsm;
  AvoidanceEvent event;
  event.path_blocked = true;
  EXPECT_EQ(fsm.update(event), AvoidanceState::kPathBlocked);
  EXPECT_EQ(fsm.update(event), AvoidanceState::kBrakeHold);
  EXPECT_EQ(fsm.update(event), AvoidanceState::kReplan);
  event.plan_attempted = true;
  event.plan_valid = true;
  EXPECT_EQ(fsm.update(event), AvoidanceState::kTrajectoryVerify);
  EXPECT_EQ(fsm.update(event), AvoidanceState::kResume);
  event.path_blocked = false;
  event.goal_reached = true;
  EXPECT_EQ(fsm.update(event), AvoidanceState::kNavigating);
}

TEST(LocalAvoidanceFSM, PlannerFailureAndSensorFaultHover)
{
  LocalAvoidanceStateMachine fsm;
  AvoidanceEvent event;
  event.raw_fresh = false;
  EXPECT_EQ(fsm.update(event), AvoidanceState::kHoverRequired);
  event.raw_fresh = true;
  event.localization_healthy = false;
  EXPECT_EQ(fsm.update(event), AvoidanceState::kHoverRequired);
  event.localization_healthy = true;
  event.path_blocked = true;
  EXPECT_EQ(fsm.update(event), AvoidanceState::kBrakeHold);
  EXPECT_EQ(fsm.update(event), AvoidanceState::kReplan);
  event.plan_attempted = true;
  event.plan_valid = false;
  EXPECT_EQ(fsm.update(event), AvoidanceState::kHoverRequired);
  EXPECT_EQ(fsm.update(event), AvoidanceState::kHoverRequired);
  EXPECT_EQ(fsm.reason(), "planner_failure_latched_path_still_blocked");

  event.path_blocked = false;
  EXPECT_EQ(fsm.update(event), AvoidanceState::kNavigating);
}
