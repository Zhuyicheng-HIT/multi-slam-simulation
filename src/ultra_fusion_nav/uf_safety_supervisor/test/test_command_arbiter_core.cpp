#include "uf_safety_supervisor/command_arbiter_core.hpp"

#include <gtest/gtest.h>

#include <limits>

using namespace uf_safety_supervisor;

namespace
{
PoseIntent pose(double stamp, double x = 0.0)
{
  PoseIntent value;
  value.received = true;
  value.finite = true;
  value.stamp_s = stamp;
  value.position = {x, 0.0, 1.0};
  return value;
}

CommandArbiterInput nominal()
{
  CommandArbiterInput input;
  input.now_s = 10.0;
  input.obstacle_state = ObstacleState::kClear;
  input.obstacle_healthy = true;
  input.obstacle_stamp_s = 9.9;
  input.current_pose = pose(9.9);
  input.mission = pose(9.9, 1.0);
  return input;
}
}  // namespace

TEST(CommandArbiterCore, MissionOwnsNormalCommand)
{
  const auto result = CommandArbiterCore().evaluate(nominal());
  EXPECT_EQ(result.action, CommandAction::kForward);
  EXPECT_EQ(result.owner, "mission");
}

TEST(CommandArbiterCore, PlannerOverridesMission)
{
  auto input = nominal();
  input.planner = pose(9.9, 1.5);
  const auto result = CommandArbiterCore().evaluate(input);
  EXPECT_EQ(result.owner, "local_planner");
}

TEST(CommandArbiterCore, RelocalizationOverridesPlannerWhenObstacleClear)
{
  auto input = nominal();
  input.planner = pose(9.9, 1.5);
  input.relocalization = pose(9.9, 0.4);
  EXPECT_EQ(CommandArbiterCore().evaluate(input).owner, "active_relocalization");
}

TEST(CommandArbiterCore, ObstacleBrakeOverridesActiveRelocalization)
{
  auto input = nominal();
  input.relocalization = pose(9.9, 0.4);
  input.obstacle_state = ObstacleState::kBrake;
  const auto result = CommandArbiterCore().evaluate(input);
  EXPECT_EQ(result.action, CommandAction::kHold);
  EXPECT_EQ(result.owner, "obstacle_safety");
}

TEST(CommandArbiterCore, LandOverridesLocalizationAndPlanner)
{
  auto input = nominal();
  input.land_requested = true;
  input.localization_hold = true;
  input.planner = pose(9.9, 1.5);
  EXPECT_EQ(CommandArbiterCore().evaluate(input).action, CommandAction::kLand);
}

TEST(CommandArbiterCore, LocalizationHoldOverridesPlanner)
{
  auto input = nominal();
  input.localization_hold = true;
  input.planner = pose(9.9, 1.5);
  EXPECT_EQ(CommandArbiterCore().evaluate(input).owner, "localization_safety");
}

TEST(CommandArbiterCore, ManualAndFcuReleaseAutomaticOwnership)
{
  auto input = nominal();
  input.manual_override = true;
  auto result = CommandArbiterCore().evaluate(input);
  EXPECT_EQ(result.action, CommandAction::kRelease);
  EXPECT_FALSE(result.publish_setpoint);
  input.manual_override = false;
  input.fcu_failsafe = true;
  EXPECT_EQ(CommandArbiterCore().evaluate(input).owner, "fcu_failsafe");
}

TEST(CommandArbiterCore, PlannerTimeoutFailsClosed)
{
  auto input = nominal();
  input.planner = pose(8.0, 1.0);
  const auto result = CommandArbiterCore().evaluate(input);
  EXPECT_TRUE(result.fail_closed);
  EXPECT_EQ(result.action, CommandAction::kHold);
}

TEST(CommandArbiterCore, NonfinitePlannerFailsClosed)
{
  auto input = nominal();
  input.planner = pose(9.9);
  input.planner.position.x() = std::numeric_limits<double>::quiet_NaN();
  input.planner.finite = false;
  EXPECT_TRUE(CommandArbiterCore().evaluate(input).fail_closed);
}

TEST(CommandArbiterCore, SetpointJumpFailsClosed)
{
  auto input = nominal();
  input.planner = pose(9.9, 8.0);
  const auto result = CommandArbiterCore().evaluate(input);
  EXPECT_EQ(result.reason, "local_planner_setpoint_jump");
  EXPECT_TRUE(result.fail_closed);
}

TEST(CommandArbiterCore, CautionLimitsSetpointStep)
{
  auto input = nominal();
  input.obstacle_state = ObstacleState::kCaution;
  const auto result = CommandArbiterCore().evaluate(input);
  EXPECT_EQ(result.action, CommandAction::kForward);
  EXPECT_NEAR((result.selected.position - input.current_pose.position).norm(), 0.30, 1.0e-9);
}

TEST(CommandArbiterCore, StaleObstacleStateFailsClosed)
{
  auto input = nominal();
  input.obstacle_stamp_s = 8.0;
  EXPECT_EQ(CommandArbiterCore().evaluate(input).owner, "obstacle_safety");
  EXPECT_TRUE(CommandArbiterCore().evaluate(input).fail_closed);
}
