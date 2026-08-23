#include "uf_safety_supervisor/obstacle_safety_core.hpp"

#include <gtest/gtest.h>

#include <limits>

using uf_safety_supervisor::ObstacleSafetyCore;
using uf_safety_supervisor::ObstacleSafetyInput;
using uf_safety_supervisor::ObstacleState;

namespace
{
ObstacleSafetyInput nominal(double speed = 0.5)
{
  ObstacleSafetyInput input;
  input.body_velocity = {speed, 0.0, 0.0};
  input.desired_direction_body = {1.0, 0.0, 0.0};
  input.body_points = {{8.0, 2.0, 0.0}};
  return input;
}
}  // namespace

TEST(ObstacleSafetyCore, StaticWallInBrakingEnvelopeBrakes)
{
  auto input = nominal();
  input.body_points = {{0.75, 0.0, 0.0}, {0.75, 0.2, 0.1}};
  const auto result = ObstacleSafetyCore().evaluate(input);
  EXPECT_EQ(result.state, ObstacleState::kBrake);
  EXPECT_NEAR(result.path_clearance_m, 0.43, 1.0e-9);
  EXPECT_EQ(result.reason, "braking_envelope_intrusion");
}

TEST(ObstacleSafetyCore, HighSpeedApproachUsesStoppingDistance)
{
  auto input = nominal(3.0);
  input.body_points = {{3.0, 0.0, 0.0}};
  const auto result = ObstacleSafetyCore().evaluate(input);
  EXPECT_EQ(result.state, ObstacleState::kBrake);
  EXPECT_GT(result.stopping_distance_m, result.path_clearance_m);
}

TEST(ObstacleSafetyCore, SuddenObstacleChangesClearToBrake)
{
  auto input = nominal(1.0);
  const ObstacleSafetyCore core;
  EXPECT_EQ(core.evaluate(input).state, ObstacleState::kClear);
  input.body_points = {{0.8, 0.0, 0.0}};
  EXPECT_EQ(core.evaluate(input).state, ObstacleState::kBrake);
}

TEST(ObstacleSafetyCore, OffCorridorPillarDoesNotBrake)
{
  auto input = nominal(1.0);
  input.body_points = {{1.0, 1.5, 0.0}};
  EXPECT_EQ(ObstacleSafetyCore().evaluate(input).state, ObstacleState::kClear);
}

TEST(ObstacleSafetyCore, RawLidarStaleFailsClosed)
{
  auto input = nominal();
  input.raw_sensor_healthy = false;
  const auto result = ObstacleSafetyCore().evaluate(input);
  EXPECT_EQ(result.state, ObstacleState::kHover);
  EXPECT_TRUE(result.fail_closed);
}

TEST(ObstacleSafetyCore, TimestampFailureFailsClosed)
{
  auto input = nominal();
  input.timestamps_valid = false;
  EXPECT_EQ(ObstacleSafetyCore().evaluate(input).state, ObstacleState::kHover);
}

TEST(ObstacleSafetyCore, NonfiniteRawPointFailsClosed)
{
  auto input = nominal();
  input.body_points.push_back({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
  EXPECT_EQ(ObstacleSafetyCore().evaluate(input).state, ObstacleState::kHover);
}

TEST(ObstacleSafetyCore, LocalizationLossStillBrakesInBodyFrame)
{
  auto input = nominal(1.2);
  input.localization_valid = false;
  input.desired_direction_body = input.body_velocity;
  input.body_points = {{1.0, 0.0, 0.0}};
  EXPECT_EQ(ObstacleSafetyCore().evaluate(input).state, ObstacleState::kBrake);
}

TEST(ObstacleSafetyCore, CautionBandDoesNotPretendClear)
{
  auto input = nominal(0.5);
  input.body_points = {{1.15, 0.0, 0.0}};
  EXPECT_EQ(ObstacleSafetyCore().evaluate(input).state, ObstacleState::kCaution);
}

namespace
{
double simulate_stop(double obstacle_distance, double speed, double appearance_s = 0.0)
{
  const ObstacleSafetyCore core;
  constexpr double dt = 0.005;
  double minimum_clearance = obstacle_distance - 0.32;
  for (double time = 0.0; time < 8.0 && speed > 1.0e-4; time += dt) {
    auto input = nominal(speed);
    input.body_points = time >= appearance_s ?
      std::vector<Eigen::Vector3d>{{obstacle_distance, 0.0, 0.0}} :
      std::vector<Eigen::Vector3d>{{20.0, 2.0, 0.0}};
    const auto state = core.evaluate(input).state;
    if (state == ObstacleState::kBrake || state == ObstacleState::kHover) {
      speed = std::max(0.0, speed - 2.0 * dt);
    }
    obstacle_distance -= speed * dt;
    minimum_clearance = std::min(minimum_clearance, obstacle_distance - 0.32);
  }
  return minimum_clearance;
}
}  // namespace

TEST(ObstacleSafetyCore, KinematicStaticWallStopsWithoutCollision)
{
  EXPECT_GT(simulate_stop(2.0, 1.0), 0.30);
}

TEST(ObstacleSafetyCore, KinematicHighSpeedApproachStopsWithoutCollision)
{
  EXPECT_GT(simulate_stop(5.0, 3.0), 0.30);
}

TEST(ObstacleSafetyCore, KinematicSuddenObstacleStopsWithoutCollision)
{
  EXPECT_GT(simulate_stop(2.0, 1.0, 0.5), 0.20);
}
