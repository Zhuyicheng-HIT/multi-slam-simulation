#include "uf_safety_supervisor/command_arbiter_core.hpp"
#include "uf_safety_supervisor/obstacle_safety_core.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace uf_safety_supervisor;

double percentile(std::vector<double> values, double q)
{
  std::sort(values.begin(), values.end());
  return values[static_cast<std::size_t>(q * static_cast<double>(values.size() - 1U))];
}

double simulated_minimum_clearance(double obstacle_distance, double speed, double appearance_s = 0.0)
{
  const ObstacleSafetyCore core;
  constexpr double dt = 0.005;
  double minimum_clearance = obstacle_distance - 0.32;
  for (double time = 0.0; time < 8.0 && speed > 1.0e-4; time += dt) {
    ObstacleSafetyInput input;
    input.body_velocity = {speed, 0.0, 0.0};
    input.desired_direction_body = {1.0, 0.0, 0.0};
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

int main()
{
  ObstacleSafetyCore obstacle;
  CommandArbiterCore arbiter;
  ObstacleSafetyInput scan;
  scan.body_velocity = {1.0, 0.0, 0.0};
  scan.desired_direction_body = {1.0, 0.0, 0.0};
  scan.body_points.reserve(20000U);
  for (std::size_t i = 0; i < 20000U; ++i) {
    scan.body_points.emplace_back(
      5.0 + static_cast<double>(i % 100U) * 0.01,
      -2.0 + static_cast<double>(i % 200U) * 0.02,
      -0.5 + static_cast<double>(i % 50U) * 0.02);
  }
  CommandArbiterInput command;
  command.now_s = 10.0;
  command.obstacle_healthy = true;
  command.obstacle_stamp_s = 10.0;
  command.obstacle_state = ObstacleState::kClear;
  command.current_pose = {true, true, 10.0, {0.0, 0.0, 1.0}, 0.0};
  command.mission = {true, true, 10.0, {1.0, 0.0, 1.0}, 0.0};

  std::vector<double> monitor_us;
  std::vector<double> arbiter_us;
  for (std::size_t iteration = 0; iteration < 500U; ++iteration) {
    const auto begin = std::chrono::steady_clock::now();
    const auto result = obstacle.evaluate(scan);
    const auto middle = std::chrono::steady_clock::now();
    command.obstacle_state = result.state;
    const auto decision = arbiter.evaluate(command);
    const auto end = std::chrono::steady_clock::now();
    if (!decision.publish_setpoint) {return 2;}
    monitor_us.push_back(std::chrono::duration<double, std::micro>(middle - begin).count());
    arbiter_us.push_back(std::chrono::duration<double, std::micro>(end - middle).count());
  }
  std::cout << std::fixed << std::setprecision(3)
            << "raw_points=20000 iterations=500 "
            << "monitor_p50_us=" << percentile(monitor_us, 0.50) << ' '
            << "monitor_p95_us=" << percentile(monitor_us, 0.95) << ' '
            << "monitor_p99_us=" << percentile(monitor_us, 0.99) << ' '
            << "arbiter_p50_us=" << percentile(arbiter_us, 0.50) << ' '
            << "arbiter_p95_us=" << percentile(arbiter_us, 0.95) << ' '
            << "arbiter_p99_us=" << percentile(arbiter_us, 0.99) << ' '
            << "static_wall_min_clearance_m=" << simulated_minimum_clearance(2.0, 1.0) << ' '
            << "high_speed_min_clearance_m=" << simulated_minimum_clearance(5.0, 3.0) << ' '
            << "sudden_obstacle_min_clearance_m=" <<
      simulated_minimum_clearance(2.0, 1.0, 0.5) << '\n';
  return 0;
}
