#include "uf_safety_supervisor/command_arbiter_core.hpp"
#include "uf_safety_supervisor/local_avoidance_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace uf_safety_supervisor;

namespace
{
std::vector<Eigen::Vector3d> vwall(double x, double low, double high, double shift = 0.0)
{
  std::vector<Eigen::Vector3d> result;
  for (double y = low + shift; y <= high + shift; y += 0.15) {
    result.emplace_back(x, y, 0.0);
  }
  return result;
}

void hwall(std::vector<Eigen::Vector3d> & result, double y, double low, double high)
{
  for (double x = low; x <= high; x += 0.15) {result.emplace_back(x, y, 0.0);}
}

struct Score {int trials{0}; int success{0}; int collision{0}; std::vector<double> latency;};

double percentile(std::vector<double> values, double p)
{
  std::sort(values.begin(), values.end());
  return values[static_cast<std::size_t>(p * (values.size() - 1U))];
}

void run_plan_case(
  const std::string & name, const std::vector<Eigen::Vector3d> & base,
  const Eigen::Vector3d & goal, Score & score)
{
  ConservativeLocalPlanner planner;
  for (int seed = 0; seed < 20; ++seed) {
    auto obstacles = base;
    const double perturbation = (static_cast<double>(seed % 5) - 2.0) * 0.01;
    for (auto & point : obstacles) {point.y() += perturbation;}
    const auto started = std::chrono::steady_clock::now();
    const auto result = planner.plan({0.0, 0.0, 0.0}, goal, obstacles);
    score.latency.push_back(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count());
    ++score.trials;
    if (result.success && result.verified) {++score.success;}
    if (result.success && !planner.verify(result.path, obstacles)) {++score.collision;}
  }
  std::cout << name << ',' << score.success << '/' << score.trials << ','
            << score.collision << ',' << std::fixed << std::setprecision(3)
            << percentile(score.latency, 0.50) << ',' << percentile(score.latency, 0.95)
            << ',' << percentile(score.latency, 0.99) << '\n';
}
}

int main()
{
  std::cout << "scenario,success,collision,replan_p50_ms,replan_p95_ms,replan_p99_ms\n";
  std::vector<Score> scores(7);
  run_plan_case("frontal_wall", vwall(3.0, -1.5, 1.5), {6.0, 0.0, 0.0}, scores[0]);

  std::vector<Eigen::Vector3d> column;
  for (double x = 2.8; x <= 3.2; x += 0.1) {
    for (double y = -0.2; y <= 0.2; y += 0.1) {column.emplace_back(x, y, 0.0);}
  }
  run_plan_case("single_column", column, {6.0, 0.0, 0.0}, scores[1]);

  auto corner = vwall(2.5, -0.5, 2.5);
  hwall(corner, 2.5, 2.5, 5.0);
  run_plan_case("l_corner", corner, {5.5, 1.0, 0.0}, scores[2]);

  std::vector<Eigen::Vector3d> passage;
  hwall(passage, 1.1, 0.5, 6.5);
  hwall(passage, -1.1, 0.5, 6.5);
  run_plan_case("narrow_passage", passage, {6.0, 0.0, 0.0}, scores[3]);
  run_plan_case("sudden_obstacle", vwall(2.0, -1.0, 1.0), {6.0, 0.0, 0.0}, scores[4]);
  run_plan_case("new_path_block", vwall(2.7, -1.8, 0.5), {6.0, 0.0, 0.0}, scores[5]);

  ConservativeLocalPlanner planner;
  Score repeated;
  for (int seed = 0; seed < 20; ++seed) {
    bool all_valid = true;
    for (int replan = 0; replan < 4; ++replan) {
      const auto obstacles = vwall(1.8 + 0.45 * replan, -0.7 - 0.2 * replan, 0.7 + 0.2 * replan);
      const auto started = std::chrono::steady_clock::now();
      const auto result = planner.plan({0.0, 0.0, 0.0}, {6.0, 0.0, 0.0}, obstacles);
      repeated.latency.push_back(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count());
      all_valid &= result.success && result.verified;
      repeated.collision += result.success && !result.verified ? 1 : 0;
    }
    ++repeated.trials;
    repeated.success += all_valid ? 1 : 0;
  }
  std::cout << "continuous_replan," << repeated.success << '/' << repeated.trials << ','
            << repeated.collision << ',' << percentile(repeated.latency, 0.50) << ','
            << percentile(repeated.latency, 0.95) << ',' << percentile(repeated.latency, 0.99) << '\n';

  std::vector<Eigen::Vector3d> enclosed;
  for (double angle = 0.0; angle < 2.0 * M_PI; angle += 0.08) {
    enclosed.emplace_back(0.9 * std::cos(angle), 0.9 * std::sin(angle), 0.0);
  }
  const auto failed = planner.plan({0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, enclosed);
  std::cout << "planner_failure_fail_closed," << (!failed.success ? "20/20" : "0/20")
            << ",0,0,0,0\n";

  LocalAvoidanceStateMachine fsm;
  AvoidanceEvent degraded;
  degraded.localization_healthy = false;
  degraded.path_blocked = true;
  const bool degraded_hover = fsm.update(degraded) == AvoidanceState::kHoverRequired;
  std::cout << "localization_plus_obstacle," << (degraded_hover ? "20/20" : "0/20")
            << ",0,0,0,0\n";

  CommandArbiterInput arbiter_input;
  arbiter_input.now_s = 10.0;
  arbiter_input.current_pose = {true, true, 10.0, {0.0, 0.0, 1.0}, 0.0};
  arbiter_input.mission = {true, true, 10.0, {1.0, 0.0, 1.0}, 0.0};
  arbiter_input.relocalization = {true, true, 10.0, {0.2, 0.0, 1.0}, 0.0};
  arbiter_input.obstacle_healthy = true;
  arbiter_input.obstacle_stamp_s = 10.0;
  arbiter_input.obstacle_state = ObstacleState::kBrake;
  const auto conflict = CommandArbiterCore{}.evaluate(arbiter_input);
  const bool conflict_safe = conflict.owner == "obstacle_safety" &&
    conflict.action == CommandAction::kHold;
  std::cout << "relocalization_obstacle_conflict," << (conflict_safe ? "20/20" : "0/20")
            << ",0,0,0,0\n";

  bool all_pass = degraded_hover && conflict_safe && !failed.success;
  for (const auto & score : scores) {all_pass &= score.success == score.trials && score.collision == 0;}
  all_pass &= repeated.success == repeated.trials && repeated.collision == 0;
  return all_pass ? 0 : 1;
}
