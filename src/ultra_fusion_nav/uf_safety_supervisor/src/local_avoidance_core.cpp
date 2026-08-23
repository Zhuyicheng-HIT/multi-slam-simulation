#include "uf_safety_supervisor/local_avoidance_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace uf_safety_supervisor
{
namespace
{

struct Cell
{
  int x{0};
  int y{0};
  bool operator==(const Cell & other) const {return x == other.x && y == other.y;}
};

std::int64_t key(const Cell cell)
{
  const auto x = static_cast<std::uint64_t>(static_cast<std::uint32_t>(cell.x));
  const auto y = static_cast<std::uint64_t>(static_cast<std::uint32_t>(cell.y));
  return static_cast<std::int64_t>((x << 32) | y);
}

double point_segment_distance_xy(
  const Eigen::Vector3d & point, const Eigen::Vector3d & start,
  const Eigen::Vector3d & goal)
{
  const Eigen::Vector2d p = point.head<2>();
  const Eigen::Vector2d a = start.head<2>();
  const Eigen::Vector2d delta = goal.head<2>() - a;
  const double denominator = delta.squaredNorm();
  const double t = denominator > 1.0e-12 ?
    std::clamp((p - a).dot(delta) / denominator, 0.0, 1.0) : 0.0;
  return (p - (a + t * delta)).norm();
}

bool path_segment_clear(
  const Eigen::Vector3d & start, const Eigen::Vector3d & goal,
  const std::vector<Eigen::Vector3d> & obstacles, const double required_clearance,
  double * minimum_clearance_m = nullptr)
{
  if (!start.allFinite() || !goal.allFinite()) {return false;}
  double minimum = std::numeric_limits<double>::infinity();
  for (const auto & obstacle : obstacles) {
    if (!obstacle.allFinite()) {return false;}
    minimum = std::min(minimum, point_segment_distance_xy(obstacle, start, goal));
  }
  if (minimum_clearance_m != nullptr) {*minimum_clearance_m = minimum;}
  return minimum >= required_clearance;
}

}  // namespace

ConservativeLocalPlanner::ConservativeLocalPlanner(LocalPlannerConfig config) : config_(config)
{
  if (!(config_.resolution_m > 0.0) || !(config_.obstacle_inflation_m > 0.0) ||
    !(config_.verification_clearance_m > 0.0) ||
    config_.obstacle_inflation_m < config_.verification_clearance_m ||
    !(config_.planning_horizon_m > 0.0) || !(config_.side_padding_m > 0.0) ||
    config_.maximum_expansions == 0U)
  {
    throw std::invalid_argument("invalid local planner configuration");
  }
}

bool ConservativeLocalPlanner::segment_clear(
  const Eigen::Vector3d & start, const Eigen::Vector3d & goal,
  const std::vector<Eigen::Vector3d> & obstacles, double * minimum_clearance_m) const
{
  return path_segment_clear(
    start, goal, obstacles, config_.verification_clearance_m, minimum_clearance_m);
}

bool ConservativeLocalPlanner::verify(
  const std::vector<Eigen::Vector3d> & path,
  const std::vector<Eigen::Vector3d> & obstacles, double * minimum_clearance_m) const
{
  if (path.size() < 2U) {return false;}
  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1; index < path.size(); ++index) {
    double clearance = 0.0;
    if (!segment_clear(path[index - 1U], path[index], obstacles, &clearance)) {
      if (minimum_clearance_m != nullptr) {*minimum_clearance_m = clearance;}
      return false;
    }
    minimum = std::min(minimum, clearance);
  }
  if (minimum_clearance_m != nullptr) {*minimum_clearance_m = minimum;}
  return true;
}

LocalPlanResult ConservativeLocalPlanner::plan(
  const Eigen::Vector3d & start, const Eigen::Vector3d & requested_goal,
  const std::vector<Eigen::Vector3d> & obstacles) const
{
  LocalPlanResult result;
  if (!start.allFinite() || !requested_goal.allFinite()) {
    result.reason = "start_or_goal_nonfinite";
    return result;
  }
  Eigen::Vector3d goal = requested_goal;
  const Eigen::Vector2d goal_delta = (goal - start).head<2>();
  if (goal_delta.norm() > config_.planning_horizon_m) {
    goal.head<2>() = start.head<2>() +
      goal_delta.normalized() * config_.planning_horizon_m;
  }
  goal.z() = start.z();
  if ((goal - start).head<2>().norm() <= config_.goal_tolerance_m) {
    result.success = true;
    result.verified = true;
    result.reason = "goal_within_tolerance";
    result.path = {start, goal};
    result.minimum_clearance_m = std::numeric_limits<double>::infinity();
    return result;
  }

  const double min_x = std::min(start.x(), goal.x()) - config_.side_padding_m;
  const double min_y = std::min(start.y(), goal.y()) - config_.side_padding_m;
  const double max_x = std::max(start.x(), goal.x()) + config_.side_padding_m;
  const double max_y = std::max(start.y(), goal.y()) + config_.side_padding_m;
  const int width = static_cast<int>(std::ceil((max_x - min_x) / config_.resolution_m)) + 1;
  const int height = static_cast<int>(std::ceil((max_y - min_y) / config_.resolution_m)) + 1;
  if (width <= 2 || height <= 2 || static_cast<std::size_t>(width) * height > 250000U) {
    result.reason = "planning_grid_invalid";
    return result;
  }
  const auto to_cell = [&](const Eigen::Vector3d & point) {
      return Cell{
        static_cast<int>(std::lround((point.x() - min_x) / config_.resolution_m)),
        static_cast<int>(std::lround((point.y() - min_y) / config_.resolution_m))};
    };
  const auto to_point = [&](const Cell cell) {
      return Eigen::Vector3d{
        min_x + cell.x * config_.resolution_m,
        min_y + cell.y * config_.resolution_m, start.z()};
    };
  const auto inside = [&](const Cell cell) {
      return cell.x >= 0 && cell.y >= 0 && cell.x < width && cell.y < height;
    };

  std::vector<std::uint8_t> occupied(static_cast<std::size_t>(width) * height, 0U);
  const int inflation_cells = static_cast<int>(
    std::ceil(config_.obstacle_inflation_m / config_.resolution_m));
  for (const auto & obstacle : obstacles) {
    if (!obstacle.allFinite()) {
      result.reason = "obstacle_nonfinite";
      return result;
    }
    const Cell center = to_cell(obstacle);
    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        const Cell cell{center.x + dx, center.y + dy};
        if (!inside(cell)) {continue;}
        if (std::hypot(dx, dy) * config_.resolution_m <= config_.obstacle_inflation_m) {
          occupied[static_cast<std::size_t>(cell.y) * width + cell.x] = 1U;
        }
      }
    }
  }
  const Cell start_cell = to_cell(start);
  const Cell goal_cell = to_cell(goal);
  const auto blocked = [&](const Cell cell) {
      return !inside(cell) || occupied[static_cast<std::size_t>(cell.y) * width + cell.x] != 0U;
    };
  if (blocked(start_cell) || blocked(goal_cell)) {
    result.reason = blocked(start_cell) ? "start_in_collision" : "local_goal_in_collision";
    return result;
  }

  struct QueueEntry {double score; Cell cell;};
  struct Greater {bool operator()(const QueueEntry & a, const QueueEntry & b) const {
      return a.score > b.score;
    }};
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, Greater> open;
  std::unordered_map<std::int64_t, double> cost;
  std::unordered_map<std::int64_t, Cell> parent;
  cost[key(start_cell)] = 0.0;
  open.push({0.0, start_cell});
  const std::array<Cell, 8> neighbors{{
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};
  bool found = false;
  while (!open.empty() && result.expanded_nodes < config_.maximum_expansions) {
    const Cell current = open.top().cell;
    open.pop();
    ++result.expanded_nodes;
    if (current == goal_cell) {found = true; break;}
    const double current_cost = cost[key(current)];
    for (const auto offset : neighbors) {
      const Cell next{current.x + offset.x, current.y + offset.y};
      if (blocked(next)) {continue;}
      if (offset.x != 0 && offset.y != 0 &&
        (blocked({current.x + offset.x, current.y}) ||
        blocked({current.x, current.y + offset.y})))
      {
        continue;
      }
      const double step = std::hypot(offset.x, offset.y) * config_.resolution_m;
      const double candidate = current_cost + step;
      const auto next_key = key(next);
      const auto existing = cost.find(next_key);
      if (existing != cost.end() && candidate >= existing->second) {continue;}
      cost[next_key] = candidate;
      parent[next_key] = current;
      const double heuristic = std::hypot(next.x - goal_cell.x, next.y - goal_cell.y) *
        config_.resolution_m;
      open.push({candidate + heuristic, next});
    }
  }
  if (!found) {
    result.reason = result.expanded_nodes >= config_.maximum_expansions ?
      "expansion_limit" : "no_collision_free_path";
    return result;
  }

  std::vector<Eigen::Vector3d> reversed{goal};
  Cell cursor = goal_cell;
  while (!(cursor == start_cell)) {
    const auto item = parent.find(key(cursor));
    if (item == parent.end()) {
      result.reason = "parent_chain_incomplete";
      return result;
    }
    cursor = item->second;
    reversed.push_back(to_point(cursor));
  }
  std::reverse(reversed.begin(), reversed.end());
  reversed.front() = start;
  reversed.back() = goal;

  std::vector<Eigen::Vector3d> simplified{reversed.front()};
  std::size_t anchor = 0U;
  while (anchor + 1U < reversed.size()) {
    std::size_t farthest = anchor + 1U;
    for (std::size_t candidate = anchor + 2U; candidate < reversed.size(); ++candidate) {
      if (path_segment_clear(
          reversed[anchor], reversed[candidate], obstacles,
          config_.obstacle_inflation_m))
      {
        farthest = candidate;
      } else {
        break;
      }
    }
    simplified.push_back(reversed[farthest]);
    anchor = farthest;
  }

  std::vector<Eigen::Vector3d> spaced{simplified.front()};
  for (std::size_t index = 1U; index < simplified.size(); ++index) {
    const Eigen::Vector3d delta = simplified[index] - spaced.back();
    const int pieces = std::max(1, static_cast<int>(
      std::ceil(delta.head<2>().norm() / config_.waypoint_spacing_m)));
    const Eigen::Vector3d base = spaced.back();
    for (int piece = 1; piece <= pieces; ++piece) {
      spaced.push_back(base + delta * (static_cast<double>(piece) / pieces));
    }
  }
  result.path = std::move(spaced);
  result.verified = verify(result.path, obstacles, &result.minimum_clearance_m);
  result.success = result.verified;
  result.reason = result.verified ? "verified_collision_free_path" : "post_smoothing_collision";
  return result;
}

AvoidanceState LocalAvoidanceStateMachine::update(const AvoidanceEvent & event)
{
  if (!event.raw_fresh) {
    state_ = AvoidanceState::kHoverRequired;
    reason_ = "raw_obstacle_map_stale";
    return state_;
  }
  if (!event.localization_healthy) {
    state_ = AvoidanceState::kHoverRequired;
    reason_ = "localization_degraded";
    return state_;
  }
  if (!event.mission_fresh) {
    state_ = AvoidanceState::kHoverRequired;
    reason_ = "mission_goal_stale_or_invalid";
    return state_;
  }
  switch (state_) {
    case AvoidanceState::kNavigating:
      if (event.path_blocked) {
        state_ = AvoidanceState::kPathBlocked;
        reason_ = "raw_path_blocked";
      } else {
        reason_ = "mission_direct";
      }
      break;
    case AvoidanceState::kPathBlocked:
      state_ = AvoidanceState::kBrakeHold;
      reason_ = "brake_before_replan";
      break;
    case AvoidanceState::kBrakeHold:
      state_ = AvoidanceState::kReplan;
      reason_ = "vehicle_held_for_replan";
      break;
    case AvoidanceState::kReplan:
      if (event.plan_attempted) {
        state_ = event.plan_valid ? AvoidanceState::kTrajectoryVerify :
          AvoidanceState::kHoverRequired;
        planner_failure_latched_ = !event.plan_valid;
        reason_ = event.plan_valid ? "candidate_generated" : "planner_failed";
      }
      break;
    case AvoidanceState::kTrajectoryVerify:
      state_ = event.plan_valid ? AvoidanceState::kResume : AvoidanceState::kHoverRequired;
      planner_failure_latched_ = !event.plan_valid;
      reason_ = event.plan_valid ? "trajectory_raw_verified" : "trajectory_verification_failed";
      break;
    case AvoidanceState::kResume:
      if (event.path_blocked) {
        state_ = AvoidanceState::kPathBlocked;
        reason_ = "active_trajectory_reblocked";
      } else if (event.goal_reached) {
        state_ = AvoidanceState::kNavigating;
        reason_ = "local_detour_complete";
      } else {
        reason_ = "verified_detour_active";
      }
      break;
    case AvoidanceState::kHoverRequired:
      if (planner_failure_latched_ && event.path_blocked) {
        state_ = AvoidanceState::kHoverRequired;
        reason_ = "planner_failure_latched_path_still_blocked";
      } else {
        state_ = event.path_blocked ? AvoidanceState::kBrakeHold :
          AvoidanceState::kNavigating;
        if (!event.path_blocked) {planner_failure_latched_ = false;}
        reason_ = event.path_blocked ? "healthy_inputs_recovered_replan" :
          "healthy_inputs_recovered_direct";
      }
      break;
  }
  return state_;
}

void LocalAvoidanceStateMachine::reset()
{
  state_ = AvoidanceState::kNavigating;
  reason_ = "mission_direct";
  planner_failure_latched_ = false;
}

const char * to_string(const AvoidanceState state)
{
  switch (state) {
    case AvoidanceState::kNavigating: return "NAVIGATING";
    case AvoidanceState::kPathBlocked: return "PATH_BLOCKED";
    case AvoidanceState::kBrakeHold: return "BRAKE_HOLD";
    case AvoidanceState::kReplan: return "REPLAN";
    case AvoidanceState::kTrajectoryVerify: return "TRAJECTORY_VERIFY";
    case AvoidanceState::kResume: return "RESUME";
    case AvoidanceState::kHoverRequired: return "HOVER_REQUIRED";
  }
  return "UNKNOWN";
}

}  // namespace uf_safety_supervisor
