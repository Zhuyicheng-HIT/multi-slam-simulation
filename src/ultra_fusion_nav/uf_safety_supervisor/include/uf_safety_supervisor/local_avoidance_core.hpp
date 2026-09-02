#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace uf_safety_supervisor
{

enum class AvoidanceState : std::uint8_t
{
  kNavigating = 0,
  kPathBlocked = 1,
  kBrakeHold = 2,
  kReplan = 3,
  kTrajectoryVerify = 4,
  kResume = 5,
  kHoverRequired = 6
};

struct LocalPlannerConfig
{
  double resolution_m{0.25};
  double obstacle_inflation_m{0.80};
  double verification_clearance_m{0.65};
  double planning_horizon_m{7.0};
  double side_padding_m{3.5};
  double goal_tolerance_m{0.35};
  double waypoint_spacing_m{0.75};
  std::size_t maximum_expansions{30000U};
};

struct LocalPlanResult
{
  bool success{false};
  bool verified{false};
  std::string reason{"uninitialized"};
  std::vector<Eigen::Vector3d> path;
  double minimum_clearance_m{0.0};
  std::size_t expanded_nodes{0U};
};

class ConservativeLocalPlanner
{
public:
  explicit ConservativeLocalPlanner(LocalPlannerConfig config = {});
  LocalPlanResult plan(
    const Eigen::Vector3d & start, const Eigen::Vector3d & goal,
    const std::vector<Eigen::Vector3d> & obstacles) const;
  bool verify(
    const std::vector<Eigen::Vector3d> & path,
    const std::vector<Eigen::Vector3d> & obstacles,
    double * minimum_clearance_m = nullptr) const;
  bool segment_clear(
    const Eigen::Vector3d & start, const Eigen::Vector3d & goal,
    const std::vector<Eigen::Vector3d> & obstacles,
    double * minimum_clearance_m = nullptr) const;

private:
  LocalPlannerConfig config_;
};

struct AvoidanceEvent
{
  bool raw_fresh{true};
  bool localization_healthy{true};
  bool mission_fresh{true};
  bool path_blocked{false};
  bool goal_reached{false};
  bool plan_attempted{false};
  bool plan_valid{false};
};

class LocalAvoidanceStateMachine
{
public:
  AvoidanceState update(const AvoidanceEvent & event);
  AvoidanceState state() const {return state_;}
  const std::string & reason() const {return reason_;}
  void reset();

private:
  AvoidanceState state_{AvoidanceState::kNavigating};
  std::string reason_{"mission_direct"};
  bool planner_failure_latched_{false};
};

const char * to_string(AvoidanceState state);

}  // namespace uf_safety_supervisor
