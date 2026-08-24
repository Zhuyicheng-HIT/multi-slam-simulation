#pragma once

#include "uf_safety_supervisor/obstacle_safety_core.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <string>

namespace uf_safety_supervisor
{

enum class CommandAction : std::uint8_t {kRelease = 0, kForward = 1, kHold = 2, kLand = 3, kReturn = 4};

struct PoseIntent
{
  bool received{false};
  bool finite{false};
  double stamp_s{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

struct CommandArbiterConfig
{
  double intent_timeout_s{0.5};
  double current_pose_timeout_s{0.25};
  double obstacle_timeout_s{0.25};
  double maximum_setpoint_jump_m{2.5};
  double caution_step_m{0.30};
};

struct CommandArbiterInput
{
  double now_s{0.0};
  bool manual_override{false};
  bool fcu_failsafe{false};
  bool land_requested{false};
  bool return_requested{false};
  bool localization_hold{false};
  bool active_relocalization_hold{false};
  bool active_relocalization_authorized{false};
  ObstacleState obstacle_state{ObstacleState::kHover};
  bool obstacle_healthy{false};
  double obstacle_stamp_s{0.0};
  PoseIntent current_pose;
  PoseIntent relocalization;
  PoseIntent planner;
  PoseIntent mission;
};

struct CommandDecision
{
  CommandAction action{CommandAction::kHold};
  std::string owner{"fail_closed"};
  std::string reason{"uninitialized"};
  bool fail_closed{true};
  bool publish_setpoint{false};
  PoseIntent selected;
};

class CommandArbiterCore
{
public:
  explicit CommandArbiterCore(CommandArbiterConfig config = {});
  CommandDecision evaluate(const CommandArbiterInput & input) const;

private:
  bool fresh(const PoseIntent & intent, double now_s) const;
  CommandDecision hold(const CommandArbiterInput & input, const std::string & owner,
    const std::string & reason, bool fail_closed) const;
  CommandDecision forward(const CommandArbiterInput & input, const PoseIntent & intent,
    const std::string & owner) const;
  CommandArbiterConfig config_;
};

const char * to_string(CommandAction action);

}  // namespace uf_safety_supervisor
