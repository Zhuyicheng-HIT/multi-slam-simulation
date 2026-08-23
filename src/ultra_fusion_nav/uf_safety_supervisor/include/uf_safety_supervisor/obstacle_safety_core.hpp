#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace uf_safety_supervisor
{

enum class ObstacleState : std::uint8_t {kClear = 0, kCaution = 1, kBrake = 2, kHover = 3};

struct ObstacleSafetyConfig
{
  double body_front_m{0.32};
  double body_half_width_m{0.32};
  double body_half_height_m{0.18};
  double lateral_margin_m{0.28};
  double vertical_margin_m{0.22};
  double safety_margin_m{0.35};
  double caution_margin_m{0.75};
  double reaction_time_s{0.25};
  double maximum_deceleration_mps2{2.0};
  double brake_ttc_s{1.25};
  double caution_ttc_s{2.5};
  double minimum_valid_range_m{0.12};
  double maximum_valid_range_m{40.0};
};

struct ObstacleSafetyInput
{
  std::vector<Eigen::Vector3d> body_points;
  Eigen::Vector3d body_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d desired_direction_body{Eigen::Vector3d::UnitX()};
  bool raw_sensor_healthy{true};
  bool timestamps_valid{true};
  bool motion_finite{true};
  bool localization_valid{true};
  double sensor_age_s{0.0};
};

struct ObstacleSafetyResult
{
  ObstacleState state{ObstacleState::kHover};
  bool fail_closed{true};
  double nearest_clearance_m{0.0};
  double path_clearance_m{0.0};
  double time_to_collision_s{0.0};
  double stopping_distance_m{0.0};
  std::string reason{"uninitialized"};
};

class ObstacleSafetyCore
{
public:
  explicit ObstacleSafetyCore(ObstacleSafetyConfig config = {});
  ObstacleSafetyResult evaluate(const ObstacleSafetyInput & input) const;

private:
  ObstacleSafetyConfig config_;
};

const char * to_string(ObstacleState state);

}  // namespace uf_safety_supervisor
