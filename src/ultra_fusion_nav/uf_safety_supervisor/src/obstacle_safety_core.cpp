#include "uf_safety_supervisor/obstacle_safety_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace uf_safety_supervisor
{

ObstacleSafetyCore::ObstacleSafetyCore(ObstacleSafetyConfig config) : config_(config)
{
  if (!(config_.maximum_deceleration_mps2 > 0.0) || !(config_.body_front_m >= 0.0) ||
    !(config_.body_half_width_m > 0.0) || !(config_.body_half_height_m > 0.0))
  {
    throw std::invalid_argument("invalid obstacle safety geometry or braking configuration");
  }
}

ObstacleSafetyResult ObstacleSafetyCore::evaluate(const ObstacleSafetyInput & input) const
{
  ObstacleSafetyResult result;
  const auto fail = [&](const std::string & reason) {
      result.state = ObstacleState::kHover;
      result.fail_closed = true;
      result.reason = reason;
      return result;
    };
  if (!input.raw_sensor_healthy) {return fail("raw_lidar_unhealthy_or_stale");}
  if (!input.timestamps_valid) {return fail("raw_lidar_timestamp_invalid");}
  if (!input.motion_finite || !input.body_velocity.allFinite() ||
    !input.desired_direction_body.allFinite())
  {
    return fail("motion_or_direction_nonfinite");
  }
  if (!std::isfinite(input.sensor_age_s) || input.sensor_age_s < 0.0) {
    return fail("raw_lidar_age_invalid");
  }

  Eigen::Vector3d direction = input.desired_direction_body;
  const double speed = input.body_velocity.norm();
  if (direction.norm() < 1.0e-6) {
    direction = speed > 1.0e-6 ? input.body_velocity : Eigen::Vector3d::UnitX();
  }
  direction.normalize();
  const double closing_speed = std::max(0.0, input.body_velocity.dot(direction));
  result.stopping_distance_m = config_.safety_margin_m +
    config_.reaction_time_s * closing_speed +
    closing_speed * closing_speed / (2.0 * config_.maximum_deceleration_mps2);
  result.nearest_clearance_m = std::numeric_limits<double>::infinity();
  result.path_clearance_m = std::numeric_limits<double>::infinity();
  result.time_to_collision_s = std::numeric_limits<double>::infinity();

  double nearest_range_squared = std::numeric_limits<double>::infinity();
  const double minimum_range_squared = config_.minimum_valid_range_m * config_.minimum_valid_range_m;
  const double maximum_range_squared = config_.maximum_valid_range_m * config_.maximum_valid_range_m;
  const double lateral_limit = config_.body_half_width_m + config_.lateral_margin_m;
  const double lateral_limit_squared = lateral_limit * lateral_limit;
  for (const auto & point : input.body_points) {
    if (!point.allFinite()) {return fail("raw_lidar_point_nonfinite");}
    const double range_squared = point.squaredNorm();
    if (range_squared < minimum_range_squared || range_squared > maximum_range_squared) {
      continue;
    }
    nearest_range_squared = std::min(nearest_range_squared, range_squared);
    const double longitudinal = point.dot(direction);
    if (longitudinal <= 0.0) {continue;}
    const Eigen::Vector3d lateral_vector = point - longitudinal * direction;
    const double vertical = std::abs(point.z());
    const double lateral_squared =
      lateral_vector.x() * lateral_vector.x() + lateral_vector.y() * lateral_vector.y();
    if (lateral_squared > lateral_limit_squared ||
      vertical > config_.body_half_height_m + config_.vertical_margin_m)
    {
      continue;
    }
    result.path_clearance_m = std::min(
      result.path_clearance_m, std::max(0.0, longitudinal - config_.body_front_m));
  }
  if (std::isfinite(nearest_range_squared)) {
    result.nearest_clearance_m = std::max(
      0.0, std::sqrt(nearest_range_squared) - config_.body_front_m);
  }

  if (std::isfinite(result.path_clearance_m) && closing_speed > 1.0e-3) {
    result.time_to_collision_s = result.path_clearance_m / closing_speed;
  }
  if (std::isfinite(result.path_clearance_m) &&
    (result.path_clearance_m <= result.stopping_distance_m ||
    result.time_to_collision_s <= config_.brake_ttc_s))
  {
    result.state = ObstacleState::kBrake;
    result.fail_closed = false;
    result.reason = "braking_envelope_intrusion";
  } else if (std::isfinite(result.path_clearance_m) &&
    (result.path_clearance_m <= result.stopping_distance_m + config_.caution_margin_m ||
    result.time_to_collision_s <= config_.caution_ttc_s))
  {
    result.state = ObstacleState::kCaution;
    result.fail_closed = false;
    result.reason = "caution_envelope_intrusion";
  } else {
    result.state = ObstacleState::kClear;
    result.fail_closed = false;
    result.reason = input.localization_valid ? "raw_corridor_clear" : "raw_body_frame_clear";
  }
  return result;
}

const char * to_string(const ObstacleState state)
{
  switch (state) {
    case ObstacleState::kClear: return "CLEAR";
    case ObstacleState::kCaution: return "CAUTION";
    case ObstacleState::kBrake: return "BRAKE";
    case ObstacleState::kHover: return "HOVER_REQUIRED";
  }
  return "UNKNOWN";
}

}  // namespace uf_safety_supervisor
