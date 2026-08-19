#include "uf_dynamic_observer/causal_imu_deskew.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace uf_dynamic_observer
{

CausalImuDeskew::CausalImuDeskew(CausalDeskewConfig config)
: config_(std::move(config))
{
  if (!(config_.max_imu_gap_s > 0.0) || !(config_.max_prediction_horizon_s > 0.0) ||
    !config_.gravity_world.allFinite() || !config_.accel_bias.allFinite() ||
    !config_.gyro_bias.allFinite())
  {
    throw std::invalid_argument("invalid causal deskew configuration");
  }
}

CausalDeskewResult CausalImuDeskew::propagate(
  const CausalPose & anchor, const std::vector<CausalImuSample> & imu,
  const std::vector<std::int64_t> & query_stamps_ns) const
{
  CausalDeskewResult output;
  output.anchor_stamp_ns = anchor.stamp_ns;
  if (query_stamps_ns.empty()) {
    output.valid = true;
    output.reason = "empty_query";
    return output;
  }
  if (!anchor.position.allFinite() || !anchor.velocity.allFinite() ||
    anchor.orientation.norm() < 1.0e-9)
  {
    output.reason = "invalid_anchor";
    return output;
  }
  if (!std::is_sorted(query_stamps_ns.begin(), query_stamps_ns.end())) {
    output.reason = "query_not_sorted";
    return output;
  }
  if (anchor.stamp_ns > query_stamps_ns.front()) {
    output.reason = "future_pose_anchor";
    return output;
  }
  const double horizon_s = static_cast<double>(query_stamps_ns.back() - anchor.stamp_ns) * 1.0e-9;
  if (horizon_s < 0.0 || horizon_s > config_.max_prediction_horizon_s) {
    output.reason = "prediction_horizon_exceeded";
    return output;
  }
  if (imu.empty() || imu.front().stamp_ns > anchor.stamp_ns)
  {
    output.reason = "imu_coverage_missing";
    return output;
  }
  if (imu.back().stamp_ns > query_stamps_ns.back()) {
    output.reason = "future_imu_sample";
    return output;
  }
  if (!imu.front().linear_acceleration.allFinite() ||
    !imu.front().angular_velocity.allFinite())
  {
    output.reason = "imu_invalid_or_unsorted";
    return output;
  }
  for (std::size_t index = 1U; index < imu.size(); ++index) {
    if (imu[index].stamp_ns <= imu[index - 1U].stamp_ns ||
      !imu[index].linear_acceleration.allFinite() ||
      !imu[index].angular_velocity.allFinite())
    {
      output.reason = "imu_invalid_or_unsorted";
      return output;
    }
  }
  // A scan often closes between two IMU ticks. Holding the last measured IMU
  // sample is causal and matches the piecewise-constant input used during
  // FAST-LIO propagation, but only inside the already configured gap bound.
  const double terminal_gap_s = static_cast<double>(
    query_stamps_ns.back() - imu.back().stamp_ns) * 1.0e-9;
  output.max_observed_imu_gap_s = terminal_gap_s;
  if (terminal_gap_s > config_.max_imu_gap_s) {
    output.reason = "terminal_imu_gap_exceeded";
    return output;
  }

  auto upper = std::upper_bound(
    imu.begin(), imu.end(), anchor.stamp_ns,
    [](std::int64_t stamp, const CausalImuSample & sample) {return stamp < sample.stamp_ns;});
  if (upper == imu.begin()) {
    output.reason = "imu_before_anchor_missing";
    return output;
  }
  std::size_t imu_index = static_cast<std::size_t>(std::distance(imu.begin(), upper) - 1);
  CausalPose state = anchor;
  state.orientation.normalize();
  std::int64_t current_ns = anchor.stamp_ns;
  output.poses.reserve(query_stamps_ns.size());

  for (const auto query_ns : query_stamps_ns) {
    while (current_ns < query_ns) {
      const std::int64_t next_imu_ns = imu_index + 1U < imu.size() ?
        imu[imu_index + 1U].stamp_ns : query_ns;
      const std::int64_t step_end_ns = std::min(query_ns, next_imu_ns);
      const double dt = static_cast<double>(step_end_ns - current_ns) * 1.0e-9;
      if (!(dt >= 0.0)) {
        output.reason = "negative_integration_step";
        output.poses.clear();
        return output;
      }
      const double sample_gap_s = imu_index + 1U < imu.size() ?
        static_cast<double>(imu[imu_index + 1U].stamp_ns - imu[imu_index].stamp_ns) * 1.0e-9 :
        dt;
      output.max_observed_imu_gap_s = std::max(output.max_observed_imu_gap_s, sample_gap_s);
      if (sample_gap_s > config_.max_imu_gap_s) {
        output.reason = "imu_gap_exceeded";
        output.poses.clear();
        return output;
      }

      const Eigen::Vector3d omega = imu[imu_index].angular_velocity - config_.gyro_bias;
      const Eigen::Vector3d body_accel = imu[imu_index].linear_acceleration - config_.accel_bias;
      const Eigen::Vector3d world_accel = state.orientation * body_accel + config_.gravity_world;
      state.position += state.velocity * dt + 0.5 * world_accel * dt * dt;
      state.velocity += world_accel * dt;
      const double angle = omega.norm() * dt;
      if (angle > 1.0e-15) {
        state.orientation = (state.orientation *
          Eigen::Quaterniond(Eigen::AngleAxisd(angle, omega.normalized()))).normalized();
      }
      current_ns = step_end_ns;
      if (current_ns == next_imu_ns && imu_index + 1U < imu.size()) {
        ++imu_index;
      }
    }
    state.stamp_ns = query_ns;
    output.poses.push_back(state);
    output.latest_imu_consumed_ns = imu[imu_index].stamp_ns;
  }
  output.valid = true;
  output.reason = "ok";
  return output;
}

}  // namespace uf_dynamic_observer
