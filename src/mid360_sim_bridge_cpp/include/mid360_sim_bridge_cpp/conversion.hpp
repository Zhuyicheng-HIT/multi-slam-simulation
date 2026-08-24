#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace mid360_sim_bridge_cpp
{

constexpr std::uint8_t kDefaultTag = 0U;
constexpr std::size_t kDefaultLineCount = 4U;

// Gazebo publishes both a dynamic-pose stream and a full pose inventory. The
// full inventory can contain the model's initial pose and may arrive after a
// newer dynamic update. Once dynamic truth is available, never let that
// fallback inventory overwrite it.
inline bool should_accept_model_pose(
  const bool dynamic_pose_seen, const bool candidate_is_dynamic)
{
  return candidate_is_dynamic || !dynamic_pose_seen;
}

// Convert a seconds/nanoseconds pair without allowing malformed or negative
// input to reach rclcpp::Time. Gazebo and ROS messages can contain a transient
// reset value while a simulation is restarting.
inline std::int64_t checked_nonnegative_stamp_ns(
  const std::int64_t seconds, const std::int64_t nanoseconds)
{
  if (seconds < 0 || nanoseconds < 0 || nanoseconds >= 1000000000LL) {
    return 0;
  }
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
  const auto max_value = std::numeric_limits<std::int64_t>::max();
  if (seconds > max_value / kNanosecondsPerSecond) {
    return 0;
  }
  const auto whole_seconds = seconds * kNanosecondsPerSecond;
  if (nanoseconds > max_value - whole_seconds) {
    return 0;
  }
  const auto value = whole_seconds + nanoseconds;
  return value > 0 ? value : 0;
}

// Return a strictly increasing, positive timestamp for an output stream.
// Invalid input falls back to the current clock value; if that clock is also
// unavailable (e.g. before /clock starts), a minimal positive epoch is used.
inline std::int64_t monotonic_positive_stamp_ns(
  const std::int64_t candidate_ns,
  const std::int64_t previous_ns,
  const std::int64_t fallback_ns)
{
  std::int64_t candidate = candidate_ns > 0 ? candidate_ns : fallback_ns;
  if (candidate <= 0) {
    candidate = 1;
  }
  if (candidate <= previous_ns) {
    if (previous_ns == std::numeric_limits<std::int64_t>::max()) {
      return previous_ns;
    }
    candidate = previous_ns + 1;
  }
  return candidate;
}

inline std::int64_t epoch_aligned_stamp_ns(
  const std::int64_t source_stamp_ns,
  const std::int64_t source_origin_ns,
  const std::int64_t epoch_origin_ns)
{
  if (source_stamp_ns <= 0 || source_origin_ns <= 0 || epoch_origin_ns <= 0) {
    return 0;
  }
  if (epoch_origin_ns >= source_origin_ns) {
    const auto delta = epoch_origin_ns - source_origin_ns;
    if (delta > std::numeric_limits<std::int64_t>::max() - source_stamp_ns) {
      return 0;
    }
    return delta + source_stamp_ns;
  }
  const auto deficit = source_origin_ns - epoch_origin_ns;
  return source_stamp_ns > deficit ? source_stamp_ns - deficit : 0;
}

inline std::uint8_t line_for_output_index(
  const std::size_t index, const std::size_t line_count)
{
  const auto safe_count = std::max<std::size_t>(1U, line_count);
  return static_cast<std::uint8_t>(index % safe_count);
}

inline std::uint32_t relative_time_ns(
  const std::size_t source_index,
  const std::size_t source_count,
  const std::uint64_t scan_period_ns)
{
  const auto denominator = std::max<std::size_t>(1U, source_count - 1U);
  const auto clamped_index = std::min(source_index, denominator);
  const long double ratio =
    static_cast<long double>(clamped_index) / static_cast<long double>(denominator);
  const long double offset = ratio * static_cast<long double>(scan_period_ns);
  return static_cast<std::uint32_t>(std::clamp<long double>(
    std::llround(offset), 0.0L,
    static_cast<long double>(UINT32_MAX)));
}

inline std::uint32_t point_offset_time_ns(
  const std::size_t source_index,
  const std::size_t source_count,
  const std::uint64_t scan_period_ns,
  const bool synthetic_scan_timing)
{
  return synthetic_scan_timing ?
         relative_time_ns(source_index, source_count, scan_period_ns) :
         static_cast<std::uint32_t>(std::min<std::uint64_t>(scan_period_ns, UINT32_MAX));
}

inline std::int64_t packet_begin_stamp_ns(
  const std::int64_t acquisition_stamp_ns,
  const std::uint64_t scan_period_ns,
  const bool synthetic_scan_timing)
{
  if (synthetic_scan_timing) {
    return acquisition_stamp_ns > 0 ? acquisition_stamp_ns : 1;
  }
  const auto period = static_cast<std::int64_t>(std::min<std::uint64_t>(
    scan_period_ns, static_cast<std::uint64_t>(INT64_MAX)));
  if (acquisition_stamp_ns <= 0) {
    return 1;
  }
  if (acquisition_stamp_ns <= period) {
    return 1;
  }
  return acquisition_stamp_ns - period;
}

inline std::uint8_t reflectivity_from_intensity(const double intensity)
{
  if (!std::isfinite(intensity)) {
    return 0U;
  }
  return static_cast<std::uint8_t>(std::clamp(std::llround(intensity), 0LL, 255LL));
}

inline bool point_in_body_exclusion_box(
  const double lidar_x,
  const double lidar_y,
  const double lidar_z,
  const std::array<double, 6> & body_bounds,
  const std::array<double, 9> & lidar_to_body_rotation,
  const std::array<double, 3> & lidar_to_body_translation)
{
  const double body_x =
    lidar_to_body_rotation[0] * lidar_x +
    lidar_to_body_rotation[1] * lidar_y +
    lidar_to_body_rotation[2] * lidar_z +
    lidar_to_body_translation[0];
  const double body_y =
    lidar_to_body_rotation[3] * lidar_x +
    lidar_to_body_rotation[4] * lidar_y +
    lidar_to_body_rotation[5] * lidar_z +
    lidar_to_body_translation[1];
  const double body_z =
    lidar_to_body_rotation[6] * lidar_x +
    lidar_to_body_rotation[7] * lidar_y +
    lidar_to_body_rotation[8] * lidar_z +
    lidar_to_body_translation[2];
  return
    body_bounds[0] <= body_x && body_x <= body_bounds[1] &&
    body_bounds[2] <= body_y && body_y <= body_bounds[3] &&
    body_bounds[4] <= body_z && body_z <= body_bounds[5];
}

}  // namespace mid360_sim_bridge_cpp
