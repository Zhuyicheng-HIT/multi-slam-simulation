#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace mid360_sim_bridge_cpp
{

constexpr std::uint8_t kDefaultTag = 0U;
constexpr std::size_t kDefaultLineCount = 4U;

inline std::int64_t epoch_aligned_stamp_ns(
  const std::int64_t source_stamp_ns,
  const std::int64_t source_origin_ns,
  const std::int64_t epoch_origin_ns)
{
  return epoch_origin_ns + (source_stamp_ns - source_origin_ns);
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
    return acquisition_stamp_ns;
  }
  const auto period = static_cast<std::int64_t>(std::min<std::uint64_t>(
    scan_period_ns, static_cast<std::uint64_t>(INT64_MAX)));
  return std::max<std::int64_t>(1, acquisition_stamp_ns - period);
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
