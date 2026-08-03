#pragma once

#include <algorithm>
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

}  // namespace mid360_sim_bridge_cpp
