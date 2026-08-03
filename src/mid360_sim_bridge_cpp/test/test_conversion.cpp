#include <cstdint>

#include <gtest/gtest.h>

#include "mid360_sim_bridge_cpp/conversion.hpp"

using mid360_sim_bridge_cpp::epoch_aligned_stamp_ns;
using mid360_sim_bridge_cpp::line_for_output_index;
using mid360_sim_bridge_cpp::packet_begin_stamp_ns;
using mid360_sim_bridge_cpp::point_offset_time_ns;
using mid360_sim_bridge_cpp::reflectivity_from_intensity;
using mid360_sim_bridge_cpp::relative_time_ns;

TEST(Conversion, AssignsFourLivoxLines)
{
  EXPECT_EQ(line_for_output_index(0U, 4U), 0U);
  EXPECT_EQ(line_for_output_index(3U, 4U), 3U);
  EXPECT_EQ(line_for_output_index(4U, 4U), 0U);
}

TEST(Conversion, PreservesSourceClockRateAfterEpochAlignment)
{
  constexpr std::int64_t source_origin_ns = 2500000000LL;
  constexpr std::int64_t epoch_origin_ns = 1785551000000000000LL;
  EXPECT_EQ(
    epoch_aligned_stamp_ns(2600000000LL, source_origin_ns, epoch_origin_ns),
    epoch_origin_ns + 100000000LL);
}

TEST(Conversion, SpansConfiguredScanPeriod)
{
  constexpr std::uint64_t period_ns = 100000000ULL;
  EXPECT_EQ(relative_time_ns(0U, 5U, period_ns), 0U);
  EXPECT_EQ(relative_time_ns(2U, 5U, period_ns), 50000000U);
  EXPECT_EQ(relative_time_ns(4U, 5U, period_ns), 100000000U);
}

TEST(Conversion, SynchronousSnapshotPlacesEveryPointAtPacketEnd)
{
  constexpr std::uint64_t period_ns = 100000000ULL;
  EXPECT_EQ(point_offset_time_ns(0U, 5U, period_ns, false), period_ns);
  EXPECT_EQ(point_offset_time_ns(2U, 5U, period_ns, false), period_ns);
  EXPECT_EQ(point_offset_time_ns(4U, 5U, period_ns, false), period_ns);
  EXPECT_EQ(point_offset_time_ns(4U, 5U, period_ns, true), 100000000U);
}

TEST(Conversion, SynchronousSnapshotPacketEndsAtAcquisitionStamp)
{
  constexpr std::int64_t acquisition_ns = 1785551000100000000LL;
  constexpr std::uint64_t period_ns = 100000000ULL;
  EXPECT_EQ(
    packet_begin_stamp_ns(acquisition_ns, period_ns, false),
    acquisition_ns - static_cast<std::int64_t>(period_ns));
  EXPECT_EQ(
    packet_begin_stamp_ns(acquisition_ns, period_ns, true), acquisition_ns);
}

TEST(Conversion, ClampsReflectivity)
{
  EXPECT_EQ(reflectivity_from_intensity(-10.0), 0U);
  EXPECT_EQ(reflectivity_from_intensity(127.6), 128U);
  EXPECT_EQ(reflectivity_from_intensity(500.0), 255U);
}
