#include <cstdint>
#include <array>

#include <gtest/gtest.h>

#include "mid360_sim_bridge_cpp/conversion.hpp"

using mid360_sim_bridge_cpp::epoch_aligned_stamp_ns;
using mid360_sim_bridge_cpp::checked_nonnegative_stamp_ns;
using mid360_sim_bridge_cpp::line_for_output_index;
using mid360_sim_bridge_cpp::monotonic_positive_stamp_ns;
using mid360_sim_bridge_cpp::packet_begin_stamp_ns;
using mid360_sim_bridge_cpp::point_in_body_exclusion_box;
using mid360_sim_bridge_cpp::point_offset_time_ns;
using mid360_sim_bridge_cpp::reflectivity_from_intensity;
using mid360_sim_bridge_cpp::relative_time_ns;
using mid360_sim_bridge_cpp::should_accept_model_pose;

TEST(Conversion, DynamicModelPosePermanentlySupersedesFallbackInventory)
{
  EXPECT_TRUE(should_accept_model_pose(false, false));
  EXPECT_TRUE(should_accept_model_pose(false, true));
  EXPECT_TRUE(should_accept_model_pose(true, true));
  EXPECT_FALSE(should_accept_model_pose(true, false));
}

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

TEST(Conversion, RejectsMalformedOrNegativeClockStamps)
{
  EXPECT_EQ(checked_nonnegative_stamp_ns(-1, 0), 0);
  EXPECT_EQ(checked_nonnegative_stamp_ns(1, -1), 0);
  EXPECT_EQ(checked_nonnegative_stamp_ns(1, 1000000000LL), 0);
  EXPECT_EQ(checked_nonnegative_stamp_ns(2, 500000000), 2500000000LL);
}

TEST(Conversion, SanitizesClockResetToStrictlyIncreasingPositiveStamp)
{
  EXPECT_EQ(monotonic_positive_stamp_ns(-10, 100, 50), 101);
  EXPECT_EQ(monotonic_positive_stamp_ns(90, 100, 50), 101);
  EXPECT_EQ(monotonic_positive_stamp_ns(120, 100, 50), 120);
  EXPECT_EQ(monotonic_positive_stamp_ns(0, 0, 0), 1);
}

TEST(Conversion, RejectsInvalidEpochAlignment)
{
  EXPECT_EQ(epoch_aligned_stamp_ns(-1, 1, 100), 0);
  EXPECT_EQ(epoch_aligned_stamp_ns(2, 1, 100), 101);
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

TEST(Conversion, ClampsPacketBeginWhenAcquisitionIsInvalidOrTooEarly)
{
  EXPECT_EQ(packet_begin_stamp_ns(-1, 100000000ULL, false), 1);
  EXPECT_EQ(packet_begin_stamp_ns(50000000LL, 100000000ULL, false), 1);
  EXPECT_EQ(packet_begin_stamp_ns(-1, 100000000ULL, true), 1);
}

TEST(Conversion, ClampsReflectivity)
{
  EXPECT_EQ(reflectivity_from_intensity(-10.0), 0U);
  EXPECT_EQ(reflectivity_from_intensity(127.6), 128U);
  EXPECT_EQ(reflectivity_from_intensity(500.0), 255U);
}

TEST(Conversion, BodyExclusionUsesLidarToBodyExtrinsic)
{
  constexpr double c = 0.984807753012208;
  constexpr double s = 0.173648177666930;
  const std::array<double, 6> bounds{-0.25, 0.25, -0.25, 0.25, -0.05, 0.05};
  const std::array<double, 9> rotation{c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c};
  const std::array<double, 3> translation{0.0, 0.0, 0.0};

  EXPECT_TRUE(point_in_body_exclusion_box(
    0.20, 0.10, 0.00, bounds, rotation, translation));
  EXPECT_FALSE(point_in_body_exclusion_box(
    0.80, 0.10, 0.20, bounds, rotation, translation));
  EXPECT_FALSE(point_in_body_exclusion_box(
    0.20, 0.60, 0.00, bounds, rotation, translation));
}
