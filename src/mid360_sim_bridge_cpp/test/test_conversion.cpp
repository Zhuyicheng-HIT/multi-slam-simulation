#include <cstdint>

#include <gtest/gtest.h>

#include "mid360_sim_bridge_cpp/conversion.hpp"

using mid360_sim_bridge_cpp::line_for_output_index;
using mid360_sim_bridge_cpp::reflectivity_from_intensity;
using mid360_sim_bridge_cpp::relative_time_ns;

TEST(Conversion, AssignsFourLivoxLines)
{
  EXPECT_EQ(line_for_output_index(0U, 4U), 0U);
  EXPECT_EQ(line_for_output_index(3U, 4U), 3U);
  EXPECT_EQ(line_for_output_index(4U, 4U), 0U);
}

TEST(Conversion, SpansConfiguredScanPeriod)
{
  constexpr std::uint64_t period_ns = 100000000ULL;
  EXPECT_EQ(relative_time_ns(0U, 5U, period_ns), 0U);
  EXPECT_EQ(relative_time_ns(2U, 5U, period_ns), 50000000U);
  EXPECT_EQ(relative_time_ns(4U, 5U, period_ns), 100000000U);
}

TEST(Conversion, ClampsReflectivity)
{
  EXPECT_EQ(reflectivity_from_intensity(-10.0), 0U);
  EXPECT_EQ(reflectivity_from_intensity(127.6), 128U);
  EXPECT_EQ(reflectivity_from_intensity(500.0), 255U);
}
