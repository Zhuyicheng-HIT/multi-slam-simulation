#include "uf_dynamic_observer/conservative_free_space.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace uf_dynamic_observer
{
namespace
{

FilterConfig test_config()
{
  FilterConfig config;
  config.voxel_size_m = 0.5;
  config.min_range_m = 0.1;
  config.max_range_m = 20.0;
  config.free_confirmations = 2U;
  config.static_confirmations = 2U;
  config.occupied_recovery = 4U;
  config.endpoint_guard_voxels = 0;
  config.dynamic_growth_voxels = 0;
  config.ray_stride = 1;
  return config;
}

TEST(ConservativeFreeSpace, NewObservationStartsUnknownThenBecomesStatic)
{
  ConservativeFreeSpaceObserver filter(test_config());
  const Point origin{0.0, 0.0, 0.0, 0.0F};
  const std::vector<Point> scan{{4.0, 0.0, 0.0, 1.0F}};
  EXPECT_EQ(filter.process(scan, origin).points.front().label, PointLabel::kUnknown);
  EXPECT_EQ(filter.process(scan, origin).points.front().label, PointLabel::kUnknown);
  EXPECT_EQ(filter.process(scan, origin).points.front().label, PointLabel::kStatic);
}

TEST(ConservativeFreeSpace, PointEnteringConfirmedFreeSpaceIsDynamic)
{
  ConservativeFreeSpaceObserver filter(test_config());
  const Point origin{0.0, 0.0, 0.0, 0.0F};
  const std::vector<Point> background{{8.0, 0.0, 0.0, 1.0F}};
  filter.process(background, origin);
  filter.process(background, origin);
  const auto result = filter.process({{4.0, 0.0, 0.0, 1.0F}}, origin);
  ASSERT_EQ(result.points.size(), 1U);
  EXPECT_EQ(result.points.front().label, PointLabel::kDynamic);
  EXPECT_FLOAT_EQ(result.points.front().dynamic_score, 1.0F);
}

TEST(ConservativeFreeSpace, RepeatedOccupancyRecoversFromPoseOrMapError)
{
  auto config = test_config();
  config.occupied_recovery = 3U;
  ConservativeFreeSpaceObserver filter(config);
  const Point origin{0.0, 0.0, 0.0, 0.0F};
  const std::vector<Point> background{{8.0, 0.0, 0.0, 1.0F}};
  filter.process(background, origin);
  filter.process(background, origin);
  const std::vector<Point> obstacle{{4.0, 0.0, 0.0, 1.0F}};
  EXPECT_EQ(filter.process(obstacle, origin).points.front().label, PointLabel::kDynamic);
  EXPECT_EQ(filter.process(obstacle, origin).points.front().label, PointLabel::kDynamic);
  EXPECT_EQ(filter.process(obstacle, origin).points.front().label, PointLabel::kDynamic);
  EXPECT_NE(filter.process(obstacle, origin).points.front().label, PointLabel::kDynamic);
}

TEST(ConservativeFreeSpace, InvalidAndOutOfRangePointsAreNotPublished)
{
  ConservativeFreeSpaceObserver filter(test_config());
  const Point origin{0.0, 0.0, 0.0, 0.0F};
  const auto result = filter.process(
    {{100.0, 0.0, 0.0, 1.0F}, {0.01, 0.0, 0.0, 1.0F}}, origin);
  EXPECT_TRUE(result.points.empty());
  EXPECT_EQ(result.stats.valid_points, 0U);
}

TEST(TemporalBaseline, PreservesExistingWarmWindowContract)
{
  TemporalVoxelBaseline filter(0.5, 2U, 2U, 0);
  const Point origin{0.0, 0.0, 0.0, 0.0F};
  const std::vector<Point> repeated{{3.0, 0.0, 0.0, 1.0F}};
  EXPECT_EQ(filter.process(repeated, origin).points.front().label, PointLabel::kUnknown);
  EXPECT_EQ(filter.process(repeated, origin).points.front().label, PointLabel::kUnknown);
  EXPECT_EQ(filter.process(repeated, origin).points.front().label, PointLabel::kStatic);
  const auto novel = filter.process({{3.0, 2.0, 0.0, 1.0F}}, origin);
  EXPECT_EQ(novel.points.front().label, PointLabel::kDynamic);
}

}  // namespace
}  // namespace uf_dynamic_observer
