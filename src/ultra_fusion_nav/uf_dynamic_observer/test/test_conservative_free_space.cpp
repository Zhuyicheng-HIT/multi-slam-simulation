#include "uf_dynamic_observer/causal_imu_deskew.hpp"
#include "uf_dynamic_observer/clean_scan_admission.hpp"
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

VisibilityFilterConfig visibility_test_config()
{
  VisibilityFilterConfig config;
  config.voxel_size_m = 0.5;
  config.min_range_m = 0.1;
  config.max_range_m = 20.0;
  config.free_confirmations = 2U;
  config.static_confirmations = 2U;
  config.occupied_recovery = 20U;
  config.endpoint_guard_voxels = 0;
  config.dynamic_growth_voxels = 0;
  config.ray_stride = 1;
  config.dynamic_hold_scans = 4U;
  config.vacated_hold_scans = 3U;
  config.dynamic_track_radius_voxels = 1;
  config.vacated_surface_radius_voxels = 1;
  config.min_static_neighbor_voxels = 0U;
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

TEST(VisibilityAwareObserver, MissingRayDoesNotTurnOcclusionIntoFreeSpace)
{
  VisibilityAwareDynamicObserver filter(visibility_test_config());
  const Point origin{0.0, 0.0, 0.0, 0.0F};
  const std::vector<Point> wall{{4.0, 0.0, 0.0, 1.0F}};
  filter.process(wall, origin);
  filter.process(wall, origin);
  ASSERT_EQ(filter.process(wall, origin).points.front().label, PointLabel::kStatic);
  filter.process({}, origin);
  filter.process({}, origin);
  EXPECT_EQ(filter.process(wall, origin).points.front().label, PointLabel::kStatic);
}

TEST(VisibilityAwareObserver, ConfirmedFreeContradictionAndStoppedTargetRemainDynamic)
{
  VisibilityAwareDynamicObserver filter(visibility_test_config());
  const Point origin{0.0, 0.0, 0.0, 0.0F};
  const std::vector<Point> background{{8.0, 0.0, 0.0, 1.0F}};
  filter.process(background, origin);
  filter.process(background, origin);
  const std::vector<Point> target{{4.0, 0.0, 0.0, 1.0F}};
  EXPECT_EQ(filter.process(target, origin).points.front().label, PointLabel::kDynamic);
  EXPECT_EQ(filter.process(target, origin).points.front().label, PointLabel::kDynamic);
  EXPECT_EQ(filter.process(target, origin).points.front().label, PointLabel::kDynamic);
}

TEST(VisibilityAwareObserver, ViewpointChangedFreeVoxelRemainsUnknown)
{
  auto config = visibility_test_config();
  config.dynamic_free_view_bins = 3U;
  VisibilityAwareDynamicObserver filter(config);
  const Point first_origin{0.0, 0.0, 0.0, 0.0F};
  filter.process({{8.0, 0.0, 0.0, 1.0F}}, first_origin);
  filter.process({{8.0, 0.0, 0.0, 1.0F}}, first_origin);
  const Point changed_origin{4.0, -4.0, 0.0, 0.0F};
  const auto result = filter.process({{4.0, 0.0, 0.0, 1.0F}}, changed_origin);
  ASSERT_EQ(result.points.size(), 1U);
  EXPECT_EQ(result.points.front().label, PointLabel::kUnknown);
}

TEST(VisibilityAwareObserver, MultiViewFreeEvidenceSupportsNovelViewDynamicDecision)
{
  auto config = visibility_test_config();
  config.dynamic_free_view_bins = 3U;
  VisibilityAwareDynamicObserver filter(config);
  filter.process({{8.0, 0.0, 0.0, 1.0F}}, {0.0, 0.0, 0.0, 0.0F});
  filter.process({{4.0, 4.0, 0.0, 1.0F}}, {4.0, -4.0, 0.0, 0.0F});
  filter.process({{4.0, -4.0, 0.0, 1.0F}}, {4.0, 4.0, 0.0, 0.0F});
  const auto result = filter.process(
    {{4.0, 0.0, 0.0, 1.0F}}, {0.0, 4.0, 0.0, 0.0F});
  ASSERT_EQ(result.points.size(), 1U);
  EXPECT_EQ(result.points.front().label, PointLabel::kDynamic);
}

TEST(VisibilityAwareObserver, MeasuredVacatedSurfaceSupportsArticulatedMotion)
{
  VisibilityAwareDynamicObserver filter(visibility_test_config());
  const Point origin{0.0, 0.0, 0.0, 0.0F};
  const std::vector<Point> closed_surface{{4.0, 0.0, 0.0, 1.0F}};
  filter.process(closed_surface, origin);
  filter.process(closed_surface, origin);
  filter.process(closed_surface, origin);
  // This measured ray, unlike an absent return, confirms that the old surface
  // cell has been vacated.
  filter.process({{8.0, 0.0, 0.0, 1.0F}}, origin);
  const auto moved_surface = filter.process({{4.0, 0.5, 0.0, 1.0F}}, origin);
  ASSERT_EQ(moved_surface.points.size(), 1U);
  EXPECT_EQ(moved_surface.points.front().label, PointLabel::kDynamic);
}

TEST(VisibilityAwareObserver, NewFieldOfViewStructureIsUnknownBeforeStatic)
{
  VisibilityAwareDynamicObserver filter(visibility_test_config());
  const Point origin{0.0, 0.0, 0.0, 0.0F};
  const std::vector<Point> new_wall{{4.0, 2.0, 0.0, 1.0F}};
  EXPECT_EQ(filter.process(new_wall, origin).points.front().label, PointLabel::kUnknown);
  EXPECT_EQ(filter.process(new_wall, origin).points.front().label, PointLabel::kUnknown);
  EXPECT_EQ(filter.process(new_wall, origin).points.front().label, PointLabel::kStatic);
}

TEST(VisibilityAwareObserver, SparseUnknownReturnIsNotAbsorbedIntoStaticMap)
{
  auto config = visibility_test_config();
  config.far_range_m = 5.0;
  config.far_static_confirmations = 10U;
  VisibilityAwareDynamicObserver filter(config);
  const Point origin{0.0, 0.0, 0.0, 0.0F};
  const std::vector<Point> isolated{{8.0, 2.0, 1.0, 1.0F}};
  for (int scan = 0; scan < 8; ++scan) {
    EXPECT_EQ(filter.process(isolated, origin).points.front().label, PointLabel::kUnknown);
  }
  const std::vector<Point> surface{
    {4.0, 0.0, 0.0, 1.0F}, {4.0, 0.5, 0.0, 1.0F}, {4.0, 0.0, 0.5, 1.0F}};
  filter.process(surface, origin);
  filter.process(surface, origin);
  const auto confirmed = filter.process(surface, origin);
  EXPECT_EQ(confirmed.points.front().label, PointLabel::kStatic);
}

TEST(CausalImuDeskew, RejectsPoseAnchorFromTheFuture)
{
  CausalPose anchor;
  anchor.stamp_ns = 20000000;
  std::vector<CausalImuSample> imu{
    {0, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}},
    {10000000, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}},
    {20000000, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}}};
  const auto result = CausalImuDeskew().propagate(anchor, imu, {10000000});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "future_pose_anchor");
}

TEST(CausalImuDeskew, UsesLivoxNanosecondOffsetsWithoutFuturePose)
{
  CausalPose anchor;
  anchor.stamp_ns = 0;
  anchor.velocity = {1.0, 0.0, 0.0};
  std::vector<CausalImuSample> imu;
  for (std::int64_t stamp = 0; stamp <= 100000000; stamp += 10000000) {
    imu.push_back({stamp, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}});
  }
  const auto result = CausalImuDeskew().propagate(
    anchor, imu, {0, 50000000, 100000000});
  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_EQ(result.poses.size(), 3U);
  EXPECT_NEAR(result.poses[1].position.x(), 0.05, 1.0e-9);
  EXPECT_NEAR(result.poses[2].position.x(), 0.10, 1.0e-9);
  EXPECT_NEAR(result.poses[2].position.z(), 0.0, 1.0e-9);
  EXPECT_LE(result.latest_imu_consumed_ns, result.poses.back().stamp_ns);
}

TEST(CausalImuDeskew, AllowsBoundedTerminalZeroOrderHold)
{
  CausalPose anchor;
  anchor.stamp_ns = 0;
  std::vector<CausalImuSample> imu;
  for (std::int64_t stamp = 0; stamp <= 90000000; stamp += 10000000) {
    imu.push_back({stamp, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}});
  }
  CausalDeskewConfig config;
  config.max_imu_gap_s = 0.015;
  const auto result = CausalImuDeskew(config).propagate(anchor, imu, {100000000});
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_EQ(result.reason, "ok");
  EXPECT_EQ(result.latest_imu_consumed_ns, 90000000);
  EXPECT_NEAR(result.max_observed_imu_gap_s, 0.010, 1.0e-12);
}

TEST(CausalImuDeskew, RejectsTerminalHoldBeyondConfiguredGap)
{
  CausalPose anchor;
  anchor.stamp_ns = 0;
  const std::vector<CausalImuSample> imu{
    {0, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}},
    {10000000, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}},
    {20000000, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}}};
  CausalDeskewConfig config;
  config.max_imu_gap_s = 0.015;
  const auto result = CausalImuDeskew(config).propagate(anchor, imu, {40000000});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "terminal_imu_gap_exceeded");
}

TEST(CausalImuDeskew, RejectsFutureImuRatherThanUsingIt)
{
  CausalPose anchor;
  anchor.stamp_ns = 0;
  const std::vector<CausalImuSample> imu{
    {0, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}},
    {10000000, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}},
    {20000000, {100.0, 0.0, 0.0}, {0.0, 0.0, 10.0}}};
  const auto result = CausalImuDeskew().propagate(anchor, imu, {10000000});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "future_imu_sample");
}

TEST(CausalImuDeskew, RejectsTimestampRegression)
{
  CausalPose anchor;
  anchor.stamp_ns = 0;
  const std::vector<CausalImuSample> imu{
    {0, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}},
    {10000000, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}},
    {9000000, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}}};
  const auto result = CausalImuDeskew().propagate(anchor, imu, {10000000});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "imu_invalid_or_unsorted");
}

TEST(CausalImuDeskew, RejectsMissingImuCoverageAndLargeGap)
{
  CausalPose anchor;
  anchor.stamp_ns = 0;
  CausalDeskewConfig config;
  config.max_imu_gap_s = 0.015;
  const std::vector<CausalImuSample> imu{
    {0, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}},
    {20000000, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}}};
  const auto result = CausalImuDeskew(config).propagate(anchor, imu, {20000000});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "imu_gap_exceeded");
}

TEST(CausalImuDeskew, UsesCalibratedBiasFromPreviousPosterior)
{
  CausalDeskewConfig config;
  config.max_imu_gap_s = 0.02;
  config.gravity_world = Eigen::Vector3d::Zero();
  CausalImuDeskew deskew(config);
  CausalPose anchor;
  anchor.stamp_ns = 1000000000LL;
  anchor.has_calibrated_bias = true;
  anchor.accel_bias = {1.0, 0.0, 0.0};
  anchor.gyro_bias = {0.0, 0.0, 0.1};
  const std::vector<CausalImuSample> imu{
    {1000000000LL, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.1}},
    {1010000000LL, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.1}}};
  const auto result = deskew.propagate(anchor, imu, {1010000000LL});
  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_EQ(result.poses.size(), 1U);
  EXPECT_NEAR(result.poses.front().position.norm(), 0.0, 1.0e-12);
  EXPECT_NEAR(result.poses.front().velocity.norm(), 0.0, 1.0e-12);
  EXPECT_NEAR(
    result.poses.front().orientation.angularDistance(Eigen::Quaterniond::Identity()),
    0.0, 1.0e-12);
}

TEST(CleanScanAdmission, KeepsStaticAndUnknownAndRemovesOnlyConfirmedDynamic)
{
  CleanScanAdmission admission;
  std::vector<LabeledPoint> labels(3U);
  labels[0].label = PointLabel::kStatic;
  labels[1].label = PointLabel::kDynamic;
  labels[2].label = PointLabel::kUnknown;
  const auto result = admission.apply(5U, {0U, 2U, 4U}, labels);
  ASSERT_TRUE(result.healthy);
  EXPECT_FALSE(result.fail_open);
  EXPECT_EQ(result.reason, "ok");
  EXPECT_EQ(result.keep, (std::vector<bool>{true, true, false, true, true}));
  EXPECT_EQ(result.static_points, 1U);
  EXPECT_EQ(result.dynamic_removed, 1U);
  EXPECT_EQ(result.unknown_points, 1U);
}

TEST(CleanScanAdmission, FailsOpenOnMalformedClassificationContract)
{
  CleanScanAdmission admission;
  std::vector<LabeledPoint> labels(2U);
  labels[0].label = PointLabel::kDynamic;
  labels[1].label = PointLabel::kDynamic;
  const auto duplicate = admission.apply(3U, {1U, 1U}, labels);
  EXPECT_FALSE(duplicate.healthy);
  EXPECT_TRUE(duplicate.fail_open);
  EXPECT_EQ(duplicate.reason, "classification_index_duplicate");
  EXPECT_EQ(duplicate.keep, (std::vector<bool>{true, true, true}));

  const auto mismatch = admission.apply(3U, {1U}, labels);
  EXPECT_FALSE(mismatch.healthy);
  EXPECT_EQ(mismatch.reason, "classification_size_mismatch");
  EXPECT_EQ(mismatch.keep, (std::vector<bool>{true, true, true}));
}

TEST(CleanScanAdmission, PreventsObserverFailureFromPublishingAnEmptyFrame)
{
  CleanScanAdmission admission;
  std::vector<LabeledPoint> labels(2U);
  labels[0].label = PointLabel::kDynamic;
  labels[1].label = PointLabel::kDynamic;
  const auto result = admission.apply(2U, {0U, 1U}, labels);
  EXPECT_FALSE(result.healthy);
  EXPECT_TRUE(result.fail_open);
  EXPECT_EQ(result.reason, "empty_clean_scan_guard");
  EXPECT_EQ(result.keep, (std::vector<bool>{true, true}));
}

}  // namespace
}  // namespace uf_dynamic_observer
