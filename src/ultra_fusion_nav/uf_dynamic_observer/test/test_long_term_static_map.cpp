#include "uf_dynamic_observer/long_term_static_map.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

namespace uf_dynamic_observer
{
namespace
{

LabeledPoint observation(Point point, PointLabel label)
{
  return {point, label, label == PointLabel::kDynamic ? 1.0F :
    (label == PointLabel::kStatic ? 0.0F : 0.5F)};
}

LongTermMapConfig fast_config()
{
  LongTermMapConfig config;
  config.voxel_size_m = 0.25;
  config.static_candidate_observations = 2U;
  config.static_confirmed_observations = 4U;
  config.static_confirmed_duration_s = 0.3;
  config.static_confirmed_view_bins = 2U;
  config.static_consistency_ratio = 0.60;
  config.candidate_free_contradictions = 2U;
  config.dynamic_candidate_free_traversals = 2U;
  config.dynamic_confirmed_free_traversals = 4U;
  config.dynamic_confirmed_view_bins = 2U;
  config.dynamic_confirmed_duration_s = 0.2;
  config.dynamic_label_confirmations = 2U;
  config.dynamic_recovery_static_observations = 5U;
  config.dynamic_recovery_duration_s = 0.4;
  config.far_range_m = 8.0;
  config.far_static_confirmed_observations = 6U;
  config.endpoint_guard_voxels = 0;
  config.ray_stride = 1;
  return config;
}

void confirm_static(LongTermStaticMap & map, const Point & target)
{
  const std::vector<Point> origins{
    {0.0, -1.0, 1.0, 0.0F}, {0.0, 1.0, 1.0, 0.0F},
    {-1.0, 0.0, 1.0, 0.0F}, {1.0, 0.0, 1.0, 0.0F},
    {0.0, -1.0, 1.0, 0.0F}, {0.0, 1.0, 1.0, 0.0F}};
  for (std::size_t index = 0U; index < origins.size(); ++index) {
    ASSERT_TRUE(map.integrate(
      {observation(target, PointLabel::kStatic)}, origins[index], index * 0.1).accepted);
  }
}

TEST(LongTermStaticMap, SingleStaticFrameIsNotPermanent)
{
  LongTermStaticMap map(fast_config());
  const Point target{3.0, 0.0, 1.0, 10.0F};
  EXPECT_TRUE(map.integrate(
    {observation(target, PointLabel::kStatic)}, {0.0, 0.0, 1.0, 0.0F}, 0.0).accepted);
  EXPECT_NE(map.state_at(target), LongTermVoxelState::kStaticConfirmed);
  EXPECT_TRUE(map.static_confirmed_points().empty());
}

TEST(LongTermStaticMap, MultiFrameMultiViewPersistentStaticIsAdmitted)
{
  LongTermStaticMap map(fast_config());
  const Point target{3.0, 0.0, 1.0, 10.0F};
  confirm_static(map, target);
  EXPECT_EQ(map.state_at(target), LongTermVoxelState::kStaticConfirmed);
  EXPECT_EQ(map.static_confirmed_points().size(), 1U);
  EXPECT_GE(map.stats().mean_admission_delay_s, 0.3);
}

TEST(LongTermStaticMap, UnknownNeverEarnsPermanentAdmission)
{
  LongTermStaticMap map(fast_config());
  const Point target{3.0, 0.0, 1.0, 10.0F};
  for (int index = 0; index < 20; ++index) {
    ASSERT_TRUE(map.integrate(
      {observation(target, PointLabel::kUnknown)},
      {0.0, index % 2 == 0 ? -1.0 : 1.0, 1.0, 0.0F}, index * 0.1).accepted);
  }
  EXPECT_EQ(map.state_at(target), LongTermVoxelState::kUnknown);
  EXPECT_TRUE(map.static_confirmed_points().empty());
}

TEST(LongTermStaticMap, ExplicitDynamicEvidenceIsExcluded)
{
  LongTermStaticMap map(fast_config());
  const Point target{3.0, 0.0, 1.0, 10.0F};
  ASSERT_TRUE(map.integrate(
    {observation(target, PointLabel::kDynamic)}, {0.0, -1.0, 1.0, 0.0F}, 0.0).accepted);
  ASSERT_TRUE(map.integrate(
    {observation(target, PointLabel::kDynamic)}, {0.0, 1.0, 1.0, 0.0F}, 0.1).accepted);
  EXPECT_EQ(map.state_at(target), LongTermVoxelState::kDynamicConfirmed);
  EXPECT_TRUE(map.static_confirmed_points().empty());
}

TEST(LongTermStaticMap, ActualFreeRaysRemoveAPreviouslyAdmittedGhost)
{
  LongTermStaticMap map(fast_config());
  const Point ghost{2.0, 0.0, 1.0, 10.0F};
  confirm_static(map, ghost);
  ASSERT_EQ(map.state_at(ghost), LongTermVoxelState::kStaticConfirmed);
  for (int index = 0; index < 8; ++index) {
    const bool from_left = index % 2 == 0;
    const Point background{from_left ? 4.0 : 0.0, 0.0, 1.0, 10.0F};
    const Point origin{from_left ? 0.0 : 4.5, 0.0, 1.0, 0.0F};
    ASSERT_TRUE(map.integrate(
      {observation(background, PointLabel::kStatic)}, origin,
      1.0 + index * 0.1).accepted);
  }
  EXPECT_EQ(map.state_at(ghost), LongTermVoxelState::kDynamicConfirmed);
  EXPECT_GE(map.stats().removed_ghost_voxels, 1U);
}

TEST(LongTermStaticMap, MissingObservationDoesNotMeanFree)
{
  LongTermStaticMap map(fast_config());
  const Point wall{3.0, 0.0, 1.0, 10.0F};
  confirm_static(map, wall);
  for (int index = 0; index < 20; ++index) {
    // Rays in the opposite half-space cannot contradict the wall.
    ASSERT_TRUE(map.integrate(
      {observation({-3.0, 0.0, 1.0, 10.0F}, PointLabel::kStatic)},
      {0.0, 0.0, 1.0, 0.0F}, 1.0 + index * 0.1).accepted);
  }
  EXPECT_EQ(map.state_at(wall), LongTermVoxelState::kStaticConfirmed);
}

TEST(LongTermStaticMap, LongOcclusionCannotDeleteConfirmedStaticWithoutMeasuredRay)
{
  LongTermStaticMap map(fast_config());
  const Point wall{3.0, 0.0, 1.0, 10.0F};
  confirm_static(map, wall);
  for (int index = 0; index < 500; ++index) {
    ASSERT_TRUE(map.integrate({}, {0.0, 0.0, 1.0, 0.0F}, 1.0 + index * 0.1).accepted);
  }
  EXPECT_EQ(map.state_at(wall), LongTermVoxelState::kStaticConfirmed);
  EXPECT_EQ(map.static_confirmed_points().size(), 1U);
  EXPECT_EQ(map.stats().removed_ghost_voxels, 0U);
}

TEST(LongTermStaticMap, CapacityBoundPreservesConfirmedMapAndRejectsNewCandidate)
{
  auto config = fast_config();
  config.max_voxels = 1U;
  LongTermStaticMap map(config);
  const Point wall{3.0, 0.0, 1.0, 10.0F};
  confirm_static(map, wall);
  ASSERT_EQ(map.stats().allocated_voxels, 1U);
  ASSERT_TRUE(map.integrate(
    {observation({-3.0, 0.0, 1.0, 10.0F}, PointLabel::kUnknown)},
    {0.0, 0.0, 1.0, 0.0F}, 1.0).accepted);
  EXPECT_EQ(map.stats().allocated_voxels, 1U);
  EXPECT_EQ(map.state_at(wall), LongTermVoxelState::kStaticConfirmed);
  EXPECT_EQ(map.stats().capacity_rejected_voxels, 1U);
}

TEST(LongTermStaticMap, CapacityPressureEvictsOnlyStaleNonConfirmedState)
{
  auto config = fast_config();
  config.max_voxels = 2U;
  config.stale_dynamic_after_scans = 1U;
  LongTermStaticMap map(config);
  ASSERT_TRUE(map.integrate(
    {observation({2.0, -1.0, 1.0, 10.0F}, PointLabel::kUnknown)},
    {0.0, 0.0, 1.0, 0.0F}, 0.0).accepted);
  ASSERT_TRUE(map.integrate(
    {observation({2.0, 1.0, 1.0, 10.0F}, PointLabel::kUnknown)},
    {0.0, 0.0, 1.0, 0.0F}, 0.1).accepted);
  ASSERT_TRUE(map.integrate({}, {0.0, 0.0, 1.0, 0.0F}, 0.2).accepted);
  ASSERT_TRUE(map.integrate(
    {observation({3.0, 0.0, 1.0, 10.0F}, PointLabel::kUnknown)},
    {0.0, 0.0, 1.0, 0.0F}, 0.3).accepted);
  EXPECT_LE(map.stats().allocated_voxels, 2U);
  EXPECT_EQ(map.stats().capacity_rejected_voxels, 0U);
}

TEST(LongTermStaticMap, TimestampRegressionIsRejectedWithoutMutation)
{
  LongTermStaticMap map(fast_config());
  const Point target{3.0, 0.0, 1.0, 10.0F};
  ASSERT_TRUE(map.integrate(
    {observation(target, PointLabel::kStatic)}, {0.0, 0.0, 1.0, 0.0F}, 2.0).accepted);
  const auto before = map.stats();
  const auto rejected = map.integrate(
    {observation(target, PointLabel::kStatic)}, {0.0, 0.0, 1.0, 0.0F}, 1.0);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.reason, "timestamp_regression");
  EXPECT_EQ(map.stats().accepted_scans, before.accepted_scans);
  EXPECT_EQ(map.stats().timestamp_regressions, 1U);
}

TEST(LongTermStaticMap, SemanticShadowCannotDeleteGeometry)
{
  LongTermStaticMap map(fast_config());
  const Point wall{3.0, 0.0, 1.0, 10.0F};
  confirm_static(map, wall);
  ASSERT_TRUE(map.add_semantic_evidence(wall, 0.95F, 1.0, true));
  ASSERT_TRUE(map.add_semantic_evidence(wall, 0.95F, 1.1, true));
  EXPECT_EQ(map.state_at(wall), LongTermVoxelState::kStaticConfirmed);
  EXPECT_EQ(map.stats().semantic_shadow_hits, 2U);
  EXPECT_EQ(map.stats().semantic_applied_hits, 0U);
}

TEST(LongTermStaticMap, AppliedSemanticIsAuxiliaryNotRequired)
{
  LongTermStaticMap map(fast_config());
  const Point target{3.0, 0.0, 1.0, 10.0F};
  confirm_static(map, target);
  ASSERT_TRUE(map.add_semantic_evidence(target, 0.95F, 1.0, false));
  ASSERT_TRUE(map.add_semantic_evidence(target, 0.95F, 1.1, false));
  EXPECT_EQ(map.state_at(target), LongTermVoxelState::kDynamicConfirmed);
  EXPECT_EQ(map.stats().semantic_applied_hits, 2U);
}

TEST(LongTermStaticMap, FarUnknownRemainsUnknown)
{
  auto config = fast_config();
  config.max_range_m = 30.0;
  LongTermStaticMap map(config);
  const Point sparse{16.0, 0.0, 1.0, 10.0F};
  for (int index = 0; index < 20; ++index) {
    ASSERT_TRUE(map.integrate(
      {observation(sparse, PointLabel::kUnknown)},
      {0.0, index % 2 == 0 ? -1.0 : 1.0, 1.0, 0.0F}, index * 0.1).accepted);
  }
  EXPECT_EQ(map.state_at(sparse), LongTermVoxelState::kUnknown);
}

TEST(LongTermStaticMap, CandidateFreeContradictionIsReversible)
{
  LongTermStaticMap map(fast_config());
  const Point candidate{2.0, 0.0, 1.0, 10.0F};
  ASSERT_TRUE(map.integrate(
    {observation(candidate, PointLabel::kStatic)}, {0.0, -1.0, 1.0, 0.0F}, 0.0).accepted);
  ASSERT_TRUE(map.integrate(
    {observation(candidate, PointLabel::kStatic)}, {0.0, 1.0, 1.0, 0.0F}, 0.1).accepted);
  ASSERT_EQ(map.state_at(candidate), LongTermVoxelState::kStaticCandidate);
  const Point background_left{4.0, 0.0, 1.0, 10.0F};
  const Point background_right{0.0, 0.0, 1.0, 10.0F};
  ASSERT_TRUE(map.integrate(
    {observation(background_left, PointLabel::kStatic)}, {0.0, 0.0, 1.0, 0.0F}, 0.2).accepted);
  ASSERT_TRUE(map.integrate(
    {observation(background_right, PointLabel::kStatic)}, {4.5, 0.0, 1.0, 0.0F}, 0.3).accepted);
  EXPECT_EQ(map.state_at(candidate), LongTermVoxelState::kUnknown);
}

TEST(LongTermStaticMap, InvalidConfigurationIsRejected)
{
  auto config = fast_config();
  config.static_confirmed_observations = 1U;
  EXPECT_THROW(LongTermStaticMap map(config), std::invalid_argument);
}

}  // namespace
}  // namespace uf_dynamic_observer
