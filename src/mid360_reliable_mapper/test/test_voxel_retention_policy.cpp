#include <algorithm>
#include <vector>

#include "gtest/gtest.h"
#include "mid360_reliable_mapper/voxel_retention_policy.hpp"

using mid360_reliable_mapper::VoxelRetentionEvidence;
using mid360_reliable_mapper::VoxelRetentionPolicy;

TEST(VoxelRetentionPolicy, RemovesLowSupportBeforeStableGeometry)
{
  VoxelRetentionPolicy policy(4U, 1U);
  const VoxelRetentionEvidence noisy{0, 0, 0, 1U, 90U, 0U, false};
  const VoxelRetentionEvidence wall{1, 0, 0, 8U, 10U, 5U, true};
  EXPECT_TRUE(policy.should_evict_before(noisy, wall, 100U));
}

TEST(VoxelRetentionPolicy, RemovesOlderBeforeNewerAtEqualSupport)
{
  VoxelRetentionPolicy policy(4U, 1U);
  const VoxelRetentionEvidence old_voxel{0, 0, 0, 2U, 20U, 3U, true};
  const VoxelRetentionEvidence new_voxel{1, 0, 0, 2U, 90U, 3U, true};
  EXPECT_TRUE(policy.should_evict_before(old_voxel, new_voxel, 100U));
}

TEST(VoxelRetentionPolicy, RemovesIsolatedAndUnsupportedFloatingNoise)
{
  VoxelRetentionPolicy policy(4U, 1U);
  const VoxelRetentionEvidence floating{0, 0, 4, 2U, 80U, 0U, false};
  const VoxelRetentionEvidence connected{0, 0, 0, 2U, 80U, 5U, true};
  EXPECT_TRUE(policy.should_evict_before(floating, connected, 100U));
}

TEST(VoxelRetentionPolicy, ProtectsMultiFrameWallAndGround)
{
  VoxelRetentionPolicy policy(4U, 1U);
  std::vector<VoxelRetentionEvidence> evidence{
    {2, 2, 2, 1U, 99U, 0U, false},
    {1, 0, 0, 7U, 10U, 6U, true},
    {0, 0, 0, 9U, 5U, 8U, true},
  };
  std::sort(evidence.begin(), evidence.end(), [&](const auto & lhs, const auto & rhs) {
    return policy.should_evict_before(lhs, rhs, 100U);
  });
  EXPECT_EQ(evidence.front().support_count, 1U);
  EXPECT_GE(evidence.back().support_count, 7U);
}

TEST(VoxelRetentionPolicy, IsDeterministicForEqualEvidence)
{
  VoxelRetentionPolicy policy(4U, 1U);
  const VoxelRetentionEvidence first{1, 2, 3, 2U, 50U, 2U, true};
  const VoxelRetentionEvidence second{2, 2, 3, 2U, 50U, 2U, true};
  EXPECT_TRUE(policy.should_evict_before(first, second, 100U));
  EXPECT_FALSE(policy.should_evict_before(second, first, 100U));
}
