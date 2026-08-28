#include "uf_relocalization/descriptor_core.hpp"
#include "uf_relocalization/offline_esf_seed.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace
{

uf_relocalization::Cloud::Ptr deterministic_cloud()
{
  auto cloud = std::make_shared<uf_relocalization::Cloud>();
  cloud->reserve(1800);
  for (int index = 0; index < 1800; ++index) {
    const float x = 0.04F * static_cast<float>(index % 43);
    const float y = 0.07F * static_cast<float>((index * 11) % 37);
    const float z = 0.35F * std::sin(0.09F * static_cast<float>(index)) +
      0.025F * static_cast<float>((index * 17) % 19);
    cloud->push_back(pcl::PointXYZ{x, y, z});
  }
  return cloud;
}

}  // namespace

TEST(OfflineEsfSeed, ReplaysIdenticalSamplingForTheSameSeed)
{
  const auto cloud = deterministic_cloud();
  uf_relocalization::set_offline_esf_seed(1731U);
  const auto first = uf_relocalization::compute_esf_descriptor(cloud);
  uf_relocalization::set_offline_esf_seed(9713U);
  const auto different_seed = uf_relocalization::compute_esf_descriptor(cloud);
  uf_relocalization::set_offline_esf_seed(1731U);
  const auto replay = uf_relocalization::compute_esf_descriptor(cloud);

  EXPECT_EQ(first, replay);
  EXPECT_NE(first, different_seed);
}
