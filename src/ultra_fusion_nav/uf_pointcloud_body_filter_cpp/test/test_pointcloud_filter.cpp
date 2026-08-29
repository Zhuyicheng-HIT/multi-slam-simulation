#include <gtest/gtest.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#if __has_include("uf_pointcloud_body_filter_cpp/pointcloud_filter.hpp")
#include "uf_pointcloud_body_filter_cpp/pointcloud_filter.hpp"
#define UF_BODY_FILTER_IMPLEMENTED 1
#else
#define UF_BODY_FILTER_IMPLEMENTED 0
#endif

namespace
{

#if UF_BODY_FILTER_IMPLEMENTED
using uf_pointcloud_body_filter_cpp::FilterConfig;
using uf_pointcloud_body_filter_cpp::FilterResult;
using uf_pointcloud_body_filter_cpp::filter_cloud;

sensor_msgs::msg::PointCloud2 make_cloud(
  const std::vector<std::array<float, 4>> & points,
  const bool big_endian = false,
  const std::uint32_t height = 1U)
{
  sensor_msgs::msg::PointCloud2 message;
  message.header.stamp.sec = 123;
  message.header.stamp.nanosec = 456U;
  message.header.frame_id = "mid360_link";
  message.height = height;
  message.width = static_cast<std::uint32_t>(points.size()) / height;
  message.fields = {
    sensor_msgs::msg::PointField().set__name("x").set__offset(0U)
    .set__datatype(sensor_msgs::msg::PointField::FLOAT32).set__count(1U),
    sensor_msgs::msg::PointField().set__name("y").set__offset(4U)
    .set__datatype(sensor_msgs::msg::PointField::FLOAT32).set__count(1U),
    sensor_msgs::msg::PointField().set__name("z").set__offset(8U)
    .set__datatype(sensor_msgs::msg::PointField::FLOAT32).set__count(1U),
    sensor_msgs::msg::PointField().set__name("intensity").set__offset(12U)
    .set__datatype(sensor_msgs::msg::PointField::FLOAT32).set__count(1U),
    sensor_msgs::msg::PointField().set__name("line").set__offset(16U)
    .set__datatype(sensor_msgs::msg::PointField::UINT8).set__count(1U),
    sensor_msgs::msg::PointField().set__name("tag").set__offset(17U)
    .set__datatype(sensor_msgs::msg::PointField::UINT8).set__count(1U),
  };
  message.is_bigendian = big_endian;
  message.point_step = 20U;
  message.row_step = message.width * message.point_step;
  message.data.resize(points.size() * message.point_step, 0U);
  message.is_dense = true;
  for (std::size_t index = 0; index < points.size(); ++index) {
    auto * record = message.data.data() + index * message.point_step;
    for (std::size_t component = 0; component < points[index].size(); ++component) {
      std::array<std::uint8_t, sizeof(float)> bytes{};
      std::memcpy(bytes.data(), &points[index][component], sizeof(float));
      if (big_endian) {
        std::reverse(bytes.begin(), bytes.end());
      }
      std::memcpy(record + component * sizeof(float), bytes.data(), sizeof(float));
    }
    record[16] = static_cast<std::uint8_t>(index + 3U);
    record[17] = static_cast<std::uint8_t>(200U + index);
    record[18] = 0x5aU;
    record[19] = 0xa5U;
  }
  return message;
}

FilterConfig default_config()
{
  FilterConfig config;
  config.body_bounds = {-0.45, 0.45, -0.45, 0.45, -0.35, 0.15};
  config.min_range_m = 0.1;
  config.max_range_m = 40.0;
  config.lidar_to_body_rotation = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  config.lidar_to_body_translation = {0.0, 0.0, 0.0};
  return config;
}

TEST(PointcloudFilter, PreservesEveryByteOfRetainedRecordsAndMetadata)
{
  const auto input = make_cloud({{0.2F, 0.1F, 0.0F, 10.0F}, {1.0F, 0.0F, 0.0F, 20.0F}});

  const FilterResult result = filter_cloud(input, default_config());

  EXPECT_EQ(result.removed_body, 1U);
  EXPECT_EQ(result.removed_range, 0U);
  EXPECT_EQ(result.total, 2U);
  EXPECT_EQ(result.cloud.header, input.header);
  EXPECT_EQ(result.cloud.fields, input.fields);
  EXPECT_EQ(result.cloud.is_bigendian, input.is_bigendian);
  EXPECT_EQ(result.cloud.point_step, input.point_step);
  EXPECT_EQ(result.cloud.height, 1U);
  EXPECT_EQ(result.cloud.width, 1U);
  EXPECT_EQ(result.cloud.row_step, input.point_step);
  EXPECT_FALSE(result.cloud.is_dense);
  const std::vector<std::uint8_t> expected(
    input.data.begin() + input.point_step, input.data.begin() + 2U * input.point_step);
  EXPECT_EQ(result.cloud.data, expected);
}

TEST(PointcloudFilter, RemovesNonfiniteAndOutOfRangePoints)
{
  const auto input = make_cloud(
    {
      {0.05F, 0.0F, 0.0F, 1.0F},
      {41.0F, 0.0F, 0.0F, 2.0F},
      {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 3.0F},
      {2.0F, 0.0F, 0.0F, 4.0F},
    });

  const FilterResult result = filter_cloud(input, default_config());

  EXPECT_EQ(result.removed_body, 0U);
  EXPECT_EQ(result.removed_range, 3U);
  EXPECT_EQ(result.total, 4U);
  ASSERT_EQ(result.cloud.width, 1U);
  EXPECT_EQ(result.cloud.data[17], 203U);
}

TEST(PointcloudFilter, AppliesLidarToBodyRotationAndTranslation)
{
  const auto input = make_cloud({{1.0F, 0.0F, 0.0F, 10.0F}});
  auto config = default_config();
  config.body_bounds = {0.90, 1.10, -0.10, 0.10, -0.20, -0.10};
  config.lidar_to_body_rotation = {
    0.984807753, 0.0, 0.173648178,
    0.0, 1.0, 0.0,
    -0.173648178, 0.0, 0.984807753};

  const FilterResult result = filter_cloud(input, config);

  EXPECT_EQ(result.removed_body, 1U);
  EXPECT_EQ(result.cloud.width, 0U);
}

TEST(PointcloudFilter, ReadsBigEndianFieldsAtArbitraryOffsets)
{
  const auto input = make_cloud({{0.2F, 0.1F, 0.0F, 10.0F}, {2.0F, 0.0F, 0.0F, 20.0F}}, true);

  const FilterResult result = filter_cloud(input, default_config());

  ASSERT_EQ(result.cloud.width, 1U);
  EXPECT_EQ(result.cloud.data[16], 4U);
  EXPECT_EQ(result.cloud.data[17], 201U);
}

TEST(PointcloudFilter, FlattensOrganizedCloudLikeThePythonContract)
{
  const auto input = make_cloud(
    {{0.2F, 0.0F, 0.0F, 1.0F}, {1.0F, 0.0F, 0.0F, 2.0F},
      {2.0F, 0.0F, 0.0F, 3.0F}, {3.0F, 0.0F, 0.0F, 4.0F}},
    false, 2U);

  const FilterResult result = filter_cloud(input, default_config());

  EXPECT_EQ(result.total, 4U);
  EXPECT_EQ(result.cloud.height, 1U);
  EXPECT_EQ(result.cloud.width, 3U);
  EXPECT_EQ(result.cloud.row_step, 3U * input.point_step);
}

TEST(PointcloudFilter, RejectsCloudWithoutCompleteXyzLayout)
{
  auto input = make_cloud({{1.0F, 0.0F, 0.0F, 2.0F}});
  input.fields.pop_back();
  input.fields.erase(input.fields.begin() + 2);

  EXPECT_THROW(filter_cloud(input, default_config()), std::invalid_argument);
}

#else

TEST(PointcloudFilter, ImplementationMustExist)
{
  FAIL() << "uf_pointcloud_body_filter_cpp/pointcloud_filter.hpp is not implemented";
}

#endif

}  // namespace
