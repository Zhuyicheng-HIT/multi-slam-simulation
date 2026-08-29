#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace uf_pointcloud_body_filter_cpp
{

struct FilterConfig
{
  std::array<double, 6> body_bounds{};
  double min_range_m{0.1};
  double max_range_m{100.0};
  std::array<double, 9> lidar_to_body_rotation{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  std::array<double, 3> lidar_to_body_translation{};
};

struct FilterResult
{
  sensor_msgs::msg::PointCloud2 cloud;
  std::size_t removed_body{0U};
  std::size_t removed_range{0U};
  std::size_t total{0U};
};

FilterResult filter_cloud(
  const sensor_msgs::msg::PointCloud2 & input,
  const FilterConfig & config);

}  // namespace uf_pointcloud_body_filter_cpp
