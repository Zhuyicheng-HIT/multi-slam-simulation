#pragma once

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace uf_pointcloud_body_filter_cpp
{

enum class GeometryMode
{
  kLegacyAxisAlignedBox,
  kComposite,
};

enum class PrimitiveType
{
  kBox,
  kCylinder,
};

struct BodyPrimitive
{
  std::string name;
  PrimitiveType type{PrimitiveType::kBox};
  std::array<double, 3> center_body_m{};
  std::array<double, 9> body_from_primitive_rotation{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  // Box: x/y/z dimensions. Cylinder: radius/length/unused, local z axis.
  std::array<double, 3> size_m{};
  double padding_m{0.0};

  static BodyPrimitive box(
    std::string name,
    const std::array<double, 3> & center_body_m,
    const std::array<double, 3> & dimensions_m,
    double padding_m = 0.0);

  static BodyPrimitive cylinder(
    std::string name,
    const std::array<double, 3> & center_body_m,
    double radius_m,
    double length_m,
    double padding_m = 0.0);
};

struct FilterConfig
{
  bool filter_enabled{true};
  bool geometry_complete{true};
  bool fail_open{true};
  GeometryMode geometry_mode{GeometryMode::kLegacyAxisAlignedBox};
  std::array<double, 6> body_bounds{};
  double min_range_m{0.1};
  double max_range_m{100.0};
  std::array<double, 9> lidar_to_body_rotation{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  std::array<double, 3> lidar_to_body_translation{};
  std::vector<BodyPrimitive> body_primitives;
};

struct FilterResult
{
  sensor_msgs::msg::PointCloud2 cloud;
  std::size_t removed_body{0U};
  std::size_t removed_range{0U};
  std::size_t total{0U};
  bool degraded_fail_open{false};
};

struct LivoxFilterResult
{
  livox_ros_driver2::msg::CustomMsg cloud;
  std::size_t removed_body{0U};
  std::size_t removed_range{0U};
  std::size_t total{0U};
  bool degraded_fail_open{false};
};

FilterResult filter_cloud(
  const sensor_msgs::msg::PointCloud2 & input,
  const FilterConfig & config);

LivoxFilterResult filter_livox_cloud(
  const livox_ros_driver2::msg::CustomMsg & input,
  const FilterConfig & config);

}  // namespace uf_pointcloud_body_filter_cpp
