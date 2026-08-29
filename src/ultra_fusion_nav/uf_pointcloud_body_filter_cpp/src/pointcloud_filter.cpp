#include "uf_pointcloud_body_filter_cpp/pointcloud_filter.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace uf_pointcloud_body_filter_cpp
{
namespace
{

bool host_is_big_endian()
{
  const std::uint16_t marker = 0x0102U;
  return *reinterpret_cast<const std::uint8_t *>(&marker) == 0x01U;
}

template<typename T>
T read_binary(const std::uint8_t * source, const bool source_big_endian)
{
  static_assert(std::is_arithmetic<T>::value, "PointField scalar must be arithmetic");
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::copy_n(source, sizeof(T), bytes.begin());
  if (source_big_endian != host_is_big_endian()) {
    std::reverse(bytes.begin(), bytes.end());
  }
  T value{};
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}

std::size_t datatype_size(const std::uint8_t datatype)
{
  using Field = sensor_msgs::msg::PointField;
  switch (datatype) {
    case Field::INT8:
    case Field::UINT8:
      return 1U;
    case Field::INT16:
    case Field::UINT16:
      return 2U;
    case Field::INT32:
    case Field::UINT32:
    case Field::FLOAT32:
      return 4U;
    case Field::FLOAT64:
      return 8U;
    default:
      return 0U;
  }
}

double read_scalar(
  const std::uint8_t * record,
  const sensor_msgs::msg::PointField & field,
  const bool big_endian,
  const std::size_t point_step)
{
  using Field = sensor_msgs::msg::PointField;
  const std::size_t size = datatype_size(field.datatype);
  if (size == 0U) {
    throw std::invalid_argument(
            "Unsupported PointField datatype for " + field.name + ": " +
            std::to_string(field.datatype));
  }
  if (field.offset > point_step || size > point_step - field.offset) {
    throw std::invalid_argument("PointField " + field.name + " exceeds point_step");
  }
  const auto * value = record + field.offset;
  switch (field.datatype) {
    case Field::INT8:
      return static_cast<double>(read_binary<std::int8_t>(value, big_endian));
    case Field::UINT8:
      return static_cast<double>(read_binary<std::uint8_t>(value, big_endian));
    case Field::INT16:
      return static_cast<double>(read_binary<std::int16_t>(value, big_endian));
    case Field::UINT16:
      return static_cast<double>(read_binary<std::uint16_t>(value, big_endian));
    case Field::INT32:
      return static_cast<double>(read_binary<std::int32_t>(value, big_endian));
    case Field::UINT32:
      return static_cast<double>(read_binary<std::uint32_t>(value, big_endian));
    case Field::FLOAT32:
      return static_cast<double>(read_binary<float>(value, big_endian));
    case Field::FLOAT64:
      return read_binary<double>(value, big_endian);
    default:
      throw std::invalid_argument("Unsupported PointField datatype");
  }
}

const sensor_msgs::msg::PointField & require_field(
  const sensor_msgs::msg::PointCloud2 & input,
  const std::string & name)
{
  const auto found = std::find_if(
    input.fields.begin(), input.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) {return field.name == name;});
  if (found == input.fields.end()) {
    throw std::invalid_argument("PointCloud2 must contain x/y/z fields and a positive point_step");
  }
  return *found;
}

void validate_finite_extrinsic(const FilterConfig & config)
{
  const auto finite = [](const double value) {return std::isfinite(value);};
  if (!std::all_of(
      config.lidar_to_body_rotation.begin(), config.lidar_to_body_rotation.end(), finite))
  {
    throw std::invalid_argument("lidar_to_body_rotation must contain 9 finite values");
  }
  if (!std::all_of(
      config.lidar_to_body_translation.begin(), config.lidar_to_body_translation.end(), finite))
  {
    throw std::invalid_argument("lidar_to_body_translation must contain 3 finite values");
  }
}

}  // namespace

FilterResult filter_cloud(
  const sensor_msgs::msg::PointCloud2 & input,
  const FilterConfig & config)
{
  if (input.point_step == 0U) {
    throw std::invalid_argument("PointCloud2 must contain x/y/z fields and a positive point_step");
  }
  const auto & x_field = require_field(input, "x");
  const auto & y_field = require_field(input, "y");
  const auto & z_field = require_field(input, "z");
  validate_finite_extrinsic(config);

  const std::uint64_t declared_count =
    static_cast<std::uint64_t>(input.width) * static_cast<std::uint64_t>(input.height);
  const std::size_t available_count = input.data.size() / input.point_step;
  const std::size_t count = static_cast<std::size_t>(std::min<std::uint64_t>(
      declared_count, static_cast<std::uint64_t>(available_count)));

  FilterResult result;
  result.total = count;
  result.cloud.header = input.header;
  result.cloud.height = 1U;
  result.cloud.fields = input.fields;
  result.cloud.is_bigendian = input.is_bigendian;
  result.cloud.point_step = input.point_step;
  result.cloud.is_dense = false;
  result.cloud.data.reserve(count * input.point_step);

  const auto & bounds = config.body_bounds;
  const auto & rotation = config.lidar_to_body_rotation;
  const auto & translation = config.lidar_to_body_translation;
  for (std::size_t index = 0; index < count; ++index) {
    const std::size_t base = index * input.point_step;
    const auto * record = input.data.data() + base;
    const double x = read_scalar(record, x_field, input.is_bigendian, input.point_step);
    const double y = read_scalar(record, y_field, input.is_bigendian, input.point_step);
    const double z = read_scalar(record, z_field, input.is_bigendian, input.point_step);
    const double distance = std::sqrt(x * x + y * y + z * z);
    if (!std::isfinite(distance) || distance < config.min_range_m ||
      distance > config.max_range_m)
    {
      ++result.removed_range;
      continue;
    }
    const double body_x =
      rotation[0] * x + rotation[1] * y + rotation[2] * z + translation[0];
    const double body_y =
      rotation[3] * x + rotation[4] * y + rotation[5] * z + translation[1];
    const double body_z =
      rotation[6] * x + rotation[7] * y + rotation[8] * z + translation[2];
    if (bounds[0] <= body_x && body_x <= bounds[1] &&
      bounds[2] <= body_y && body_y <= bounds[3] &&
      bounds[4] <= body_z && body_z <= bounds[5])
    {
      ++result.removed_body;
      continue;
    }
    result.cloud.data.insert(
      result.cloud.data.end(),
      input.data.begin() + static_cast<std::ptrdiff_t>(base),
      input.data.begin() + static_cast<std::ptrdiff_t>(base + input.point_step));
  }

  const std::size_t retained = result.cloud.data.size() / input.point_step;
  if (retained > std::numeric_limits<std::uint32_t>::max() ||
    retained * input.point_step > std::numeric_limits<std::uint32_t>::max())
  {
    throw std::overflow_error("Filtered PointCloud2 dimensions exceed uint32 fields");
  }
  result.cloud.width = static_cast<std::uint32_t>(retained);
  result.cloud.row_step = static_cast<std::uint32_t>(retained * input.point_step);
  return result;
}

}  // namespace uf_pointcloud_body_filter_cpp
