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
#include <utility>

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

bool proper_rotation(const std::array<double, 9> & rotation)
{
  constexpr double kTolerance = 1.0e-6;
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column < 3U; ++column) {
      double value = 0.0;
      for (std::size_t inner = 0U; inner < 3U; ++inner) {
        value += rotation[inner * 3U + row] * rotation[inner * 3U + column];
      }
      const double expected = row == column ? 1.0 : 0.0;
      if (std::abs(value - expected) > kTolerance) {
        return false;
      }
    }
  }
  const double determinant =
    rotation[0] * (rotation[4] * rotation[8] - rotation[5] * rotation[7]) -
    rotation[1] * (rotation[3] * rotation[8] - rotation[5] * rotation[6]) +
    rotation[2] * (rotation[3] * rotation[7] - rotation[4] * rotation[6]);
  return std::abs(determinant - 1.0) <= kTolerance;
}

void validate_config(const FilterConfig & config)
{
  const auto finite = [](const double value) {return std::isfinite(value);};
  if (!std::all_of(
      config.lidar_to_body_rotation.begin(), config.lidar_to_body_rotation.end(), finite))
  {
    throw std::invalid_argument("lidar_to_body_rotation must contain 9 finite values");
  }
  if (!proper_rotation(config.lidar_to_body_rotation)) {
    throw std::invalid_argument("lidar_to_body_rotation must be a proper orthonormal rotation");
  }
  if (!std::all_of(
      config.lidar_to_body_translation.begin(), config.lidar_to_body_translation.end(), finite))
  {
    throw std::invalid_argument("lidar_to_body_translation must contain 3 finite values");
  }
  if (!std::isfinite(config.min_range_m) || !std::isfinite(config.max_range_m) ||
    config.min_range_m < 0.0 || config.max_range_m <= config.min_range_m)
  {
    throw std::invalid_argument("range limits must be finite and ordered");
  }
  for (const auto & primitive : config.body_primitives) {
    if (!std::all_of(primitive.center_body_m.begin(), primitive.center_body_m.end(), finite) ||
      !std::all_of(primitive.size_m.begin(), primitive.size_m.end(), finite) ||
      !std::isfinite(primitive.padding_m) || primitive.padding_m < 0.0 ||
      !proper_rotation(primitive.body_from_primitive_rotation))
    {
      throw std::invalid_argument("body primitive contains invalid geometry: " + primitive.name);
    }
    if (primitive.type == PrimitiveType::kBox &&
      !std::all_of(primitive.size_m.begin(), primitive.size_m.end(),
        [](double value) {return value > 0.0;}))
    {
      throw std::invalid_argument("box dimensions must be positive: " + primitive.name);
    }
    if (primitive.type == PrimitiveType::kCylinder &&
      (primitive.size_m[0] <= 0.0 || primitive.size_m[1] <= 0.0))
    {
      throw std::invalid_argument("cylinder radius and length must be positive: " + primitive.name);
    }
  }
}

std::array<double, 3> body_point(
  const double x, const double y, const double z, const FilterConfig & config)
{
  const auto & rotation = config.lidar_to_body_rotation;
  const auto & translation = config.lidar_to_body_translation;
  return {
    rotation[0] * x + rotation[1] * y + rotation[2] * z + translation[0],
    rotation[3] * x + rotation[4] * y + rotation[5] * z + translation[1],
    rotation[6] * x + rotation[7] * y + rotation[8] * z + translation[2],
  };
}

bool inside_primitive(const std::array<double, 3> & point, const BodyPrimitive & primitive)
{
  const std::array<double, 3> delta{
    point[0] - primitive.center_body_m[0],
    point[1] - primitive.center_body_m[1],
    point[2] - primitive.center_body_m[2],
  };
  const auto & rotation = primitive.body_from_primitive_rotation;
  const std::array<double, 3> local{
    rotation[0] * delta[0] + rotation[3] * delta[1] + rotation[6] * delta[2],
    rotation[1] * delta[0] + rotation[4] * delta[1] + rotation[7] * delta[2],
    rotation[2] * delta[0] + rotation[5] * delta[1] + rotation[8] * delta[2],
  };
  if (primitive.type == PrimitiveType::kBox) {
    return std::abs(local[0]) <= primitive.size_m[0] * 0.5 + primitive.padding_m &&
           std::abs(local[1]) <= primitive.size_m[1] * 0.5 + primitive.padding_m &&
           std::abs(local[2]) <= primitive.size_m[2] * 0.5 + primitive.padding_m;
  }
  const double radius = primitive.size_m[0] + primitive.padding_m;
  return local[0] * local[0] + local[1] * local[1] <= radius * radius &&
         std::abs(local[2]) <= primitive.size_m[1] * 0.5 + primitive.padding_m;
}

bool inside_body(const std::array<double, 3> & point, const FilterConfig & config)
{
  if (config.geometry_mode == GeometryMode::kComposite) {
    return std::any_of(
      config.body_primitives.begin(), config.body_primitives.end(),
      [&point](const BodyPrimitive & primitive) {return inside_primitive(point, primitive);});
  }
  const auto & bounds = config.body_bounds;
  return bounds[0] <= point[0] && point[0] <= bounds[1] &&
         bounds[2] <= point[1] && point[1] <= bounds[3] &&
         bounds[4] <= point[2] && point[2] <= bounds[5];
}

enum class PointDecision
{
  kKeep,
  kRemoveRange,
  kRemoveBody,
};

PointDecision classify_point(
  const double x, const double y, const double z, const FilterConfig & config)
{
  const double distance = std::sqrt(x * x + y * y + z * z);
  if (!std::isfinite(distance) || distance < config.min_range_m ||
    distance > config.max_range_m)
  {
    return PointDecision::kRemoveRange;
  }
  return inside_body(body_point(x, y, z, config), config) ?
         PointDecision::kRemoveBody : PointDecision::kKeep;
}

}  // namespace

BodyPrimitive BodyPrimitive::box(
  std::string name, const std::array<double, 3> & center_body_m,
  const std::array<double, 3> & dimensions_m, const double padding_m)
{
  BodyPrimitive primitive;
  primitive.name = std::move(name);
  primitive.type = PrimitiveType::kBox;
  primitive.center_body_m = center_body_m;
  primitive.size_m = dimensions_m;
  primitive.padding_m = padding_m;
  return primitive;
}

BodyPrimitive BodyPrimitive::cylinder(
  std::string name, const std::array<double, 3> & center_body_m,
  const double radius_m, const double length_m, const double padding_m)
{
  BodyPrimitive primitive;
  primitive.name = std::move(name);
  primitive.type = PrimitiveType::kCylinder;
  primitive.center_body_m = center_body_m;
  primitive.size_m = {radius_m, length_m, 0.0};
  primitive.padding_m = padding_m;
  return primitive;
}

FilterResult filter_cloud(
  const sensor_msgs::msg::PointCloud2 & input,
  const FilterConfig & config)
{
  if (!config.filter_enabled || !config.geometry_complete) {
    FilterResult result;
    result.cloud = input;
    result.total = input.point_step == 0U ? 0U : input.data.size() / input.point_step;
    result.degraded_fail_open = !config.geometry_complete;
    return result;
  }
  if (input.point_step == 0U) {
    throw std::invalid_argument("PointCloud2 must contain x/y/z fields and a positive point_step");
  }
  const auto & x_field = require_field(input, "x");
  const auto & y_field = require_field(input, "y");
  const auto & z_field = require_field(input, "z");
  validate_config(config);

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

  for (std::size_t index = 0; index < count; ++index) {
    const std::size_t base = index * input.point_step;
    const auto * record = input.data.data() + base;
    const double x = read_scalar(record, x_field, input.is_bigendian, input.point_step);
    const double y = read_scalar(record, y_field, input.is_bigendian, input.point_step);
    const double z = read_scalar(record, z_field, input.is_bigendian, input.point_step);
    const PointDecision decision = classify_point(x, y, z, config);
    if (decision == PointDecision::kRemoveRange) {
      ++result.removed_range;
      continue;
    }
    if (decision == PointDecision::kRemoveBody) {
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

LivoxFilterResult filter_livox_cloud(
  const livox_ros_driver2::msg::CustomMsg & input,
  const FilterConfig & config)
{
  LivoxFilterResult result;
  result.total = input.points.size();
  if (!config.filter_enabled || !config.geometry_complete) {
    result.cloud = input;
    result.degraded_fail_open = !config.geometry_complete;
    return result;
  }
  validate_config(config);
  result.cloud.header = input.header;
  result.cloud.timebase = input.timebase;
  result.cloud.lidar_id = input.lidar_id;
  result.cloud.rsvd = input.rsvd;
  result.cloud.points.reserve(input.points.size());
  for (const auto & point : input.points) {
    const PointDecision decision = classify_point(point.x, point.y, point.z, config);
    if (decision == PointDecision::kRemoveRange) {
      ++result.removed_range;
    } else if (decision == PointDecision::kRemoveBody) {
      ++result.removed_body;
    } else {
      result.cloud.points.push_back(point);
    }
  }
  result.cloud.point_num = static_cast<std::uint32_t>(result.cloud.points.size());
  return result;
}

}  // namespace uf_pointcloud_body_filter_cpp
