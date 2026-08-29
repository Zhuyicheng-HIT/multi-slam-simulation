#include "uf_pointcloud_body_filter_cpp/pointcloud_filter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace uf_pointcloud_body_filter_cpp
{
namespace
{

struct GeometryDefaults
{
  std::string input_topic{"/sim/mid360/points_raw"};
  std::string output_topic{"/sensors/lidar/points_body_filtered"};
  bool filter_enabled{true};
  bool geometry_complete{true};
  bool fail_open{true};
  std::string geometry_mode{"legacy_aabb"};
  std::array<double, 9> rotation{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  std::optional<std::array<double, 3>> translation{std::array<double, 3>{0.0, 0.0, 0.0}};
  std::vector<BodyPrimitive> primitives;
};

std::string resolve_contract_path(const std::string & path)
{
  constexpr const char * kPrefix = "package://";
  if (path.rfind(kPrefix, 0U) != 0U) {
    return path;
  }
  const std::string package_and_path = path.substr(std::char_traits<char>::length(kPrefix));
  const std::size_t separator = package_and_path.find('/');
  if (separator == std::string::npos || separator == 0U ||
    separator + 1U >= package_and_path.size())
  {
    throw std::invalid_argument("invalid package:// geometry contract URI");
  }
  const std::string package = package_and_path.substr(0U, separator);
  const std::string relative = package_and_path.substr(separator + 1U);
  return ament_index_cpp::get_package_share_directory(package) + "/" + relative;
}

template<std::size_t Size>
std::array<double, Size> yaml_array(const YAML::Node & value, const std::string & name)
{
  if (!value || !value.IsSequence() || value.size() != Size) {
    throw std::invalid_argument(name + " must contain " + std::to_string(Size) + " values");
  }
  std::array<double, Size> result{};
  for (std::size_t index = 0U; index < Size; ++index) {
    result[index] = value[index].as<double>();
    if (!std::isfinite(result[index])) {
      throw std::invalid_argument(name + " must contain finite values");
    }
  }
  return result;
}

GeometryDefaults load_geometry_defaults(const std::string & path)
{
  GeometryDefaults defaults;
  if (path.empty()) {
    return defaults;
  }
  const YAML::Node root = YAML::LoadFile(resolve_contract_path(path));
  if (!root["schema_version"] || root["schema_version"].as<int>() != 1) {
    throw std::invalid_argument("geometry contract schema_version must be 1");
  }
  defaults.input_topic = root["topics"]["lidar_raw"].as<std::string>();
  defaults.output_topic = root["topics"]["lidar_filtered"].as<std::string>();
  const YAML::Node body_lidar = root["transforms"]["body_lidar"];
  defaults.rotation = yaml_array<9>(body_lidar["rotation_matrix"], "body_lidar rotation");
  if (!body_lidar["translation_m"] || body_lidar["translation_m"].IsNull()) {
    defaults.translation.reset();
    defaults.geometry_complete = false;
  } else {
    defaults.translation = yaml_array<3>(body_lidar["translation_m"], "body_lidar translation");
  }
  const YAML::Node envelope = root["body_envelope"];
  defaults.filter_enabled = envelope["enabled"].as<bool>(false);
  defaults.fail_open = envelope["fail_open"].as<bool>(true);
  defaults.geometry_mode = envelope["mode"].as<std::string>("composite");
  const YAML::Node primitives = envelope["primitives"];
  if (primitives && primitives.IsSequence()) {
    for (const auto & item : primitives) {
      BodyPrimitive primitive;
      primitive.name = item["name"].as<std::string>();
      const std::string type = item["type"].as<std::string>();
      primitive.type = type == "box" ? PrimitiveType::kBox : PrimitiveType::kCylinder;
      if (type != "box" && type != "cylinder") {
        throw std::invalid_argument("unknown body primitive type: " + type);
      }
      primitive.center_body_m = yaml_array<3>(item["center_body_m"], "primitive center");
      primitive.body_from_primitive_rotation = yaml_array<9>(
        item["rotation_matrix"], "primitive rotation");
      if (primitive.type == PrimitiveType::kBox) {
        primitive.size_m = yaml_array<3>(item["dimensions_m"], "box dimensions");
      } else {
        primitive.size_m = {
          item["radius_m"].as<double>(), item["length_m"].as<double>(), 0.0};
      }
      primitive.padding_m = item["padding_m"].as<double>(0.0);
      defaults.primitives.push_back(std::move(primitive));
    }
  }
  defaults.geometry_complete = defaults.geometry_complete && !defaults.primitives.empty();
  defaults.filter_enabled = defaults.filter_enabled && defaults.geometry_complete;
  return defaults;
}

template<std::size_t Size>
std::array<double, Size> finite_array(
  const std::vector<double> & values,
  const std::string & name)
{
  if (values.size() != Size ||
    !std::all_of(
      values.begin(), values.end(), [](const double value) {
        return std::isfinite(value);
      }))
  {
    throw std::invalid_argument(
            name + " must contain " + std::to_string(Size) + " finite values");
  }
  std::array<double, Size> output{};
  std::copy(values.begin(), values.end(), output.begin());
  return output;
}

double percentile(std::vector<double> samples, const double quantile)
{
  if (samples.empty()) {
    return 0.0;
  }
  std::sort(samples.begin(), samples.end());
  const double position = quantile * static_cast<double>(samples.size() - 1U);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  const double weight = position - static_cast<double>(lower);
  return samples[lower] * (1.0 - weight) + samples[upper] * weight;
}

}  // namespace

class PointCloudBodyFilterNode final : public rclcpp::Node
{
public:
  PointCloudBodyFilterNode()
  : Node("pointcloud_body_filter")
  {
    const bool parameter_fail_open = declare_parameter<bool>("fail_open", true);
    const std::string geometry_contract_file = declare_parameter<std::string>(
      "geometry_contract_file", "");
    GeometryDefaults defaults;
    try {
      defaults = load_geometry_defaults(geometry_contract_file);
    } catch (const std::exception & exception) {
      if (!parameter_fail_open) {
        throw;
      }
      defaults.filter_enabled = false;
      defaults.geometry_complete = false;
      degraded_reason_ = std::string("geometry_contract_error:") + exception.what();
    }
    const std::string input_topic = declare_parameter<std::string>(
      "input_topic", defaults.input_topic);
    const std::string output_topic = declare_parameter<std::string>(
      "output_topic", defaults.output_topic);
    const std::string input_message_type = declare_parameter<std::string>(
      "input_message_type", "pointcloud2");
    config_.filter_enabled = declare_parameter<bool>("filter_enabled", defaults.filter_enabled);
    config_.geometry_complete = declare_parameter<bool>(
      "geometry_complete", defaults.geometry_complete);
    config_.fail_open = parameter_fail_open;
    const std::string geometry_mode = declare_parameter<std::string>(
      "geometry_mode", defaults.geometry_mode);
    if (geometry_mode == "legacy_aabb") {
      config_.geometry_mode = GeometryMode::kLegacyAxisAlignedBox;
    } else if (geometry_mode == "composite") {
      config_.geometry_mode = GeometryMode::kComposite;
    } else if (config_.fail_open) {
      config_.geometry_complete = false;
      degraded_reason_ = "invalid_geometry_mode";
    } else {
      throw std::invalid_argument("geometry_mode must be legacy_aabb or composite");
    }
    config_.body_bounds = {
      declare_parameter<double>("body_min_x_m", -0.45),
      declare_parameter<double>("body_max_x_m", 0.45),
      declare_parameter<double>("body_min_y_m", -0.45),
      declare_parameter<double>("body_max_y_m", 0.45),
      declare_parameter<double>("body_min_z_m", -0.35),
      declare_parameter<double>("body_max_z_m", 0.15)};
    config_.min_range_m = declare_parameter<double>("min_range_m", 0.10);
    config_.max_range_m = declare_parameter<double>("max_range_m", 40.0);
    const auto rotation_values = declare_parameter<std::vector<double>>(
      "lidar_to_body_rotation",
      std::vector<double>(defaults.rotation.begin(), defaults.rotation.end()));
    const auto translation_values = declare_parameter<std::vector<double>>(
      "lidar_to_body_translation",
      defaults.translation ?
      std::vector<double>(defaults.translation->begin(), defaults.translation->end()) :
      std::vector<double>{0.0, 0.0, 0.0});
    try {
      config_.lidar_to_body_rotation = finite_array<9>(
        rotation_values, "lidar_to_body_rotation");
      config_.lidar_to_body_translation = finite_array<3>(
        translation_values, "lidar_to_body_translation");
    } catch (const std::exception & exception) {
      if (!config_.fail_open) {
        throw;
      }
      config_.filter_enabled = false;
      config_.geometry_complete = false;
      degraded_reason_ = std::string("invalid_mount_geometry:") + exception.what();
    }
    configure_primitives(defaults.primitives);
    profiling_enabled_ = declare_parameter<bool>("enable_profiling", false);
    if (profiling_enabled_) {
      callback_samples_ms_.reserve(kMaximumProfileSamples);
    }

    ratio_publisher_ = create_publisher<std_msgs::msg::Float32>(
      "/sensors/lidar/body_removed_ratio", rclcpp::SensorDataQoS());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/sensors/lidar/body_filter_status", rclcpp::QoS(1).transient_local());
    if (input_message_type == "pointcloud2") {
      cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic, rclcpp::SensorDataQoS());
      cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&PointCloudBodyFilterNode::on_cloud, this, std::placeholders::_1));
    } else if (input_message_type == "livox_custom") {
      livox_publisher_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
        output_topic, rclcpp::SensorDataQoS());
      livox_subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        input_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&PointCloudBodyFilterNode::on_livox_cloud, this, std::placeholders::_1));
    } else {
      throw std::invalid_argument("input_message_type must be pointcloud2 or livox_custom");
    }
    report_timer_ = create_wall_timer(
      std::chrono::seconds(5), std::bind(&PointCloudBodyFilterNode::report, this));
    RCLCPP_INFO(
      get_logger(),
      "C++ body filter: %s -> %s type=%s enabled=%s geometry_complete=%s mode=%s "
      "body-frame bounds [%.3f, %.3f] [%.3f, %.3f] [%.3f, %.3f]",
      input_topic.c_str(), output_topic.c_str(), input_message_type.c_str(),
      config_.filter_enabled ? "true" : "false",
      config_.geometry_complete ? "true" : "false", geometry_mode.c_str(),
      config_.body_bounds[0], config_.body_bounds[1],
      config_.body_bounds[2], config_.body_bounds[3],
      config_.body_bounds[4], config_.body_bounds[5]);
  }

private:
  void configure_primitives(const std::vector<BodyPrimitive> & default_primitives)
  {
    std::vector<std::string> default_types;
    std::vector<std::string> default_names;
    std::vector<double> default_centers;
    std::vector<double> default_sizes;
    std::vector<double> default_rotations;
    std::vector<double> default_paddings;
    for (const auto & primitive : default_primitives) {
      default_types.push_back(primitive.type == PrimitiveType::kBox ? "box" : "cylinder");
      default_names.push_back(primitive.name);
      default_centers.insert(
        default_centers.end(), primitive.center_body_m.begin(), primitive.center_body_m.end());
      default_sizes.insert(default_sizes.end(), primitive.size_m.begin(), primitive.size_m.end());
      default_rotations.insert(
        default_rotations.end(), primitive.body_from_primitive_rotation.begin(),
        primitive.body_from_primitive_rotation.end());
      default_paddings.push_back(primitive.padding_m);
    }
    declare_parameter("primitive_types", rclcpp::ParameterValue(default_types));
    declare_parameter("primitive_names", rclcpp::ParameterValue(default_names));
    declare_parameter("primitive_centers_m", rclcpp::ParameterValue(default_centers));
    declare_parameter("primitive_sizes_m", rclcpp::ParameterValue(default_sizes));
    declare_parameter("primitive_rotations", rclcpp::ParameterValue(default_rotations));
    declare_parameter("primitive_paddings_m", rclcpp::ParameterValue(default_paddings));
    const auto types = get_parameter("primitive_types").as_string_array();
    const auto names = get_parameter("primitive_names").as_string_array();
    const auto centers = get_parameter("primitive_centers_m").as_double_array();
    const auto sizes = get_parameter("primitive_sizes_m").as_double_array();
    const auto rotations = get_parameter("primitive_rotations").as_double_array();
    const auto paddings = get_parameter("primitive_paddings_m").as_double_array();
    if (config_.geometry_mode != GeometryMode::kComposite || !config_.geometry_complete) {
      return;
    }
    const std::size_t count = types.size();
    const bool valid_lengths = count > 0U && names.size() == count &&
      centers.size() == count * 3U && sizes.size() == count * 3U &&
      rotations.size() == count * 9U && paddings.size() == count;
    if (!valid_lengths) {
      if (config_.fail_open) {
        config_.geometry_complete = false;
        degraded_reason_ = "incomplete_composite_geometry";
        return;
      }
      throw std::invalid_argument("composite primitive arrays have inconsistent lengths");
    }
    config_.body_primitives.reserve(count);
    for (std::size_t index = 0U; index < count; ++index) {
      BodyPrimitive primitive;
      primitive.name = names[index];
      if (types[index] == "box") {
        primitive.type = PrimitiveType::kBox;
      } else if (types[index] == "cylinder") {
        primitive.type = PrimitiveType::kCylinder;
      } else if (config_.fail_open) {
        config_.geometry_complete = false;
        degraded_reason_ = "invalid_primitive_type";
        config_.body_primitives.clear();
        return;
      } else {
        throw std::invalid_argument("primitive type must be box or cylinder");
      }
      std::copy_n(centers.begin() + static_cast<std::ptrdiff_t>(index * 3U), 3U,
        primitive.center_body_m.begin());
      std::copy_n(sizes.begin() + static_cast<std::ptrdiff_t>(index * 3U), 3U,
        primitive.size_m.begin());
      std::copy_n(rotations.begin() + static_cast<std::ptrdiff_t>(index * 9U), 9U,
        primitive.body_from_primitive_rotation.begin());
      primitive.padding_m = paddings[index];
      config_.body_primitives.push_back(std::move(primitive));
    }
  }

  void publish_frame_status(const std::size_t removed_body, const std::size_t total,
    const bool degraded_fail_open)
  {
    ++frames_;
    removed_body_ += removed_body;
    input_points_ += total;
    std_msgs::msg::Float32 ratio;
    ratio.data = static_cast<float>(removed_body) /
      static_cast<float>(std::max<std::size_t>(1U, total));
    ratio_publisher_->publish(ratio);
    std_msgs::msg::String status;
    status.data = degraded_fail_open ?
      "DEGRADED_FAIL_OPEN:" + (degraded_reason_.empty() ? "geometry_incomplete" : degraded_reason_) :
      (config_.filter_enabled ? "ACTIVE" : "BYPASS_DISABLED");
    status_publisher_->publish(status);
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr input)
  {
    const auto callback_start = std::chrono::steady_clock::now();
    try {
      FilterResult result = filter_cloud(*input, config_);
      publish_frame_status(result.removed_body, result.total, result.degraded_fail_open);
      cloud_publisher_->publish(std::move(result.cloud));
    } catch (const std::exception & exception) {
      if (!config_.fail_open) {
        RCLCPP_ERROR(get_logger(), "%s", exception.what());
        return;
      }
      degraded_reason_ = std::string("internal_exception:") + exception.what();
      publish_frame_status(0U, input->point_step == 0U ? 0U :
        input->data.size() / input->point_step, true);
      cloud_publisher_->publish(*input);
    }
    record_profile(callback_start);
  }

  void on_livox_cloud(const livox_ros_driver2::msg::CustomMsg::SharedPtr input)
  {
    const auto callback_start = std::chrono::steady_clock::now();
    try {
      LivoxFilterResult result = filter_livox_cloud(*input, config_);
      publish_frame_status(result.removed_body, result.total, result.degraded_fail_open);
      livox_publisher_->publish(std::move(result.cloud));
    } catch (const std::exception & exception) {
      if (!config_.fail_open) {
        RCLCPP_ERROR(get_logger(), "%s", exception.what());
        return;
      }
      degraded_reason_ = std::string("internal_exception:") + exception.what();
      publish_frame_status(0U, input->points.size(), true);
      livox_publisher_->publish(*input);
    }
    record_profile(callback_start);
  }

  void record_profile(const std::chrono::steady_clock::time_point callback_start)
  {
    if (profiling_enabled_) {
      const double callback_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - callback_start).count();
      if (callback_samples_ms_.size() < kMaximumProfileSamples) {
        callback_samples_ms_.push_back(callback_ms);
      } else {
        callback_samples_ms_[profile_write_index_] = callback_ms;
        profile_write_index_ = (profile_write_index_ + 1U) % kMaximumProfileSamples;
      }
    }
  }

  void report()
  {
    const double ratio = static_cast<double>(removed_body_) /
      static_cast<double>(std::max<std::uint64_t>(1U, input_points_));
    if (profiling_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "body_filter frames=%lu input_points=%lu removed_body=%lu ratio=%.5f "
        "callback_samples=%zu callback_p50_ms=%.6f callback_p95_ms=%.6f "
        "callback_max_ms=%.6f",
        static_cast<unsigned long>(frames_), static_cast<unsigned long>(input_points_),
        static_cast<unsigned long>(removed_body_), ratio, callback_samples_ms_.size(),
        percentile(callback_samples_ms_, 0.50), percentile(callback_samples_ms_, 0.95),
        callback_samples_ms_.empty() ? 0.0 :
        *std::max_element(callback_samples_ms_.begin(), callback_samples_ms_.end()));
    } else {
      RCLCPP_INFO(
        get_logger(), "body_filter frames=%lu input_points=%lu removed_body=%lu ratio=%.5f",
        static_cast<unsigned long>(frames_), static_cast<unsigned long>(input_points_),
        static_cast<unsigned long>(removed_body_), ratio);
    }
  }

  static constexpr std::size_t kMaximumProfileSamples = 10000U;
  FilterConfig config_;
  bool profiling_enabled_{false};
  std::string degraded_reason_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ratio_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_subscription_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  std::uint64_t frames_{0U};
  std::uint64_t removed_body_{0U};
  std::uint64_t input_points_{0U};
  std::vector<double> callback_samples_ms_;
  std::size_t profile_write_index_{0U};
};

}  // namespace uf_pointcloud_body_filter_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<uf_pointcloud_body_filter_cpp::PointCloudBodyFilterNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("pointcloud_body_filter"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
