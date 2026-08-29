#include "uf_pointcloud_body_filter_cpp/pointcloud_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace uf_pointcloud_body_filter_cpp
{
namespace
{

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
    const std::string input_topic = declare_parameter<std::string>(
      "input_topic", "/sim/mid360/points_raw");
    const std::string output_topic = declare_parameter<std::string>(
      "output_topic", "/sensors/lidar/points_body_filtered");
    config_.body_bounds = {
      declare_parameter<double>("body_min_x_m", -0.45),
      declare_parameter<double>("body_max_x_m", 0.45),
      declare_parameter<double>("body_min_y_m", -0.45),
      declare_parameter<double>("body_max_y_m", 0.45),
      declare_parameter<double>("body_min_z_m", -0.35),
      declare_parameter<double>("body_max_z_m", 0.15)};
    config_.min_range_m = declare_parameter<double>("min_range_m", 0.10);
    config_.max_range_m = declare_parameter<double>("max_range_m", 40.0);
    config_.lidar_to_body_rotation = finite_array<9>(
      declare_parameter<std::vector<double>>(
        "lidar_to_body_rotation",
        {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}),
      "lidar_to_body_rotation");
    config_.lidar_to_body_translation = finite_array<3>(
      declare_parameter<std::vector<double>>(
        "lidar_to_body_translation", {0.0, 0.0, 0.0}),
      "lidar_to_body_translation");
    profiling_enabled_ = declare_parameter<bool>("enable_profiling", false);
    if (profiling_enabled_) {
      callback_samples_ms_.reserve(kMaximumProfileSamples);
    }

    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::SensorDataQoS());
    ratio_publisher_ = create_publisher<std_msgs::msg::Float32>(
      "/sensors/lidar/body_removed_ratio", rclcpp::SensorDataQoS());
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&PointCloudBodyFilterNode::on_cloud, this, std::placeholders::_1));
    report_timer_ = create_wall_timer(
      std::chrono::seconds(5), std::bind(&PointCloudBodyFilterNode::report, this));
    RCLCPP_INFO(
      get_logger(),
      "C++ body filter active: %s -> %s, body-frame bounds "
      "[%.3f, %.3f] [%.3f, %.3f] [%.3f, %.3f]",
      input_topic.c_str(), output_topic.c_str(),
      config_.body_bounds[0], config_.body_bounds[1],
      config_.body_bounds[2], config_.body_bounds[3],
      config_.body_bounds[4], config_.body_bounds[5]);
  }

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr input)
  {
    const auto callback_start = std::chrono::steady_clock::now();
    try {
      FilterResult result = filter_cloud(*input, config_);
      ++frames_;
      removed_body_ += result.removed_body;
      input_points_ += result.total;
      std_msgs::msg::Float32 ratio;
      ratio.data = static_cast<float>(result.removed_body) /
        static_cast<float>(std::max<std::size_t>(1U, result.total));
      ratio_publisher_->publish(ratio);
      cloud_publisher_->publish(std::move(result.cloud));
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "%s", exception.what());
      return;
    }
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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ratio_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
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
