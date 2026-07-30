#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{
using Clock = std::chrono::steady_clock;

int64_t stamp_ns(const builtin_interfaces::msg::Time &_stamp)
{
  return static_cast<int64_t>(_stamp.sec) * 1000000000LL + _stamp.nanosec;
}

double percentile(std::vector<double> _values, double _q)
{
  if (_values.empty()) return 0.0;
  std::sort(_values.begin(), _values.end());
  const double position = _q * static_cast<double>(_values.size() - 1);
  const auto low = static_cast<std::size_t>(std::floor(position));
  const auto high = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(low);
  return _values[low] * (1.0 - fraction) + _values[high] * fraction;
}

double mean(const std::vector<double> &_values)
{
  return _values.empty() ? 0.0 :
    std::accumulate(_values.begin(), _values.end(), 0.0) /
    static_cast<double>(_values.size());
}

struct Stream
{
  uint64_t count{0};
  Clock::time_point first{};
  Clock::time_point last{};
  std::vector<double> intervals_ms;
  std::map<int64_t, Clock::time_point> stamps;

  void update(int64_t _stamp)
  {
    const auto now = Clock::now();
    if (count == 0) first = now;
    else intervals_ms.push_back(
      std::chrono::duration<double, std::milli>(now - last).count());
    last = now;
    stamps[_stamp] = now;
    ++count;
  }

  double overall_hz() const
  {
    if (count < 2) return 0.0;
    return static_cast<double>(count - 1) /
      std::chrono::duration<double>(last - first).count();
  }

  std::vector<double> rates() const
  {
    std::vector<double> result;
    result.reserve(intervals_ms.size());
    for (const double interval : intervals_ms)
      if (interval > 0.0) result.push_back(1000.0 / interval);
    return result;
  }
};
}  // namespace

class RosRgbdRateProbe : public rclcpp::Node
{
public:
  RosRgbdRateProbe()
  : Node("ros_rgbd_rate_probe")
  {
    const std::string color_topic = declare_parameter<std::string>(
      "color_topic", "/front/d435i/color/image_raw");
    const std::string depth_topic = declare_parameter<std::string>(
      "depth_topic", "/front/d435i/aligned_depth_to_color/image_raw");
    const std::string info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/front/d435i/color/camera_info");
    const std::string reliability = declare_parameter<std::string>(
      "qos_reliability", "best_effort");
    const int depth = std::max(
      1, static_cast<int>(declare_parameter<int64_t>("qos_depth", 5)));
    validate_depth_ = declare_parameter<bool>("validate_depth", true);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(depth)).durability_volatile();
    if (reliability == "reliable") qos.reliable();
    else qos.best_effort();
    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
      color_topic, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        color_.update(stamp_ns(message->header.stamp));
        if (color_.count == 1)
        {
          color_width_ = message->width;
          color_height_ = message->height;
          color_step_ = message->step;
          color_encoding_ = message->encoding;
          color_frame_ = message->header.frame_id;
        }
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        depth_.update(stamp_ns(message->header.stamp));
        if (depth_.count == 1)
        {
          depth_width_ = message->width;
          depth_height_ = message->height;
          depth_step_ = message->step;
          depth_encoding_ = message->encoding;
          depth_frame_ = message->header.frame_id;
          depth_bigendian_ = message->is_bigendian;
        }
        if (validate_depth_ && depth_validation_frames_ < 3)
          validate_depth(*message);
      });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        if (have_info_) return;
        info_width_ = message->width;
        info_height_ = message->height;
        info_frame_ = message->header.frame_id;
        info_fx_ = message->k[0];
        info_fy_ = message->k[4];
        info_cx_ = message->k[2];
        info_cy_ = message->k[5];
        have_info_ = true;
      });
  }

  void write(const std::string &_path)
  {
    std::ostream *output = &std::cout;
    std::ofstream file;
    if (!_path.empty())
    {
      file.open(_path);
      output = &file;
    }
    const auto color_rates = color_.rates();
    const auto depth_rates = depth_.rates();
    std::size_t matches = 0;
    std::vector<double> delivery_skew_ms;
    for (const auto &[stamp, arrival] : depth_.stamps)
    {
      const auto found = color_.stamps.find(stamp);
      if (found == color_.stamps.end()) continue;
      ++matches;
      delivery_skew_ms.push_back(std::abs(
        std::chrono::duration<double, std::milli>(arrival - found->second).count()));
    }
    const std::size_t failures =
      color_.stamps.size() + depth_.stamps.size() - 2 * matches;
    const double exact_fraction = depth_.stamps.empty() ? 0.0 :
      static_cast<double>(matches) / static_cast<double>(depth_.stamps.size());
    std::sort(valid_depth_values_.begin(), valid_depth_values_.end());
    const double valid_ratio = depth_values_seen_ == 0 ? 0.0 :
      static_cast<double>(valid_depth_values_.size()) /
      static_cast<double>(depth_values_seen_);

    *output << std::fixed << std::setprecision(6)
      << "color_count=" << color_.count << '\n'
      << "depth_count=" << depth_.count << '\n'
      << "exact_pair_count=" << matches << '\n'
      << "exact_match_fraction=" << exact_fraction << '\n'
      << "exact_sync_failure_messages=" << failures << '\n'
      << "delivery_skew_mean_ms=" << mean(delivery_skew_ms) << '\n'
      << "delivery_skew_p95_ms=" << percentile(delivery_skew_ms, 0.95) << '\n';
    write_stream(*output, "color", color_, color_rates);
    write_stream(*output, "depth", depth_, depth_rates);
    *output
      << "color_width=" << color_width_ << '\n'
      << "color_height=" << color_height_ << '\n'
      << "color_step=" << color_step_ << '\n'
      << "color_encoding=" << color_encoding_ << '\n'
      << "color_frame=" << color_frame_ << '\n'
      << "depth_width=" << depth_width_ << '\n'
      << "depth_height=" << depth_height_ << '\n'
      << "depth_step=" << depth_step_ << '\n'
      << "depth_encoding=" << depth_encoding_ << '\n'
      << "depth_frame=" << depth_frame_ << '\n'
      << "depth_bigendian=" << static_cast<int>(depth_bigendian_) << '\n'
      << "depth_validation_frames=" << depth_validation_frames_ << '\n'
      << "depth_valid_ratio=" << valid_ratio << '\n'
      << "depth_zero_count=" << zero_count_ << '\n'
      << "depth_nan_count=" << nan_count_ << '\n'
      << "depth_inf_count=" << inf_count_ << '\n'
      << "depth_valid_min_m=" << (valid_depth_values_.empty() ? 0.0 : valid_depth_values_.front()) << '\n'
      << "depth_valid_median_m=" << percentile(valid_depth_values_, 0.5) << '\n'
      << "depth_valid_max_m=" << (valid_depth_values_.empty() ? 0.0 : valid_depth_values_.back()) << '\n'
      << "depth_center_m=" << center_depth_m_ << '\n'
      << "camera_info_width=" << info_width_ << '\n'
      << "camera_info_height=" << info_height_ << '\n'
      << "camera_info_frame=" << info_frame_ << '\n'
      << "camera_info_fx=" << info_fx_ << '\n'
      << "camera_info_fy=" << info_fy_ << '\n'
      << "camera_info_cx=" << info_cx_ << '\n'
      << "camera_info_cy=" << info_cy_ << '\n';
  }

private:
  static void write_stream(
    std::ostream &_output, const std::string &_name,
    const Stream &_stream, const std::vector<double> &_rates)
  {
    const double longest = _stream.intervals_ms.empty() ? 0.0 :
      *std::max_element(_stream.intervals_ms.begin(), _stream.intervals_ms.end());
    _output
      << _name << "_mean_hz=" << _stream.overall_hz() << '\n'
      << _name << "_median_hz=" << percentile(_rates, 0.5) << '\n'
      << _name << "_p05_hz=" << percentile(_rates, 0.05) << '\n'
      << _name << "_p95_hz=" << percentile(_rates, 0.95) << '\n'
      << _name << "_min_hz=" << (_rates.empty() ? 0.0 : *std::min_element(_rates.begin(), _rates.end())) << '\n'
      << _name << "_max_hz=" << (_rates.empty() ? 0.0 : *std::max_element(_rates.begin(), _rates.end())) << '\n'
      << _name << "_longest_interval_ms=" << longest << '\n';
  }

  void validate_depth(const sensor_msgs::msg::Image &_message)
  {
    const std::size_t pixels = static_cast<std::size_t>(_message.width) * _message.height;
    if (_message.encoding == "16UC1" && _message.data.size() >= pixels * 2)
    {
      for (std::size_t index = 0; index < pixels; ++index)
      {
        uint16_t millimeters = 0;
        std::memcpy(&millimeters, _message.data.data() + index * 2, 2);
        ++depth_values_seen_;
        if (millimeters == 0) ++zero_count_;
        else valid_depth_values_.push_back(static_cast<double>(millimeters) * 0.001);
      }
      uint16_t center = 0;
      std::memcpy(&center, _message.data.data() + (pixels / 2) * 2, 2);
      center_depth_m_ = static_cast<double>(center) * 0.001;
      ++depth_validation_frames_;
    }
    else if (_message.encoding == "32FC1" && _message.data.size() >= pixels * 4)
    {
      for (std::size_t index = 0; index < pixels; ++index)
      {
        float value = 0.0F;
        std::memcpy(&value, _message.data.data() + index * 4, 4);
        ++depth_values_seen_;
        if (std::isnan(value)) ++nan_count_;
        else if (std::isinf(value)) ++inf_count_;
        else if (value == 0.0F) ++zero_count_;
        else valid_depth_values_.push_back(value);
      }
      float center = 0.0F;
      std::memcpy(&center, _message.data.data() + (pixels / 2) * 4, 4);
      center_depth_m_ = center;
      ++depth_validation_frames_;
    }
  }

  bool validate_depth_{true};
  bool have_info_{false};
  Stream color_;
  Stream depth_;
  uint32_t color_width_{0};
  uint32_t color_height_{0};
  uint32_t color_step_{0};
  uint32_t depth_width_{0};
  uint32_t depth_height_{0};
  uint32_t depth_step_{0};
  uint32_t info_width_{0};
  uint32_t info_height_{0};
  uint8_t depth_bigendian_{0};
  std::string color_encoding_;
  std::string color_frame_;
  std::string depth_encoding_;
  std::string depth_frame_;
  std::string info_frame_;
  double info_fx_{0.0};
  double info_fy_{0.0};
  double info_cx_{0.0};
  double info_cy_{0.0};
  std::size_t depth_validation_frames_{0};
  std::size_t depth_values_seen_{0};
  uint64_t zero_count_{0};
  uint64_t nan_count_{0};
  uint64_t inf_count_{0};
  double center_depth_m_{0.0};
  std::vector<double> valid_depth_values_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosRgbdRateProbe>();
  double duration = 30.0;
  std::string output_path;
  for (int index = 1; index < argc; ++index)
  {
    const std::string argument(argv[index]);
    if (argument == "--duration" && index + 1 < argc)
      duration = std::stod(argv[++index]);
    else if (argument == "--output" && index + 1 < argc)
      output_path = argv[++index];
  }
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  const auto finish = Clock::now() + std::chrono::duration<double>(duration);
  while (rclcpp::ok() && Clock::now() < finish)
    executor.spin_some(std::chrono::milliseconds(50));
  executor.remove_node(node);
  node->write(output_path);
  rclcpp::shutdown();
  return 0;
}
