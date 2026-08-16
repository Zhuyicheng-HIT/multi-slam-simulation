
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/transport/Node.hh>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace
{
using SteadyClock = std::chrono::steady_clock;

int64_t stamp_ns(const gz::msgs::Header &_header)
{
  return _header.stamp().sec() * 1000000000LL + _header.stamp().nsec();
}

int64_t stamp_ns(const builtin_interfaces::msg::Time &_stamp)
{
  return static_cast<int64_t>(_stamp.sec) * 1000000000LL + _stamp.nanosec;
}

builtin_interfaces::msg::Time ros_stamp(int64_t _stamp_ns)
{
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(_stamp_ns / 1000000000LL);
  result.nanosec = static_cast<uint32_t>(_stamp_ns % 1000000000LL);
  return result;
}

double percentile(std::vector<double> _values, double _q)
{
  if (_values.empty())
    return 0.0;
  std::sort(_values.begin(), _values.end());
  const double position = _q * static_cast<double>(_values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  if (lower == upper)
    return _values[lower];
  const double fraction = position - static_cast<double>(lower);
  return _values[lower] * (1.0 - fraction) + _values[upper] * fraction;
}

double mean(const std::vector<double> &_values)
{
  if (_values.empty())
    return 0.0;
  return std::accumulate(_values.begin(), _values.end(), 0.0) /
         static_cast<double>(_values.size());
}

std::string wall_time_iso8601()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t raw = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&raw, &local);
  std::ostringstream stream;
  stream << std::put_time(&local, "%Y-%m-%dT%H:%M:%S%z");
  return stream.str();
}
}  // namespace

class D435iRgbdBridge : public rclcpp::Node
{
public:
  D435iRgbdBridge()
  : Node("d435i_rgbd_bridge_cpp"), stats_window_start_(SteadyClock::now())
  {
    gz_prefix_ = declare_parameter<std::string>("gz_prefix", "/front/d435i/gz");
    ros_prefix_ = declare_parameter<std::string>("ros_prefix", "/front/d435i");
    camera_link_frame_ = declare_parameter<std::string>(
      "camera_link_frame", "d435i_link");
    mode_ = declare_parameter<std::string>("mode", "cpp");
    depth_encoding_ = declare_parameter<std::string>("depth_encoding", "16UC1");
    max_depth_m_ = declare_parameter<double>("max_depth_m", 6.0);
    min_depth_m_ = declare_parameter<double>("min_depth_m", 0.3);
    sync_queue_depth_ = std::max(
      1, static_cast<int>(declare_parameter<int64_t>("sync_queue_depth", 2)));
    qos_depth_ = std::max(
      1, static_cast<int>(declare_parameter<int64_t>("qos_depth", 1)));
    qos_reliability_ = declare_parameter<std::string>("qos_reliability", "best_effort");
    enable_pointcloud_ = declare_parameter<bool>("enable_pointcloud", false);
    pointcloud_hz_ = std::max(0.1, declare_parameter<double>("pointcloud_hz", 10.0));
    pointcloud_stride_ = std::max(
      1, static_cast<int>(declare_parameter<int64_t>("pointcloud_stride", 4)));
    performance_stats_enabled_ =
      declare_parameter<bool>("performance_stats_enabled", false);
    performance_stats_period_s_ =
      std::max(1.0, declare_parameter<double>("performance_stats_period_s", 5.0));
    performance_csv_path_ =
      declare_parameter<std::string>("performance_csv_path", "");

    if (depth_encoding_ != "16UC1" && depth_encoding_ != "32FC1")
      throw std::invalid_argument("depth_encoding must be 16UC1 or 32FC1");
    if (mode_ != "cpp" && mode_ != "hybrid")
      throw std::invalid_argument("mode must be cpp or hybrid");
    use_gz_color_payload_ = mode_ == "cpp";

    auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth_)).durability_volatile();
    if (qos_reliability_ == "reliable")
      qos.reliable();
    else if (qos_reliability_ == "best_effort")
      qos.best_effort();
    else
      throw std::invalid_argument("qos_reliability must be best_effort or reliable");

    color_pub_ = create_publisher<sensor_msgs::msg::Image>(
      ros_prefix_ + "/color/image_raw", qos);
    if (mode_ == "hybrid")
    {
      const std::string hybrid_color_topic = declare_parameter<std::string>(
        "hybrid_color_topic", ros_prefix_ + "/transport/color_raw");
      hybrid_color_sub_ = create_subscription<sensor_msgs::msg::Image>(
        hybrid_color_topic, qos,
        std::bind(&D435iRgbdBridge::hybrid_color_callback, this, std::placeholders::_1));
    }
    color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      ros_prefix_ + "/color/camera_info", qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
      ros_prefix_ + "/depth/image_rect_raw", qos);
    aligned_pub_ = create_publisher<sensor_msgs::msg::Image>(
      ros_prefix_ + "/aligned_depth_to_color/image_raw", qos);
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      ros_prefix_ + "/depth/camera_info", qos);
    if (enable_pointcloud_)
      points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        ros_prefix_ + "/depth/color/points", qos);
    gyro_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      ros_prefix_ + "/gyro/sample", qos);
    accel_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      ros_prefix_ + "/accel/sample", qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      ros_prefix_ + "/imu", qos);
    frame_tracking_pub_ = create_publisher<std_msgs::msg::String>(
      ros_prefix_ + "/transport/frame_tracking", rclcpp::QoS(10).reliable());

    initialize_camera_info();
    initialize_pointcloud();
    initialize_stats_csv();
    publish_static_tf();

    color_topic_ = gz_prefix_ + "/image";
    depth_topic_ = gz_prefix_ + "/depth_image";
    info_topic_ = gz_prefix_ + "/camera_info";
    imu_topic_ = gz_prefix_ + "/imu";
    const bool color_ok = gz_node_.Subscribe(
      color_topic_, &D435iRgbdBridge::color_callback, this);
    const bool depth_ok = gz_node_.Subscribe(
      depth_topic_, &D435iRgbdBridge::depth_callback, this);
    const bool info_ok = gz_node_.Subscribe(
      info_topic_, &D435iRgbdBridge::info_callback, this);
    const bool imu_ok = gz_node_.Subscribe(
      imu_topic_, &D435iRgbdBridge::imu_callback, this);
    if (!color_ok || !depth_ok || !info_ok || !imu_ok)
      throw std::runtime_error("failed to subscribe to one or more Gazebo D435i topics");

    worker_ = std::thread(&D435iRgbdBridge::worker_loop, this);
    if (performance_stats_enabled_)
    {
      stats_timer_ = create_wall_timer(
        std::chrono::duration<double>(performance_stats_period_s_),
        std::bind(&D435iRgbdBridge::report_performance, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "D435i C++ bridge active: mode=%s depth=%s qos=%s/%d sync_queue=%d",
      mode_.c_str(), depth_encoding_.c_str(), qos_reliability_.c_str(), qos_depth_,
      sync_queue_depth_);
  }

  ~D435iRgbdBridge() override
  {
    {
      std::lock_guard<std::mutex> lock(pair_mutex_);
      stop_worker_ = true;
    }
    pair_cv_.notify_all();
    if (worker_.joinable())
      worker_.join();
    gz_node_.Unsubscribe(color_topic_);
    gz_node_.Unsubscribe(depth_topic_);
    gz_node_.Unsubscribe(info_topic_);
    gz_node_.Unsubscribe(imu_topic_);
  }

private:
  struct ImageToken
  {
    int64_t stamp{0};
    uint64_t sequence{0};
    SteadyClock::time_point arrival;
    std::shared_ptr<gz::msgs::Image> image;
  };

  struct Pair
  {
    ImageToken color;
    ImageToken depth;
    SteadyClock::time_point matched;
  };

  struct WindowStats
  {
    uint64_t color_callbacks{0};
    uint64_t depth_callbacks{0};
    uint64_t matched_pairs{0};
    uint64_t published_pairs{0};
    uint64_t color_unmatched_drops{0};
    uint64_t depth_unmatched_drops{0};
    uint64_t ready_pair_overwrites{0};
    uint64_t conversion_errors{0};
    uint64_t color_sequence_gaps{0};
    uint64_t depth_sequence_gaps{0};
    std::vector<double> process_ms;
    std::vector<double> color_ms;
    std::vector<double> depth_ms;
    std::vector<double> publish_ms;
    std::vector<double> queue_ms;
    std::vector<double> publish_intervals_ms;
  };

  void color_callback(const gz::msgs::Image &_message)
  {
    ImageToken token;
    token.stamp = stamp_ns(_message.header());
    token.arrival = SteadyClock::now();
    {
      std::lock_guard<std::mutex> lock(pair_mutex_);
      token.sequence = ++color_sequence_;
      if (use_gz_color_payload_)
        token.image = std::make_shared<gz::msgs::Image>(_message);
      color_queue_.push_back(std::move(token));
      trim_queue_locked(color_queue_, true);
      match_locked();
    }
    increment_stat(&WindowStats::color_callbacks);
  }

  void depth_callback(const gz::msgs::Image &_message)
  {
    ImageToken token;
    token.stamp = stamp_ns(_message.header());
    token.arrival = SteadyClock::now();
    token.image = std::make_shared<gz::msgs::Image>(_message);
    {
      std::lock_guard<std::mutex> lock(pair_mutex_);
      token.sequence = ++depth_sequence_;
      depth_queue_.push_back(std::move(token));
      trim_queue_locked(depth_queue_, false);
      match_locked();
    }
    increment_stat(&WindowStats::depth_callbacks);
  }

  void hybrid_color_callback(sensor_msgs::msg::Image::ConstSharedPtr _message)
  {
    const int64_t stamp = stamp_ns(_message->header.stamp);
    {
      std::lock_guard<std::mutex> lock(hybrid_color_mutex_);
      hybrid_colors_[stamp] = std::move(_message);
      hybrid_color_order_.push_back(stamp);
      while (hybrid_color_order_.size() > 5)
      {
        hybrid_colors_.erase(hybrid_color_order_.front());
        hybrid_color_order_.pop_front();
      }
    }
    hybrid_color_cv_.notify_all();
  }

  sensor_msgs::msg::Image::ConstSharedPtr wait_for_hybrid_color(int64_t _stamp)
  {
    std::unique_lock<std::mutex> lock(hybrid_color_mutex_);
    hybrid_color_cv_.wait_for(lock, std::chrono::milliseconds(50), [this, _stamp]() {
      return hybrid_colors_.find(_stamp) != hybrid_colors_.end();
    });
    const auto found = hybrid_colors_.find(_stamp);
    if (found == hybrid_colors_.end())
      return nullptr;
    auto result = found->second;
    hybrid_colors_.erase(found);
    const auto order_found = std::find(
      hybrid_color_order_.begin(), hybrid_color_order_.end(), _stamp);
    if (order_found != hybrid_color_order_.end())
      hybrid_color_order_.erase(order_found);
    return result;
  }

  void trim_queue_locked(std::deque<ImageToken> &_queue, bool _color)
  {
    while (static_cast<int>(_queue.size()) > sync_queue_depth_)
    {
      _queue.pop_front();
      std::lock_guard<std::mutex> stats_lock(stats_mutex_);
      if (_color)
        ++stats_.color_unmatched_drops;
      else
        ++stats_.depth_unmatched_drops;
    }
  }

  void match_locked()
  {
    for (auto color_it = color_queue_.begin(); color_it != color_queue_.end(); ++color_it)
    {
      const auto depth_it = std::find_if(
        depth_queue_.begin(), depth_queue_.end(),
        [color_it](const ImageToken &_depth) {return _depth.stamp == color_it->stamp;});
      if (depth_it == depth_queue_.end())
        continue;

      Pair pair;
      pair.color = std::move(*color_it);
      pair.depth = std::move(*depth_it);
      pair.matched = SteadyClock::now();
      const auto color_drops = static_cast<uint64_t>(
        std::distance(color_queue_.begin(), color_it));
      const auto depth_drops = static_cast<uint64_t>(
        std::distance(depth_queue_.begin(), depth_it));
      color_queue_.erase(color_queue_.begin(), std::next(color_it));
      depth_queue_.erase(depth_queue_.begin(), std::next(depth_it));
      {
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.color_unmatched_drops += color_drops;
        stats_.depth_unmatched_drops += depth_drops;
        ++stats_.matched_pairs;
        if (ready_pair_)
          ++stats_.ready_pair_overwrites;
      }
      ready_pair_ = std::move(pair);
      pair_cv_.notify_one();
      return;
    }
  }

  void worker_loop()
  {
    while (true)
    {
      Pair pair;
      {
        std::unique_lock<std::mutex> lock(pair_mutex_);
        pair_cv_.wait(lock, [this]() {return stop_worker_ || ready_pair_.has_value();});
        if (stop_worker_)
          return;
        pair = std::move(*ready_pair_);
        ready_pair_.reset();
      }
      process_pair(pair);
    }
  }

  static std::string color_encoding(gz::msgs::PixelFormatType _format)
  {
    switch (_format)
    {
      case gz::msgs::L_INT8: return "mono8";
      case gz::msgs::L_INT16: return "mono16";
      case gz::msgs::RGB_INT8: return "rgb8";
      case gz::msgs::RGBA_INT8: return "rgba8";
      case gz::msgs::BGRA_INT8: return "bgra8";
      case gz::msgs::BGR_INT8: return "bgr8";
      default: return "";
    }
  }

  void process_pair(const Pair &_pair)
  {
    const auto process_start = SteadyClock::now();
    try
    {
      const auto stamp = ros_stamp(_pair.color.stamp);
      double color_ms = 0.0;
      if (mode_ == "cpp")
      {
        const auto color_start = SteadyClock::now();
        if (!_pair.color.image)
          throw std::runtime_error("missing color payload in cpp mode");
        const auto &source = *_pair.color.image;
        const std::string encoding = color_encoding(source.pixel_format_type());
        if (encoding.empty())
          throw std::runtime_error("unsupported Gazebo color pixel format");
        color_out_.header.stamp = stamp;
        color_out_.header.frame_id = color_frame_;
        color_out_.height = source.height();
        color_out_.width = source.width();
        color_out_.encoding = encoding;
        color_out_.is_bigendian = false;
        color_out_.step = source.step();
        color_out_.data.resize(source.data().size());
        std::memcpy(color_out_.data.data(), source.data().data(), source.data().size());
        color_ms = elapsed_ms(color_start);
      }
      else
      {
        const auto color_start = SteadyClock::now();
        const auto source = wait_for_hybrid_color(_pair.color.stamp);
        if (!source)
          throw std::runtime_error("timed out waiting for ros_gz hybrid color frame");
        color_out_.header.stamp = stamp;
        color_out_.header.frame_id = color_frame_;
        color_out_.height = source->height;
        color_out_.width = source->width;
        color_out_.encoding = source->encoding;
        color_out_.is_bigendian = source->is_bigendian;
        color_out_.step = source->step;
        color_out_.data.resize(source->data.size());
        std::memcpy(color_out_.data.data(), source->data.data(), source->data.size());
        color_ms = elapsed_ms(color_start);
      }

      const auto depth_start = SteadyClock::now();
      convert_depth(*_pair.depth.image, stamp);
      const double depth_ms = elapsed_ms(depth_start);

      const auto publish_start = SteadyClock::now();
      color_pub_->publish(color_out_);
      depth_out_.header.frame_id = depth_frame_;
      depth_pub_->publish(depth_out_);
      depth_out_.header.frame_id = color_frame_;
      aligned_pub_->publish(depth_out_);

      sensor_msgs::msg::CameraInfo info;
      bool have_info = false;
      {
        std::lock_guard<std::mutex> lock(info_mutex_);
        if (have_camera_info_)
        {
          info = camera_info_template_;
          have_info = true;
        }
      }
      if (have_info)
      {
        info.header.stamp = stamp;
        info.header.frame_id = color_frame_;
        color_info_pub_->publish(info);
        info.header.frame_id = depth_frame_;
        depth_info_pub_->publish(info);
        maybe_publish_pointcloud(info, stamp);
      }
      publish_frame_tracking(_pair);
      const double publish_ms = elapsed_ms(publish_start);
      const double process_ms = elapsed_ms(process_start);
      const double queue_ms = std::chrono::duration<double, std::milli>(
        process_start - _pair.matched).count();

      std::lock_guard<std::mutex> stats_lock(stats_mutex_);
      ++stats_.published_pairs;
      if (last_published_color_sequence_ != 0 &&
          _pair.color.sequence > last_published_color_sequence_ + 1)
        stats_.color_sequence_gaps +=
          _pair.color.sequence - last_published_color_sequence_ - 1;
      if (last_published_depth_sequence_ != 0 &&
          _pair.depth.sequence > last_published_depth_sequence_ + 1)
        stats_.depth_sequence_gaps +=
          _pair.depth.sequence - last_published_depth_sequence_ - 1;
      last_published_color_sequence_ = _pair.color.sequence;
      last_published_depth_sequence_ = _pair.depth.sequence;
      stats_.process_ms.push_back(process_ms);
      stats_.color_ms.push_back(color_ms);
      stats_.depth_ms.push_back(depth_ms);
      stats_.publish_ms.push_back(publish_ms);
      stats_.queue_ms.push_back(queue_ms);
      const auto now = SteadyClock::now();
      if (last_publish_time_ != SteadyClock::time_point{})
      {
        stats_.publish_intervals_ms.push_back(
          std::chrono::duration<double, std::milli>(now - last_publish_time_).count());
      }
      last_publish_time_ = now;
    }
    catch (const std::exception &error)
    {
      increment_stat(&WindowStats::conversion_errors);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "RGB-D conversion failed: %s", error.what());
    }
  }

  void convert_depth(
    const gz::msgs::Image &_source, const builtin_interfaces::msg::Time &_stamp)
  {
    const std::size_t width = _source.width();
    const std::size_t height = _source.height();
    const std::size_t pixels = width * height;
    const bool float_input = _source.step() == width * sizeof(float);
    const bool uint16_input = _source.step() == width * sizeof(uint16_t);
    if (!float_input && !uint16_input)
      throw std::runtime_error("unsupported depth row step");
    if (_source.data().size() < static_cast<std::size_t>(_source.step()) * height)
      throw std::runtime_error("truncated Gazebo depth payload");

    depth_meters_.resize(pixels);
    if (float_input)
    {
      std::memcpy(depth_meters_.data(), _source.data().data(), pixels * sizeof(float));
    }
    else
    {
      for (std::size_t i = 0; i < pixels; ++i)
      {
        uint16_t millimeters = 0;
        std::memcpy(&millimeters, _source.data().data() + i * sizeof(uint16_t), sizeof(uint16_t));
        depth_meters_[i] = static_cast<float>(millimeters) * 0.001F;
      }
    }

    depth_out_.header.stamp = _stamp;
    depth_out_.height = static_cast<uint32_t>(height);
    depth_out_.width = static_cast<uint32_t>(width);
    depth_out_.is_bigendian = false;
    if (depth_encoding_ == "32FC1")
    {
      depth_out_.encoding = "32FC1";
      depth_out_.step = static_cast<uint32_t>(width * sizeof(float));
      depth_out_.data.resize(pixels * sizeof(float));
      auto *destination = depth_out_.data.data();
      const float invalid = std::numeric_limits<float>::quiet_NaN();
      for (std::size_t i = 0; i < pixels; ++i)
      {
        float value = depth_meters_[i];
        if (!std::isfinite(value) || value < min_depth_m_ || value > max_depth_m_)
          value = invalid;
        depth_meters_[i] = value;
        std::memcpy(destination + i * sizeof(float), &value, sizeof(float));
      }
    }
    else
    {
      depth_out_.encoding = "16UC1";
      depth_out_.step = static_cast<uint32_t>(width * sizeof(uint16_t));
      depth_out_.data.resize(pixels * sizeof(uint16_t));
      auto *destination = depth_out_.data.data();
      for (std::size_t i = 0; i < pixels; ++i)
      {
        const float value = depth_meters_[i];
        uint16_t millimeters = 0;
        if (std::isfinite(value) && value >= min_depth_m_ && value <= max_depth_m_)
          millimeters = static_cast<uint16_t>(std::lround(value * 1000.0F));
        else
          depth_meters_[i] = std::numeric_limits<float>::quiet_NaN();
        destination[i * 2] = static_cast<uint8_t>(millimeters & 0xffU);
        destination[i * 2 + 1] = static_cast<uint8_t>((millimeters >> 8U) & 0xffU);
      }
    }
  }

  void info_callback(const gz::msgs::CameraInfo &_message)
  {
    sensor_msgs::msg::CameraInfo info;
    info.width = _message.width();
    info.height = _message.height();
    switch (_message.distortion().model())
    {
      case gz::msgs::CameraInfo::Distortion::RATIONAL_POLYNOMIAL:
        info.distortion_model = "rational_polynomial";
        break;
      case gz::msgs::CameraInfo::Distortion::EQUIDISTANT:
        info.distortion_model = "equidistant";
        break;
      default:
        info.distortion_model = "plumb_bob";
        break;
    }
    info.d.assign(_message.distortion().k().begin(), _message.distortion().k().end());
    info.k.fill(0.0);
    info.p.fill(0.0);
    info.r.fill(0.0);
    info.r[0] = info.r[4] = info.r[8] = 1.0;
    for (int i = 0; i < std::min(9, _message.intrinsics().k_size()); ++i)
      info.k[static_cast<std::size_t>(i)] = _message.intrinsics().k(i);
    for (int i = 0; i < std::min(12, _message.projection().p_size()); ++i)
      info.p[static_cast<std::size_t>(i)] = _message.projection().p(i);
    for (int i = 0; i < std::min(9, _message.rectification_matrix_size()); ++i)
      info.r[static_cast<std::size_t>(i)] = _message.rectification_matrix(i);
    std::lock_guard<std::mutex> lock(info_mutex_);
    camera_info_template_ = std::move(info);
    have_camera_info_ = true;
  }

  void imu_callback(const gz::msgs::IMU &_message)
  {
    if (!rclcpp::ok())
      return;
    sensor_msgs::msg::Imu combined;
    combined.header.stamp = ros_stamp(stamp_ns(_message.header()));
    combined.header.frame_id = imu_frame_;
    combined.orientation.x = _message.orientation().x();
    combined.orientation.y = _message.orientation().y();
    combined.orientation.z = _message.orientation().z();
    combined.orientation.w = _message.orientation().w();
    combined.angular_velocity.x = _message.angular_velocity().x();
    combined.angular_velocity.y = _message.angular_velocity().y();
    combined.angular_velocity.z = _message.angular_velocity().z();
    combined.linear_acceleration.x = _message.linear_acceleration().x();
    combined.linear_acceleration.y = _message.linear_acceleration().y();
    combined.linear_acceleration.z = _message.linear_acceleration().z();
    combined.angular_velocity_covariance = {4e-8, 0.0, 0.0, 0.0, 4e-8, 0.0, 0.0, 0.0, 4e-8};
    combined.linear_acceleration_covariance = {4e-6, 0.0, 0.0, 0.0, 4e-6, 0.0, 0.0, 0.0, 4e-6};
    imu_pub_->publish(combined);
    auto gyro = combined;
    gyro.orientation_covariance[0] = -1.0;
    gyro.linear_acceleration_covariance[0] = -1.0;
    gyro_pub_->publish(gyro);
    auto accel = combined;
    accel.orientation_covariance[0] = -1.0;
    accel.angular_velocity_covariance[0] = -1.0;
    accel_pub_->publish(accel);
  }

  void maybe_publish_pointcloud(
    const sensor_msgs::msg::CameraInfo &_info,
    const builtin_interfaces::msg::Time &_stamp)
  {
    if (!points_pub_ || points_pub_->get_subscription_count() == 0)
      return;
    const auto now = SteadyClock::now();
    if (last_cloud_time_ != SteadyClock::time_point{} &&
        std::chrono::duration<double>(now - last_cloud_time_).count() < 1.0 / pointcloud_hz_)
      return;
    if (_info.k[0] <= 0.0 || _info.k[4] <= 0.0)
      return;

    const std::size_t source_width = depth_out_.width;
    const std::size_t source_height = depth_out_.height;
    const std::size_t width = (source_width + pointcloud_stride_ - 1) / pointcloud_stride_;
    const std::size_t height = (source_height + pointcloud_stride_ - 1) / pointcloud_stride_;
    points_out_.header.stamp = _stamp;
    points_out_.header.frame_id = color_frame_;
    points_out_.width = static_cast<uint32_t>(width);
    points_out_.height = static_cast<uint32_t>(height);
    points_out_.row_step = static_cast<uint32_t>(width * points_out_.point_step);
    points_out_.data.resize(height * points_out_.row_step);
    bool all_valid = true;
    for (std::size_t y = 0; y < height; ++y)
    {
      for (std::size_t x = 0; x < width; ++x)
      {
        const std::size_t source_x = std::min(x * pointcloud_stride_, source_width - 1);
        const std::size_t source_y = std::min(y * pointcloud_stride_, source_height - 1);
        const float z = depth_meters_[source_y * source_width + source_x];
        float xyz[3];
        if (std::isfinite(z))
        {
          xyz[0] = static_cast<float>((static_cast<double>(source_x) - _info.k[2]) * z / _info.k[0]);
          xyz[1] = static_cast<float>((static_cast<double>(source_y) - _info.k[5]) * z / _info.k[4]);
          xyz[2] = z;
        }
        else
        {
          xyz[0] = xyz[1] = xyz[2] = std::numeric_limits<float>::quiet_NaN();
          all_valid = false;
        }
        const std::size_t offset = (y * width + x) * sizeof(xyz);
        std::memcpy(points_out_.data.data() + offset, xyz, sizeof(xyz));
      }
    }
    points_out_.is_dense = all_valid;
    points_pub_->publish(points_out_);
    last_cloud_time_ = now;
  }

  void initialize_camera_info()
  {
    camera_info_template_.r.fill(0.0);
    camera_info_template_.r[0] = camera_info_template_.r[4] = camera_info_template_.r[8] = 1.0;
  }

  void initialize_pointcloud()
  {
    points_out_.is_bigendian = false;
    points_out_.point_step = 12;
    points_out_.fields.resize(3);
    const std::array<std::string, 3> names{"x", "y", "z"};
    for (std::size_t i = 0; i < names.size(); ++i)
    {
      points_out_.fields[i].name = names[i];
      points_out_.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
      points_out_.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      points_out_.fields[i].count = 1;
    }
  }

  static geometry_msgs::msg::TransformStamped transform(
    const std::string &_parent, const std::string &_child,
    double _x = 0.0, double _y = 0.0, double _z = 0.0, bool _optical = false)
  {
    geometry_msgs::msg::TransformStamped result;
    result.header.frame_id = _parent;
    result.child_frame_id = _child;
    result.transform.translation.x = _x;
    result.transform.translation.y = _y;
    result.transform.translation.z = _z;
    if (_optical)
    {
      result.transform.rotation.x = -0.5;
      result.transform.rotation.y = 0.5;
      result.transform.rotation.z = -0.5;
      result.transform.rotation.w = 0.5;
    }
    else
    {
      result.transform.rotation.w = 1.0;
    }
    return result;
  }

  void publish_static_tf()
  {
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    std::vector<geometry_msgs::msg::TransformStamped> transforms{
      transform(camera_link_frame_, "front_d435i_color_frame"),
      transform("front_d435i_color_frame", color_frame_, 0.0, 0.0, 0.0, true),
      transform(camera_link_frame_, "front_d435i_depth_frame"),
      transform("front_d435i_depth_frame", depth_frame_, 0.0, 0.0, 0.0, true),
      transform(camera_link_frame_, imu_frame_)};
    const auto stamp = now();
    for (auto &item : transforms)
      item.header.stamp = stamp;
    static_tf_broadcaster_->sendTransform(transforms);
  }

  template<typename Member>
  void increment_stat(Member _member)
  {
    if (!performance_stats_enabled_)
      return;
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++(stats_.*_member);
  }

  static double elapsed_ms(const SteadyClock::time_point &_start)
  {
    return std::chrono::duration<double, std::milli>(SteadyClock::now() - _start).count();
  }

  static int64_t steady_ns(const SteadyClock::time_point &_time)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      _time.time_since_epoch()).count();
  }

  void publish_frame_tracking(const Pair &_pair)
  {
    const uint64_t pair_sequence = ++published_pair_sequence_;
    if (frame_tracking_pub_->get_subscription_count() == 0)
      return;
    const auto publish_time = SteadyClock::now();
    std_msgs::msg::String message;
    std::ostringstream data;
    data << pair_sequence << ',' << _pair.color.stamp << ','
         << steady_ns(_pair.color.arrival) << ','
         << steady_ns(_pair.depth.arrival) << ','
         << steady_ns(_pair.matched) << ','
         << steady_ns(publish_time) << ','
         << _pair.color.sequence << ',' << _pair.depth.sequence;
    message.data = data.str();
    frame_tracking_pub_->publish(message);
  }

  void initialize_stats_csv()
  {
    if (!performance_stats_enabled_ || performance_csv_path_.empty())
      return;
    const std::filesystem::path path(performance_csv_path_);
    if (path.has_parent_path())
      std::filesystem::create_directories(path.parent_path());
    if (!std::filesystem::exists(path) || std::filesystem::file_size(path) == 0)
    {
      std::ofstream output(path);
      output << "wall_time,window_s,gazebo_color_callback_hz,gazebo_depth_callback_hz,"
                "matched_pair_hz,ros_pair_publish_hz,process_mean_ms,process_median_ms,"
                "process_p05_ms,process_p95_ms,process_min_ms,process_max_ms,color_mean_ms,"
                "depth_mean_ms,publish_mean_ms,queue_mean_ms,longest_frame_interval_ms,"
                "color_unmatched_drops,depth_unmatched_drops,ready_pair_overwrites,"
                "color_sequence_gaps,depth_sequence_gaps,exact_sync_failures,conversion_errors\n";
    }
  }

  void report_performance()
  {
    WindowStats stats;
    double window_s = 0.0;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      const auto now = SteadyClock::now();
      window_s = std::chrono::duration<double>(now - stats_window_start_).count();
      stats_window_start_ = now;
      stats = std::move(stats_);
      stats_ = WindowStats{};
    }
    const double color_hz = static_cast<double>(stats.color_callbacks) / window_s;
    const double depth_hz = static_cast<double>(stats.depth_callbacks) / window_s;
    const double matched_hz = static_cast<double>(stats.matched_pairs) / window_s;
    const double publish_hz = static_cast<double>(stats.published_pairs) / window_s;
    const double process_mean = mean(stats.process_ms);
    const double process_median = percentile(stats.process_ms, 0.5);
    const double process_p05 = percentile(stats.process_ms, 0.05);
    const double process_p95 = percentile(stats.process_ms, 0.95);
    const double process_min = stats.process_ms.empty() ? 0.0 :
      *std::min_element(stats.process_ms.begin(), stats.process_ms.end());
    const double process_max = stats.process_ms.empty() ? 0.0 :
      *std::max_element(stats.process_ms.begin(), stats.process_ms.end());
    const double longest_interval = stats.publish_intervals_ms.empty() ? 0.0 :
      *std::max_element(stats.publish_intervals_ms.begin(), stats.publish_intervals_ms.end());
    const uint64_t exact_failures =
      stats.color_unmatched_drops + stats.depth_unmatched_drops;

    RCLCPP_INFO(
      get_logger(),
      "D435I_CPP_PERF source=%.2f/%.2fHz matched=%.2fHz published=%.2fHz "
      "process=%.3f/%.3fms(mean/p95) queue=%.3fms drops=%lu/%lu overwrite=%lu",
      color_hz, depth_hz, matched_hz, publish_hz, process_mean, process_p95,
      mean(stats.queue_ms), stats.color_unmatched_drops, stats.depth_unmatched_drops,
      stats.ready_pair_overwrites);

    if (!performance_csv_path_.empty())
    {
      std::ofstream output(performance_csv_path_, std::ios::app);
      output << wall_time_iso8601() << ',' << window_s << ',' << color_hz << ',' << depth_hz
             << ',' << matched_hz << ',' << publish_hz << ',' << process_mean << ','
             << process_median << ',' << process_p05 << ',' << process_p95 << ','
             << process_min << ',' << process_max << ',' << mean(stats.color_ms) << ','
             << mean(stats.depth_ms) << ',' << mean(stats.publish_ms) << ','
             << mean(stats.queue_ms) << ',' << longest_interval << ','
             << stats.color_unmatched_drops << ',' << stats.depth_unmatched_drops << ','
             << stats.ready_pair_overwrites << ',' << stats.color_sequence_gaps << ','
             << stats.depth_sequence_gaps << ',' << exact_failures << ','
             << stats.conversion_errors << '\n';
    }
  }

  std::string gz_prefix_;
  std::string ros_prefix_;
  std::string camera_link_frame_;
  std::string mode_;
  std::string depth_encoding_;
  std::string qos_reliability_;
  std::string color_topic_;
  std::string depth_topic_;
  std::string info_topic_;
  std::string imu_topic_;
  std::string performance_csv_path_;
  const std::string color_frame_{"front_d435i_color_optical_frame"};
  const std::string depth_frame_{"front_d435i_depth_optical_frame"};
  const std::string imu_frame_{"front_d435i_imu_frame"};
  double max_depth_m_{6.0};
  double min_depth_m_{0.3};
  double pointcloud_hz_{10.0};
  double performance_stats_period_s_{5.0};
  int sync_queue_depth_{2};
  int qos_depth_{1};
  int pointcloud_stride_{4};
  bool use_gz_color_payload_{true};
  bool enable_pointcloud_{false};
  bool performance_stats_enabled_{false};

  gz::transport::Node gz_node_;
  std::mutex pair_mutex_;
  std::condition_variable pair_cv_;
  std::deque<ImageToken> color_queue_;
  std::deque<ImageToken> depth_queue_;
  std::optional<Pair> ready_pair_;
  bool stop_worker_{false};
  std::thread worker_;
  uint64_t color_sequence_{0};
  uint64_t depth_sequence_{0};

  std::mutex hybrid_color_mutex_;
  std::condition_variable hybrid_color_cv_;
  std::map<int64_t, sensor_msgs::msg::Image::ConstSharedPtr> hybrid_colors_;
  std::deque<int64_t> hybrid_color_order_;

  std::mutex info_mutex_;
  sensor_msgs::msg::CameraInfo camera_info_template_;
  bool have_camera_info_{false};

  sensor_msgs::msg::Image color_out_;
  sensor_msgs::msg::Image depth_out_;
  sensor_msgs::msg::PointCloud2 points_out_;
  std::vector<float> depth_meters_;
  SteadyClock::time_point last_cloud_time_{};
  SteadyClock::time_point last_publish_time_{};

  std::mutex stats_mutex_;
  WindowStats stats_;
  SteadyClock::time_point stats_window_start_;
  uint64_t last_published_color_sequence_{0};
  uint64_t last_published_depth_sequence_{0};
  uint64_t published_pair_sequence_{0};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gyro_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr accel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr frame_tracking_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr hybrid_color_sub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<D435iRgbdBridge>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception &error)
  {
    std::cerr << "D435i C++ bridge fatal error: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
