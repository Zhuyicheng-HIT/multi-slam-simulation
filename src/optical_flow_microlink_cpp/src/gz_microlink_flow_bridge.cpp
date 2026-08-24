#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>
#include <mavros_msgs/msg/optical_flow_rad.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/header.hpp>

namespace {

double stamp_seconds(const gz::msgs::Image &msg)
{
  return static_cast<double>(msg.header().stamp().sec()) +
    static_cast<double>(msg.header().stamp().nsec()) * 1.0e-9;
}

double stamp_seconds(const gz::msgs::LaserScan &msg)
{
  return static_cast<double>(msg.header().stamp().sec()) +
    static_cast<double>(msg.header().stamp().nsec()) * 1.0e-9;
}

double stamp_seconds(const sensor_msgs::msg::Imu &msg)
{
  return static_cast<double>(msg.header.stamp.sec) +
    static_cast<double>(msg.header.stamp.nanosec) * 1.0e-9;
}

struct GyroSample
{
  double stamp{};
  cv::Vec3d value{};
};

struct RangeSample
{
  double stamp{};
  double value{};
};

}  // namespace

class GazeboMicoLinkFlowBridge final : public rclcpp::Node
{
public:
  GazeboMicoLinkFlowBridge()
  : Node("gz_microlink_flow_bridge")
  {
    image_topic_ = declare_parameter("image_gz_topic", "/camera/camera");
    range_topic_ = declare_parameter("range_gz_topic", "/flow/range");
    imu_topic_ = declare_parameter("imu_topic", "/mavros/imu/data_raw");
    flow_topic_ = declare_parameter("flow_topic", "/sim/optical_flow/rad");
    output_range_topic_ = declare_parameter("range_topic", "/sim/optical_flow/range");
    frame_id_ = declare_parameter("frame_id", "mtf01_flow_frd");
    max_rate_hz_ = declare_parameter("max_rate_hz", 15.0);
    max_imu_gap_s_ = declare_parameter("max_imu_gap_s", 0.12);
    range_timeout_s_ = declare_parameter("range_timeout_s", 0.25);
    max_corners_ = declare_parameter("max_corners", 160);
    min_inliers_ = declare_parameter("min_inliers", 8);
    focal_length_px_ = declare_parameter("focal_length_px", 75.0);
    min_range_m_ = declare_parameter("min_range_m", 0.08);
    max_range_m_ = declare_parameter("max_range_m", 12.0);

    flow_pub_ = create_publisher<mavros_msgs::msg::OpticalFlowRad>(
      flow_topic_, rclcpp::SensorDataQoS());
    range_pub_ = create_publisher<sensor_msgs::msg::Range>(
      output_range_topic_, rclcpp::SensorDataQoS());
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GazeboMicoLinkFlowBridge::on_imu, this, std::placeholders::_1));

    gz_node_.Subscribe(image_topic_, &GazeboMicoLinkBridge::on_image, this);
    gz_node_.Subscribe(range_topic_, &GazeboMicoLinkBridge::on_range, this);
    RCLCPP_INFO(get_logger(),
      "C++ latest-only MicoLink flow bridge active: image=%s range=%s imu=%s -> %s",
      image_topic_.c_str(), range_topic_.c_str(), imu_topic_.c_str(), flow_topic_.c_str());
  }

private:
  using GazeboMicoLinkBridge = GazeboMicoLinkFlowBridge;

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const double stamp = stamp_seconds(*msg);
    if (!(stamp > 0.0) || !std::isfinite(stamp)) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    gyro_.push_back({stamp, cv::Vec3d(
        msg->angular_velocity.x, -msg->angular_velocity.y, -msg->angular_velocity.z)});
    while (gyro_.size() > 256 || (gyro_.size() > 2 && stamp - gyro_.front().stamp > 1.0)) {
      gyro_.pop_front();
    }
  }

  void on_range(const gz::msgs::LaserScan &msg)
  {
    std::vector<double> values;
    values.reserve(static_cast<size_t>(msg.ranges_size()));
    for (int i = 0; i < msg.ranges_size(); ++i) {
      const double value = msg.ranges(i);
      if (std::isfinite(value) && value >= min_range_m_ && value <= max_range_m_) {
        values.push_back(value);
      }
    }
    if (values.empty()) {
      return;
    }
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    std::lock_guard<std::mutex> lock(mutex_);
    range_ = {stamp_seconds(msg), *middle};
  }

  cv::Mat image_to_gray(const gz::msgs::Image &msg) const
  {
    const int width = static_cast<int>(msg.width());
    const int height = static_cast<int>(msg.height());
    if (width <= 0 || height <= 0) {
      return {};
    }
    const size_t pixels = static_cast<size_t>(width) * static_cast<size_t>(height);
    const std::string &data = msg.data();
    if (data.size() < pixels) {
      return {};
    }
    cv::Mat gray(height, width, CV_8UC1);
    const size_t channels = data.size() >= pixels * 3 ? 3 : 1;
    for (size_t i = 0; i < pixels; ++i) {
      if (channels == 1) {
        gray.data[i] = static_cast<unsigned char>(data[i]);
      } else {
        const size_t offset = i * channels;
        gray.data[i] = static_cast<unsigned char>(
          (static_cast<unsigned char>(data[offset]) +
           static_cast<unsigned char>(data[offset + 1]) +
           static_cast<unsigned char>(data[offset + 2])) / 3);
      }
    }
    return gray;
  }

  bool integrate_gyro(double start, double end, cv::Vec3d &result)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (gyro_.size() < 2 || end <= start) {
      return false;
    }
    const auto first = std::lower_bound(
      gyro_.begin(), gyro_.end(), start,
      [](const GyroSample &sample, double value) {return sample.stamp < value;});
    const auto last = std::upper_bound(
      gyro_.begin(), gyro_.end(), end,
      [](double value, const GyroSample &sample) {return value < sample.stamp;});
    if (first == gyro_.begin() || last == gyro_.end()) {
      return false;
    }
    std::vector<GyroSample> samples;
    samples.emplace_back(*(first - 1));
    samples.insert(samples.end(), first, last);
    samples.emplace_back(*last);
    result = cv::Vec3d(0.0, 0.0, 0.0);
    for (size_t i = 1; i < samples.size(); ++i) {
      const double t0 = std::max(start, samples[i - 1].stamp);
      const double t1 = std::min(end, samples[i].stamp);
      if (t1 > t0 && t1 - t0 <= max_imu_gap_s_) {
        result += 0.5 * (samples[i - 1].value + samples[i].value) * (t1 - t0);
      }
    }
    return true;
  }

  void publish_range(const std_msgs::msg::Header &header, double distance)
  {
    if (!std::isfinite(distance)) {
      return;
    }
    sensor_msgs::msg::Range range_msg;
    range_msg.header = header;
    range_msg.header.frame_id = "flow_range_link";
    range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    range_msg.field_of_view = 0.119428926;
    range_msg.min_range = min_range_m_;
    range_msg.max_range = max_range_m_;
    range_msg.range = distance;
    range_pub_->publish(range_msg);
  }

  void publish_low_quality(double stamp, double dt)
  {
    cv::Vec3d gyro{};
    const bool gyro_valid = integrate_gyro(stamp - dt, stamp, gyro);
    double distance = std::numeric_limits<double>::quiet_NaN();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (range_.stamp > 0.0 && std::abs(stamp - range_.stamp) <= range_timeout_s_) {
        distance = range_.value;
      }
    }
    mavros_msgs::msg::OpticalFlowRad flow;
    const int64_t stamp_ns = static_cast<int64_t>(stamp * 1.0e9);
    flow.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    flow.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    flow.header.frame_id = frame_id_;
    flow.integration_time_us = static_cast<uint32_t>(std::max(1.0, dt * 1.0e6));
    flow.integrated_xgyro = gyro_valid ? gyro[0] : 0.0;
    flow.integrated_ygyro = gyro_valid ? gyro[1] : 0.0;
    flow.integrated_zgyro = gyro_valid ? gyro[2] : 0.0;
    flow.quality = 0;
    flow.distance = std::isfinite(distance) ? distance : -1.0;
    flow_pub_->publish(flow);
    publish_range(flow.header, distance);
  }

  void on_image(const gz::msgs::Image &msg)
  {
    const double stamp = stamp_seconds(msg);
    const cv::Mat current = image_to_gray(msg);
    if (current.empty() || !(stamp > 0.0)) {
      return;
    }
    if (previous_.empty()) {
      previous_ = current;
      previous_stamp_ = stamp;
      return;
    }
    const double dt = stamp - previous_stamp_;
    const double minimum_period = 1.0 / std::max(1.0, max_rate_hz_);
    if (dt < minimum_period * 0.90) {
      return;
    }
    if (dt > 0.5) {
      previous_ = current;
      previous_stamp_ = stamp;
      return;
    }
    cv::Mat previous = previous_;
    previous_ = current;
    previous_stamp_ = stamp;

    std::vector<cv::Point2f> points;
    cv::goodFeaturesToTrack(previous, points, max_corners_, 0.01, 7.0);
    if (points.empty()) {
      publish_low_quality(stamp, dt);
      return;
    }
    std::vector<cv::Point2f> next, back;
    std::vector<unsigned char> status, back_status;
    std::vector<float> error;
    const cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01);
    cv::calcOpticalFlowPyrLK(previous, current, points, next, status, error, {21, 21}, 3, criteria);
    if (next.empty()) {
      publish_low_quality(stamp, dt);
      return;
    }
    cv::calcOpticalFlowPyrLK(current, previous, next, back, back_status, error, {21, 21}, 3, criteria);
    std::vector<cv::Point2f> displacements;
    for (size_t i = 0; i < points.size() && i < next.size() && i < back.size(); ++i) {
      const cv::Point2f delta = next[i] - points[i];
      const double norm = std::hypot(delta.x, delta.y);
      const double fb = cv::norm(back[i] - points[i]);
      if (status[i] && back_status[i] && std::isfinite(norm) && norm <= 40.0 && fb <= 1.0) {
        displacements.push_back(delta);
      }
    }
    if (static_cast<int>(displacements.size()) < min_inliers_) {
      publish_low_quality(stamp, dt);
      return;
    }
    std::vector<float> xs, ys;
    xs.reserve(displacements.size());
    ys.reserve(displacements.size());
    for (const auto &delta : displacements) {xs.push_back(delta.x); ys.push_back(delta.y);}
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    const float dx = xs[xs.size() / 2];
    const float dy = ys[ys.size() / 2];
    cv::Vec3d gyro{};
    const bool gyro_valid = integrate_gyro(stamp - dt, stamp, gyro);
    double distance = std::numeric_limits<double>::quiet_NaN();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (range_.stamp > 0.0 && std::abs(stamp - range_.stamp) <= range_timeout_s_) {
        distance = range_.value;
      }
    }
    mavros_msgs::msg::OpticalFlowRad flow;
    const int64_t stamp_ns = static_cast<int64_t>(stamp * 1.0e9);
    flow.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    flow.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    flow.header.frame_id = frame_id_;
    flow.integration_time_us = static_cast<uint32_t>(std::max(1.0, dt * 1.0e6));
    const double image_x = std::atan2(static_cast<double>(dx), focal_length_px_);
    const double image_y = std::atan2(static_cast<double>(dy), focal_length_px_);
    // Match the MTF-01 MicoLink int16 cm/s-at-1m wire representation before
    // converting back to OPTICAL_FLOW_RAD for the unified ROS interface.
    const auto microlink_velocity = [dt](double integrated) {
        return std::clamp(
          static_cast<long>(std::lround(integrated / dt * 100.0)),
          static_cast<long>(std::numeric_limits<int16_t>::min()),
          static_cast<long>(std::numeric_limits<int16_t>::max()));
      };
    flow.integrated_x = static_cast<double>(microlink_velocity(image_x)) * 0.01 * dt;
    flow.integrated_y = static_cast<double>(microlink_velocity(image_y)) * 0.01 * dt;
    flow.integrated_xgyro = gyro_valid ? gyro[0] : 0.0;
    flow.integrated_ygyro = gyro_valid ? gyro[1] : 0.0;
    flow.integrated_zgyro = gyro_valid ? gyro[2] : 0.0;
    flow.quality = static_cast<uint8_t>(std::min<size_t>(255, displacements.size() * 8));
    flow.distance = std::isfinite(distance) ? distance : -1.0;
    flow_pub_->publish(flow);
    publish_range(flow.header, distance);
  }

  std::string image_topic_, range_topic_, imu_topic_, flow_topic_, output_range_topic_, frame_id_;
  double max_rate_hz_{15.0}, max_imu_gap_s_{0.12}, range_timeout_s_{0.25};
  int max_corners_{160}, min_inliers_{8};
  double focal_length_px_{75.0}, min_range_m_{0.08}, max_range_m_{12.0};
  rclcpp::Publisher<mavros_msgs::msg::OpticalFlowRad>::SharedPtr flow_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  gz::transport::Node gz_node_;
  cv::Mat previous_;
  double previous_stamp_{0.0};
  std::deque<GyroSample> gyro_;
  RangeSample range_{};
  std::mutex mutex_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboMicoLinkFlowBridge>());
  rclcpp::shutdown();
  return 0;
}
