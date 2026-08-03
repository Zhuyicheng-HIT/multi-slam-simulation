#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/world_stats.pb.h>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "mid360_sim_bridge_cpp/conversion.hpp"

namespace mid360_sim_bridge_cpp
{

class GzLivoxBridgeNode final : public rclcpp::Node
{
public:
  GzLivoxBridgeNode()
  : Node("gz_livox_bridge")
  {
    gz_topic_ = declare_parameter<std::string>("gz_topic", "/mid360/lidar");
    lidar_topic_ = declare_parameter<std::string>("livox_lidar_topic", "/livox/lidar");
    input_imu_topic_ =
      declare_parameter<std::string>("input_imu_topic", "/mavros/imu/data_raw");
    output_imu_topic_ = declare_parameter<std::string>("livox_imu_topic", "/livox/imu");
    lidar_frame_id_ = declare_parameter<std::string>("lidar_frame_id", "mid360_link");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "base_link");
    map_frame_id_ = declare_parameter<std::string>("map_frame_id", "camera_init");
    ground_truth_odom_topic_ = declare_parameter<std::string>(
      "ground_truth_odom_topic", "/sim/mid360/ground_truth_odom");
    world_stats_topic_ = declare_parameter<std::string>(
      "world_stats_topic", "/world/simple_apm_rgbd_mid360/stats");
    rtf_topic_ = declare_parameter<std::string>("rtf_topic", "/simulation/rtf");
    publish_ground_truth_odom_ =
      declare_parameter<bool>("publish_ground_truth_odom", true);
    restamp_lidar_ = declare_parameter<bool>("restamp_lidar", true);
    stamp_lidar_from_latest_imu_ =
      declare_parameter<bool>("stamp_lidar_from_latest_imu", false);
    preserve_sim_scan_clock_ =
      declare_parameter<bool>("preserve_sim_scan_clock", false);
    restamp_imu_ = declare_parameter<bool>("restamp_imu", false);
    point_stride_ = static_cast<int>(std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("point_stride", 1)));
    max_points_ = static_cast<int>(std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("max_points", 20000)));
    line_count_ = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter<std::int64_t>(
        "scan_lines", static_cast<std::int64_t>(kDefaultLineCount)),
      1, 255));
    synthetic_scan_timing_ = declare_parameter<bool>("synthetic_scan_timing", false);
    const double frame_rate_hz =
      std::max(0.1, declare_parameter<double>("frame_rate_hz", 10.0));
    scan_period_ns_ = static_cast<std::uint64_t>(std::llround(1.0e9 / frame_rate_hz));

    const auto fastlio_qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();
    lidar_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
      lidar_topic_, fastlio_qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(output_imu_topic_, fastlio_qos);
    if (publish_ground_truth_odom_) {
      ground_truth_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        ground_truth_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(2)).best_effort());
    }
    rtf_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      rtf_topic_, rclcpp::QoS(rclcpp::KeepLast(2)).best_effort());
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GzLivoxBridgeNode::on_imu, this, std::placeholders::_1));

    if (!gz_node_.Subscribe(gz_topic_, &GzLivoxBridgeNode::on_scan, this)) {
      throw std::runtime_error("Failed to subscribe to Gazebo topic " + gz_topic_);
    }
    if (!gz_node_.Subscribe(
        world_stats_topic_, &GzLivoxBridgeNode::on_world_stats, this))
    {
      throw std::runtime_error(
              "Failed to subscribe to Gazebo topic " + world_stats_topic_);
    }

    status_timer_ = create_wall_timer(
      std::chrono::seconds(2), std::bind(&GzLivoxBridgeNode::report_status, this));
    rtf_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&GzLivoxBridgeNode::publish_rtf, this));
    last_status_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(),
      "Direct MID360 adapter active: %s -> %s, %s -> %s, stride=%d, max_points=%d, "
      "point_timing=%s, stamp_mode=%s",
      gz_topic_.c_str(), lidar_topic_.c_str(), input_imu_topic_.c_str(),
      output_imu_topic_.c_str(), point_stride_, max_points_,
      synthetic_scan_timing_ ? "synthetic_scan" : "snapshot_at_packet_end",
      stamp_lidar_from_latest_imu_ ? "latest_fcu_imu" :
      (preserve_sim_scan_clock_ ? "sim_rate_epoch_aligned" :
      (restamp_lidar_ ? "wall_each_frame" : "raw_gazebo")));
  }

private:
  static std::int64_t protobuf_stamp_ns(const gz::msgs::LaserScan & msg)
  {
    if (!msg.has_header() || !msg.header().has_stamp()) {
      return 0;
    }
    return static_cast<std::int64_t>(msg.header().stamp().sec()) * 1000000000LL +
           static_cast<std::int64_t>(msg.header().stamp().nsec());
  }

  std::int64_t monotonic_lidar_stamp(const gz::msgs::LaserScan & msg)
  {
    const auto source_stamp_ns = protobuf_stamp_ns(msg);
    std::int64_t stamp_ns = 0;
    const auto latest_imu_stamp_ns =
      last_imu_stamp_ns_.load(std::memory_order_relaxed);
    if (stamp_lidar_from_latest_imu_ && latest_imu_stamp_ns > 0) {
      stamp_ns = latest_imu_stamp_ns;
    } else if (preserve_sim_scan_clock_ && source_stamp_ns > 0) {
      if (lidar_source_origin_ns_ <= 0) {
        lidar_source_origin_ns_ = source_stamp_ns;
        lidar_epoch_origin_ns_ = now().nanoseconds();
      }
      stamp_ns = epoch_aligned_stamp_ns(
        source_stamp_ns, lidar_source_origin_ns_, lidar_epoch_origin_ns_);
    } else {
      stamp_ns = restamp_lidar_ ? now().nanoseconds() : source_stamp_ns;
    }
    if (stamp_ns <= 0) {
      stamp_ns = now().nanoseconds();
    }
    const auto previous = last_lidar_stamp_ns_.load(std::memory_order_relaxed);
    if (stamp_ns <= previous) {
      stamp_ns = previous + 1;
      adjusted_lidar_stamps_.fetch_add(1, std::memory_order_relaxed);
    }
    last_lidar_stamp_ns_.store(stamp_ns, std::memory_order_relaxed);
    return stamp_ns;
  }

  void refresh_angle_cache(const gz::msgs::LaserScan & msg)
  {
    const auto horizontal_count = std::max<std::size_t>(1U, msg.count());
    const auto vertical_count = std::max<std::size_t>(1U, msg.vertical_count());
    const bool unchanged =
      horizontal_count == horizontal_cos_.size() &&
      vertical_count == vertical_cos_.size() &&
      msg.angle_min() == cached_angle_min_ &&
      msg.angle_step() == cached_angle_step_ &&
      msg.vertical_angle_min() == cached_vertical_min_ &&
      msg.vertical_angle_step() == cached_vertical_step_;
    if (unchanged) {
      return;
    }

    horizontal_cos_.resize(horizontal_count);
    horizontal_sin_.resize(horizontal_count);
    vertical_cos_.resize(vertical_count);
    vertical_sin_.resize(vertical_count);
    for (std::size_t i = 0; i < horizontal_count; ++i) {
      const double angle = msg.angle_min() + static_cast<double>(i) * msg.angle_step();
      horizontal_cos_[i] = static_cast<float>(std::cos(angle));
      horizontal_sin_[i] = static_cast<float>(std::sin(angle));
    }
    for (std::size_t i = 0; i < vertical_count; ++i) {
      const double angle =
        msg.vertical_angle_min() + static_cast<double>(i) * msg.vertical_angle_step();
      vertical_cos_[i] = static_cast<float>(std::cos(angle));
      vertical_sin_[i] = static_cast<float>(std::sin(angle));
    }
    cached_angle_min_ = msg.angle_min();
    cached_angle_step_ = msg.angle_step();
    cached_vertical_min_ = msg.vertical_angle_min();
    cached_vertical_step_ = msg.vertical_angle_step();
  }

  void on_scan(const gz::msgs::LaserScan & msg)
  {
    const std::lock_guard<std::mutex> lock(scan_mutex_);
    refresh_angle_cache(msg);

    const std::size_t horizontal_count = horizontal_cos_.size();
    const std::size_t vertical_count = vertical_cos_.size();
    const std::size_t declared_count = horizontal_count * vertical_count;
    const std::size_t source_count = std::min<std::size_t>(
      declared_count, static_cast<std::size_t>(msg.ranges_size()));
    if (source_count == 0U) {
      return;
    }

    livox_ros_driver2::msg::CustomMsg output;
    const auto acquisition_stamp_ns = monotonic_lidar_stamp(msg);
    const auto packet_stamp_ns = packet_begin_stamp_ns(
      acquisition_stamp_ns, scan_period_ns_, synthetic_scan_timing_);
    output.header.stamp = rclcpp::Time(packet_stamp_ns, RCL_SYSTEM_TIME);
    output.header.frame_id = lidar_frame_id_;
    output.timebase = static_cast<std::uint64_t>(packet_stamp_ns);
    output.lidar_id = 1U;
    output.rsvd = {0U, 0U, 0U};
    const std::size_t reserve_count = std::min<std::size_t>(
      static_cast<std::size_t>(max_points_),
      (source_count + static_cast<std::size_t>(point_stride_) - 1U) /
      static_cast<std::size_t>(point_stride_));
    output.points.reserve(reserve_count);

    const double range_min = msg.range_min();
    const double range_max = msg.range_max();
    const std::size_t stride = static_cast<std::size_t>(point_stride_);
    for (std::size_t source_index = 0;
      source_index < source_count && output.points.size() < reserve_count;
      source_index += stride)
    {
      const double range = msg.ranges(static_cast<int>(source_index));
      if (!std::isfinite(range) || range < range_min || range > range_max) {
        continue;
      }
      const std::size_t vertical_index = source_index / horizontal_count;
      const std::size_t horizontal_index = source_index % horizontal_count;
      const float radial_xy = static_cast<float>(range) * vertical_cos_[vertical_index];

      livox_ros_driver2::msg::CustomPoint point;
      point.x = radial_xy * horizontal_cos_[horizontal_index];
      point.y = radial_xy * horizontal_sin_[horizontal_index];
      point.z = static_cast<float>(range) * vertical_sin_[vertical_index];
      const double intensity = source_index < static_cast<std::size_t>(msg.intensities_size()) ?
        msg.intensities(static_cast<int>(source_index)) : 120.0;
      point.reflectivity = reflectivity_from_intensity(intensity);
      point.tag = kDefaultTag;
      point.line = line_for_output_index(output.points.size(), line_count_);
      point.offset_time = point_offset_time_ns(
        source_index, source_count, scan_period_ns_, synthetic_scan_timing_);
      output.points.push_back(point);
    }

    output.point_num = static_cast<std::uint32_t>(output.points.size());
    publish_ground_truth_odom(msg, acquisition_stamp_ns);
    last_point_count_.store(output.point_num, std::memory_order_relaxed);
    cloud_count_.fetch_add(1, std::memory_order_relaxed);
    lidar_pub_->publish(std::move(output));
  }

  void publish_ground_truth_odom(
    const gz::msgs::LaserScan & msg, const std::int64_t stamp_ns)
  {
    if (!ground_truth_odom_pub_ || !msg.has_world_pose()) {
      return;
    }
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Time(stamp_ns, RCL_SYSTEM_TIME);
    odom.header.frame_id = map_frame_id_;
    odom.child_frame_id = lidar_frame_id_;
    const auto & pose = msg.world_pose();
    odom.pose.pose.position.x = pose.position().x();
    odom.pose.pose.position.y = pose.position().y();
    odom.pose.pose.position.z = pose.position().z();
    odom.pose.pose.orientation.x = pose.orientation().x();
    odom.pose.pose.orientation.y = pose.orientation().y();
    odom.pose.pose.orientation.z = pose.orientation().z();
    odom.pose.pose.orientation.w = pose.orientation().w();
    ground_truth_odom_pub_->publish(std::move(odom));
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr input)
  {
    sensor_msgs::msg::Imu output = *input;
    if (restamp_imu_) {
      output.header.stamp = now();
    }
    auto stamp_ns = rclcpp::Time(output.header.stamp).nanoseconds();
    const auto previous = last_imu_stamp_ns_.load(std::memory_order_relaxed);
    if (stamp_ns <= previous) {
      stamp_ns = previous + 1;
      output.header.stamp = rclcpp::Time(stamp_ns, RCL_SYSTEM_TIME);
      adjusted_imu_stamps_.fetch_add(1, std::memory_order_relaxed);
    }
    output.header.frame_id = imu_frame_id_;
    last_imu_stamp_ns_.store(stamp_ns, std::memory_order_relaxed);
    imu_count_.fetch_add(1, std::memory_order_relaxed);
    imu_pub_->publish(std::move(output));
  }

  void on_world_stats(const gz::msgs::WorldStatistics & msg)
  {
    const double rtf = msg.real_time_factor();
    if (std::isfinite(rtf) && rtf >= 0.0) {
      latest_rtf_.store(rtf, std::memory_order_relaxed);
    }
    const double step_ms =
      static_cast<double>(msg.step_size().sec()) * 1000.0 +
      static_cast<double>(msg.step_size().nsec()) * 1.0e-6;
    if (std::isfinite(step_ms) && step_ms >= 0.0) {
      latest_step_ms_.store(step_ms, std::memory_order_relaxed);
    }
    latest_sim_time_s_.store(
      static_cast<double>(msg.sim_time().sec()) +
      static_cast<double>(msg.sim_time().nsec()) * 1.0e-9,
      std::memory_order_relaxed);
    latest_real_time_s_.store(
      static_cast<double>(msg.real_time().sec()) +
      static_cast<double>(msg.real_time().nsec()) * 1.0e-9,
      std::memory_order_relaxed);
    world_stats_count_.fetch_add(1, std::memory_order_relaxed);
  }

  void publish_rtf()
  {
    if (world_stats_count_.load(std::memory_order_relaxed) == 0U) {
      return;
    }
    std_msgs::msg::Float64MultiArray output;
    output.data = {
      latest_rtf_.load(std::memory_order_relaxed),
      latest_step_ms_.load(std::memory_order_relaxed),
      latest_sim_time_s_.load(std::memory_order_relaxed),
      latest_real_time_s_.load(std::memory_order_relaxed)};
    rtf_pub_->publish(std::move(output));
  }

  void report_status()
  {
    const auto current_time = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(current_time - last_status_time_).count();
    const auto clouds = cloud_count_.load(std::memory_order_relaxed);
    const auto imus = imu_count_.load(std::memory_order_relaxed);
    const double cloud_hz = static_cast<double>(clouds - last_status_cloud_count_) / elapsed;
    const double imu_hz = static_cast<double>(imus - last_status_imu_count_) / elapsed;
    const auto lidar_stamp_ns = last_lidar_stamp_ns_.load(std::memory_order_relaxed);
    const auto imu_stamp_ns = last_imu_stamp_ns_.load(std::memory_order_relaxed);
    const double stamp_delta_ms = lidar_stamp_ns > 0 && imu_stamp_ns > 0 ?
      static_cast<double>(lidar_stamp_ns - imu_stamp_ns) / 1.0e6 : 0.0;
    RCLCPP_INFO(
      get_logger(),
      "direct bridge clouds=%lu points=%u cloud_hz=%.2f imu_hz=%.2f "
      "cloud_minus_imu_ms=%.1f adjusted_lidar=%lu adjusted_imu=%lu",
      static_cast<unsigned long>(clouds), last_point_count_.load(std::memory_order_relaxed),
      cloud_hz, imu_hz, stamp_delta_ms,
      static_cast<unsigned long>(adjusted_lidar_stamps_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(adjusted_imu_stamps_.load(std::memory_order_relaxed)));
    last_status_time_ = current_time;
    last_status_cloud_count_ = clouds;
    last_status_imu_count_ = imus;
  }

  std::string gz_topic_;
  std::string lidar_topic_;
  std::string input_imu_topic_;
  std::string output_imu_topic_;
  std::string lidar_frame_id_;
  std::string imu_frame_id_;
  std::string map_frame_id_;
  std::string ground_truth_odom_topic_;
  std::string world_stats_topic_;
  std::string rtf_topic_;
  bool publish_ground_truth_odom_{true};
  bool restamp_lidar_{true};
  bool stamp_lidar_from_latest_imu_{false};
  bool preserve_sim_scan_clock_{false};
  bool restamp_imu_{false};
  bool synthetic_scan_timing_{false};
  int point_stride_{1};
  int max_points_{20000};
  int line_count_{static_cast<int>(kDefaultLineCount)};
  std::uint64_t scan_period_ns_{100000000ULL};

  gz::transport::Node gz_node_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rtf_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr rtf_timer_;

  std::mutex scan_mutex_;
  std::vector<float> horizontal_cos_;
  std::vector<float> horizontal_sin_;
  std::vector<float> vertical_cos_;
  std::vector<float> vertical_sin_;
  double cached_angle_min_{std::numeric_limits<double>::quiet_NaN()};
  double cached_angle_step_{std::numeric_limits<double>::quiet_NaN()};
  double cached_vertical_min_{std::numeric_limits<double>::quiet_NaN()};
  double cached_vertical_step_{std::numeric_limits<double>::quiet_NaN()};

  std::atomic<std::uint64_t> cloud_count_{0U};
  std::atomic<std::uint64_t> imu_count_{0U};
  std::atomic<std::uint64_t> adjusted_lidar_stamps_{0U};
  std::atomic<std::uint64_t> adjusted_imu_stamps_{0U};
  std::atomic<std::uint32_t> last_point_count_{0U};
  std::atomic<std::int64_t> last_lidar_stamp_ns_{0};
  std::atomic<std::int64_t> last_imu_stamp_ns_{0};
  std::int64_t lidar_source_origin_ns_{0};
  std::int64_t lidar_epoch_origin_ns_{0};
  std::atomic<std::uint64_t> world_stats_count_{0U};
  std::atomic<double> latest_rtf_{0.0};
  std::atomic<double> latest_step_ms_{0.0};
  std::atomic<double> latest_sim_time_s_{0.0};
  std::atomic<double> latest_real_time_s_{0.0};
  std::chrono::steady_clock::time_point last_status_time_;
  std::uint64_t last_status_cloud_count_{0U};
  std::uint64_t last_status_imu_count_{0U};
};

}  // namespace mid360_sim_bridge_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mid360_sim_bridge_cpp::GzLivoxBridgeNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("gz_livox_bridge"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
