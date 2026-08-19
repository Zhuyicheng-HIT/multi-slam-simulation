#include "uf_dynamic_observer/conservative_free_space.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace uf_dynamic_observer
{
namespace
{

std::int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

diagnostic_msgs::msg::KeyValue diagnostic_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

std::string number(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

}  // namespace

class DynamicObserverNode : public rclcpp::Node
{
public:
  DynamicObserverNode()
  : Node("dynamic_static_map_observer")
  {
    const bool enabled = declare_parameter<bool>("enabled", false);
    input_mode_ = declare_parameter<std::string>("input_mode", "livox_custom");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    const auto pose_topic = declare_parameter<std::string>("pose_topic", "/fusion/unified/odom");
    const auto livox_topic = declare_parameter<std::string>("livox_topic", "/livox/lidar");
    const auto pointcloud_topic =
      declare_parameter<std::string>("pointcloud_topic", "/sim/mid360/points_raw");
    max_pending_scans_ = static_cast<std::size_t>(
      std::max<std::int64_t>(1, declare_parameter<int>("max_pending_scans", 8)));
    max_pose_wait_ms_ = declare_parameter<double>("max_pose_wait_ms", 250.0);
    max_pose_gap_s_ = declare_parameter<double>("max_pose_gap_s", 0.15);

    FilterConfig config;
    config.voxel_size_m = declare_parameter<double>("filter.voxel_size_m", 0.25);
    config.min_range_m = declare_parameter<double>("filter.min_range_m", 0.5);
    config.max_range_m = declare_parameter<double>("filter.max_range_m", 35.0);
    config.free_confirmations = static_cast<std::uint16_t>(
      std::max<std::int64_t>(1, declare_parameter<int>("filter.free_confirmations", 4)));
    config.static_confirmations = static_cast<std::uint16_t>(
      std::max<std::int64_t>(1, declare_parameter<int>("filter.static_confirmations", 2)));
    config.occupied_recovery = static_cast<std::uint16_t>(
      std::max<std::int64_t>(1, declare_parameter<int>("filter.occupied_recovery", 20)));
    config.endpoint_guard_voxels =
      static_cast<int>(std::max<std::int64_t>(
        0, declare_parameter<int>("filter.endpoint_guard_voxels", 1)));
    config.dynamic_growth_voxels =
      static_cast<int>(std::max<std::int64_t>(
        0, declare_parameter<int>("filter.dynamic_growth_voxels", 1)));
    config.ray_stride = static_cast<int>(std::max<std::int64_t>(
      1, declare_parameter<int>("filter.ray_stride", 4)));
    config.max_voxels = static_cast<std::size_t>(
      std::max<std::int64_t>(1000, declare_parameter<int>("filter.max_voxels", 1500000)));
    observer_ = std::make_unique<ConservativeFreeSpaceObserver>(config);

    const auto translation = declare_parameter<std::vector<double>>(
      "extrinsic.body_from_lidar_translation", {0.0, 0.0, 0.0});
    const auto quaternion = declare_parameter<std::vector<double>>(
      "extrinsic.body_from_lidar_quaternion_xyzw", {0.0, 0.0, 0.0, 1.0});
    if (translation.size() != 3U || quaternion.size() != 4U) {
      throw std::invalid_argument("LiDAR extrinsic parameters must have 3 and 4 elements");
    }
    Eigen::Quaterniond body_from_lidar_rotation(
      quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
    if (body_from_lidar_rotation.norm() < 1.0e-9) {
      throw std::invalid_argument("LiDAR extrinsic quaternion is invalid");
    }
    body_from_lidar_rotation.normalize();
    body_from_lidar_ = Eigen::Isometry3d::Identity();
    body_from_lidar_.linear() = body_from_lidar_rotation.toRotationMatrix();
    body_from_lidar_.translation() = Eigen::Vector3d(
      translation[0], translation[1], translation[2]);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(4);
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort();
    static_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dynamic_observer/static_candidates", output_qos);
    dynamic_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dynamic_observer/dynamic_candidates", output_qos);
    unknown_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dynamic_observer/unknown_candidates", output_qos);
    score_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dynamic_observer/scored_cloud", output_qos);
    statistics_pub_ = create_publisher<std_msgs::msg::String>(
      "/dynamic_observer/statistics", rclcpp::QoS(10));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/dynamic_observer/latency_diagnostics", rclcpp::QoS(10));

    if (!enabled) {
      RCLCPP_INFO(
        get_logger(),
        "Dynamic observer is disabled. It does not subscribe to or modify the FAST-LIO input.");
      return;
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      pose_topic, sensor_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {on_odometry(*message);});
    if (input_mode_ == "livox_custom") {
      livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic, sensor_qos,
        [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {on_livox(*message);});
    } else if (input_mode_ == "pointcloud2") {
      pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic, sensor_qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {on_pointcloud(*message);});
    } else {
      throw std::invalid_argument("input_mode must be livox_custom or pointcloud2");
    }
    drain_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() {drain_pending();});
    RCLCPP_INFO(
      get_logger(),
      "Observer enabled in %s mode. Outputs are side-channel only; FAST-LIO ownership is unchanged.",
      input_mode_.c_str());
  }

private:
  struct PoseSample
  {
    std::int64_t stamp_ns{0};
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  };

  struct RawPoint
  {
    Point point;
    std::uint32_t offset_ns{0U};
  };

  struct PendingScan
  {
    builtin_interfaces::msg::Time stamp;
    std::int64_t start_ns{0};
    std::int64_t end_ns{0};
    std::string source_frame;
    std::vector<RawPoint> points;
    std::chrono::steady_clock::time_point arrival;
  };

  void on_odometry(const nav_msgs::msg::Odometry & message)
  {
    PoseSample sample;
    sample.stamp_ns = stamp_ns(message.header.stamp);
    sample.translation = Eigen::Vector3d(
      message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z);
    sample.rotation = Eigen::Quaterniond(
      message.pose.pose.orientation.w, message.pose.pose.orientation.x,
      message.pose.pose.orientation.y, message.pose.pose.orientation.z);
    if (!sample.translation.allFinite() || sample.rotation.norm() < 1.0e-9) {
      return;
    }
    sample.rotation.normalize();
    if (!poses_.empty() && sample.stamp_ns <= poses_.back().stamp_ns) {
      return;
    }
    poses_.push_back(sample);
    while (poses_.size() > 1000U) {
      poses_.pop_front();
    }
  }

  void on_livox(const livox_ros_driver2::msg::CustomMsg & message)
  {
    PendingScan scan;
    scan.stamp = message.header.stamp;
    scan.start_ns = stamp_ns(message.header.stamp);
    scan.end_ns = scan.start_ns;
    scan.source_frame = message.header.frame_id;
    scan.arrival = std::chrono::steady_clock::now();
    scan.points.reserve(message.points.size());
    for (const auto & source : message.points) {
      if (!std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(source.z)) {
        continue;
      }
      scan.points.push_back({
        {source.x, source.y, source.z, static_cast<float>(source.reflectivity)},
        source.offset_time});
      scan.end_ns = std::max(
        scan.end_ns, scan.start_ns + static_cast<std::int64_t>(source.offset_time));
    }
    enqueue(std::move(scan));
  }

  void on_pointcloud(const sensor_msgs::msg::PointCloud2 & message)
  {
    PendingScan scan;
    scan.stamp = message.header.stamp;
    scan.start_ns = stamp_ns(message.header.stamp);
    scan.end_ns = scan.start_ns;
    scan.source_frame = message.header.frame_id;
    scan.arrival = std::chrono::steady_clock::now();
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
          scan.points.push_back({{*x, *y, *z, 0.0F}, 0U});
        }
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "PointCloud2 is missing float x/y/z fields: %s",
        error.what());
      return;
    }
    enqueue(std::move(scan));
  }

  void enqueue(PendingScan scan)
  {
    if (scan.points.empty()) {
      return;
    }
    if (pending_.size() >= max_pending_scans_) {
      pending_.pop_front();
      ++queue_overflow_count_;
    }
    pending_.push_back(std::move(scan));
  }

  std::optional<PoseSample> pose_at(std::int64_t query_ns) const
  {
    if (poses_.size() < 2U || query_ns < poses_.front().stamp_ns ||
      query_ns > poses_.back().stamp_ns)
    {
      return std::nullopt;
    }
    auto right = std::lower_bound(
      poses_.begin(), poses_.end(), query_ns,
      [](const PoseSample & sample, std::int64_t stamp) {return sample.stamp_ns < stamp;});
    if (right == poses_.end()) {
      return std::nullopt;
    }
    if (right->stamp_ns == query_ns) {
      return *right;
    }
    if (right == poses_.begin()) {
      return std::nullopt;
    }
    const auto left = std::prev(right);
    const double span_s = static_cast<double>(right->stamp_ns - left->stamp_ns) * 1.0e-9;
    if (!(span_s > 0.0) || span_s > max_pose_gap_s_) {
      return std::nullopt;
    }
    const double ratio = static_cast<double>(query_ns - left->stamp_ns) /
      static_cast<double>(right->stamp_ns - left->stamp_ns);
    PoseSample output;
    output.stamp_ns = query_ns;
    output.translation = left->translation + ratio * (right->translation - left->translation);
    output.rotation = left->rotation.slerp(ratio, right->rotation).normalized();
    return output;
  }

  static Eigen::Isometry3d transform(const PoseSample & pose)
  {
    Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
    output.linear() = pose.rotation.toRotationMatrix();
    output.translation() = pose.translation;
    return output;
  }

  void drain_pending()
  {
    while (!pending_.empty()) {
      const auto now = std::chrono::steady_clock::now();
      const double residence_ms =
        std::chrono::duration<double, std::milli>(now - pending_.front().arrival).count();
      const bool expired = residence_ms > max_pose_wait_ms_;
      if (poses_.empty() || poses_.back().stamp_ns < pending_.front().end_ns) {
        if (!expired) {
          return;
        }
        ++pose_timeout_count_;
        pending_.pop_front();
        continue;
      }
      auto start_pose = pose_at(pending_.front().start_ns);
      auto end_pose = pose_at(pending_.front().end_ns);
      if (!start_pose || !end_pose) {
        if (!expired) {
          return;
        }
        ++pose_timeout_count_;
        pending_.pop_front();
        continue;
      }
      PendingScan scan = std::move(pending_.front());
      pending_.pop_front();
      process_scan(scan, *start_pose, *end_pose, residence_ms);
    }
  }

  void process_scan(
    const PendingScan & scan, const PoseSample & start_pose, const PoseSample & end_pose,
    double queue_residence_ms)
  {
    const auto wall_start = std::chrono::steady_clock::now();
    std::vector<Point> world_points;
    world_points.reserve(scan.points.size());
    const double duration_ns = static_cast<double>(std::max<std::int64_t>(1, scan.end_ns - scan.start_ns));
    for (const auto & raw : scan.points) {
      const double ratio = std::clamp(static_cast<double>(raw.offset_ns) / duration_ns, 0.0, 1.0);
      PoseSample interpolated;
      interpolated.translation =
        start_pose.translation + ratio * (end_pose.translation - start_pose.translation);
      interpolated.rotation = start_pose.rotation.slerp(ratio, end_pose.rotation).normalized();
      const Eigen::Isometry3d world_from_lidar = transform(interpolated) * body_from_lidar_;
      const Eigen::Vector3d source(raw.point.x, raw.point.y, raw.point.z);
      const Eigen::Vector3d target = world_from_lidar * source;
      world_points.push_back({target.x(), target.y(), target.z(), raw.point.intensity});
    }
    const Eigen::Vector3d origin_vector = (transform(start_pose) * body_from_lidar_).translation();
    const Point origin{origin_vector.x(), origin_vector.y(), origin_vector.z(), 0.0F};
    const auto result = observer_->process(world_points, origin);
    const double processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - wall_start).count();

    std_msgs::msg::Header header;
    header.stamp = scan.stamp;
    header.frame_id = world_frame_;
    static_pub_->publish(make_cloud(header, result.points, PointLabel::kStatic, true));
    dynamic_pub_->publish(make_cloud(header, result.points, PointLabel::kDynamic, true));
    unknown_pub_->publish(make_cloud(header, result.points, PointLabel::kUnknown, true));
    score_pub_->publish(make_cloud(header, result.points, PointLabel::kUnknown, false));
    publish_statistics(result.stats, processing_ms, queue_residence_ms, scan.end_ns - scan.start_ns);
  }

  static sensor_msgs::msg::PointCloud2 make_cloud(
    const std_msgs::msg::Header & header, const std::vector<LabeledPoint> & points,
    PointLabel selected, bool filter_label)
  {
    std::size_t count = points.size();
    if (filter_label) {
      count = static_cast<std::size_t>(std::count_if(
        points.begin(), points.end(),
        [selected](const LabeledPoint & point) {return point.label == selected;}));
    }
    sensor_msgs::msg::PointCloud2 message;
    message.header = header;
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2Fields(
      5, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "dynamic_score", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(count);
    sensor_msgs::PointCloud2Iterator<float> x(message, "x");
    sensor_msgs::PointCloud2Iterator<float> y(message, "y");
    sensor_msgs::PointCloud2Iterator<float> z(message, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(message, "intensity");
    sensor_msgs::PointCloud2Iterator<float> score(message, "dynamic_score");
    for (const auto & point : points) {
      if (filter_label && point.label != selected) {
        continue;
      }
      *x = static_cast<float>(point.point.x);
      *y = static_cast<float>(point.point.y);
      *z = static_cast<float>(point.point.z);
      *intensity = point.point.intensity;
      *score = point.dynamic_score;
      ++x;
      ++y;
      ++z;
      ++intensity;
      ++score;
    }
    return message;
  }

  void publish_statistics(
    const FilterStats & stats, double processing_ms, double queue_residence_ms,
    std::int64_t scan_duration_ns)
  {
    std_msgs::msg::String statistics;
    std::ostringstream json;
    json << "{\"scan_index\":" << stats.scan_index <<
      ",\"input_points\":" << stats.input_points <<
      ",\"static_points\":" << stats.static_points <<
      ",\"dynamic_points\":" << stats.dynamic_points <<
      ",\"unknown_points\":" << stats.unknown_points <<
      ",\"free_voxels\":" << stats.free_voxels <<
      ",\"allocated_voxels\":" << stats.allocated_voxels <<
      ",\"processing_ms\":" << number(processing_ms) <<
      ",\"queue_residence_ms\":" << number(queue_residence_ms) <<
      ",\"scan_duration_ms\":" << number(static_cast<double>(scan_duration_ns) * 1.0e-6) <<
      ",\"queue_depth\":" << pending_.size() <<
      ",\"queue_overflow\":" << queue_overflow_count_ <<
      ",\"pose_timeout\":" << pose_timeout_count_ <<
      ",\"fastlio_input_modified\":false}";
    statistics.data = json.str();
    statistics_pub_->publish(statistics);

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "uf_dynamic_observer/latency";
    status.hardware_id = "mid360-side-channel";
    status.level = processing_ms <= 100.0 ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = status.level == diagnostic_msgs::msg::DiagnosticStatus::OK ?
      "observer healthy" : "observer latency high";
    status.values.push_back(diagnostic_value("processing_ms", number(processing_ms)));
    status.values.push_back(diagnostic_value("queue_residence_ms", number(queue_residence_ms)));
    status.values.push_back(diagnostic_value("queue_depth", std::to_string(pending_.size())));
    status.values.push_back(diagnostic_value("queue_overflow", std::to_string(queue_overflow_count_)));
    status.values.push_back(diagnostic_value("pose_timeout", std::to_string(pose_timeout_count_)));
    status.values.push_back(diagnostic_value("input_mode", input_mode_));
    status.values.push_back(diagnostic_value("fastlio_input_modified", "false"));
    diagnostics.status.push_back(std::move(status));
    diagnostics_pub_->publish(diagnostics);
  }

  std::string input_mode_;
  std::string world_frame_;
  std::size_t max_pending_scans_{8U};
  double max_pose_wait_ms_{250.0};
  double max_pose_gap_s_{0.15};
  std::uint64_t queue_overflow_count_{0U};
  std::uint64_t pose_timeout_count_{0U};
  Eigen::Isometry3d body_from_lidar_{Eigen::Isometry3d::Identity()};
  std::unique_ptr<ConservativeFreeSpaceObserver> observer_;
  std::deque<PoseSample> poses_;
  std::deque<PendingScan> pending_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr score_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statistics_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr drain_timer_;
};

}  // namespace uf_dynamic_observer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<uf_dynamic_observer::DynamicObserverNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("dynamic_static_map_observer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
