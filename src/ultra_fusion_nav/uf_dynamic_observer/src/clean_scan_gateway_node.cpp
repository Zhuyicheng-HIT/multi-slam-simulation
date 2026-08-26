#include "uf_dynamic_observer/causal_imu_deskew.hpp"
#include "uf_dynamic_observer/clean_scan_admission.hpp"
#include "uf_dynamic_observer/conservative_free_space.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <uf_dynamic_interfaces/msg/previous_fast_lio_state.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
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

std::string number(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::string json_escape(const std::string & value)
{
  std::ostringstream stream;
  for (const unsigned char character : value) {
    switch (character) {
      case '"': stream << "\\\""; break;
      case '\\': stream << "\\\\"; break;
      case '\n': stream << "\\n"; break;
      case '\r': stream << "\\r"; break;
      case '\t': stream << "\\t"; break;
      default:
        if (character < 0x20U) {
          stream << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
                 << static_cast<int>(character) << std::dec << std::setfill(' ');
        } else {
          stream << character;
        }
    }
  }
  return stream.str();
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

}  // namespace

class CleanScanGatewayNode : public rclcpp::Node
{
public:
  CleanScanGatewayNode()
  : Node("clean_scan_gateway")
  {
    const bool enabled = declare_parameter<bool>("enabled", false);
    raw_topic_ = declare_parameter<std::string>("raw_topic", "/livox/lidar");
    clean_topic_ = declare_parameter<std::string>(
      "clean_topic", "/dynamic_observer/clean/livox");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/livox/imu");
    const auto state_topic = declare_parameter<std::string>(
      "previous_state_topic", "/clean_fast_lio/previous_state");
    expected_map_frame_ = declare_parameter<std::string>("expected_map_frame", "camera_init");
    expected_body_frame_ = declare_parameter<std::string>("expected_body_frame", "body");
    max_pending_scans_ = static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<int>("max_pending_scans", 8)));
    max_state_wait_ms_ = declare_parameter<double>("max_state_wait_ms", 250.0);
    max_processing_ms_ = declare_parameter<double>("max_processing_ms", 20.0);

    VisibilityFilterConfig filter_config;
    filter_config.voxel_size_m = declare_parameter<double>("filter.voxel_size_m", 0.25);
    filter_config.min_range_m = declare_parameter<double>("filter.min_range_m", 0.5);
    filter_config.max_range_m = declare_parameter<double>("filter.max_range_m", 35.0);
    filter_config.free_confirmations = static_cast<std::uint16_t>(std::max<std::int64_t>(
      1, declare_parameter<int>("filter.free_confirmations", 4)));
    filter_config.static_confirmations = static_cast<std::uint16_t>(std::max<std::int64_t>(
      1, declare_parameter<int>("filter.static_confirmations", 2)));
    filter_config.occupied_recovery = static_cast<std::uint16_t>(std::max<std::int64_t>(
      1, declare_parameter<int>("filter.occupied_recovery", 20)));
    filter_config.endpoint_guard_voxels = std::max<std::int64_t>(
      0, declare_parameter<int>("filter.endpoint_guard_voxels", 1));
    filter_config.dynamic_growth_voxels = std::max<std::int64_t>(
      0, declare_parameter<int>("filter.dynamic_growth_voxels", 1));
    filter_config.ray_stride = std::max<std::int64_t>(
      1, declare_parameter<int>("filter.ray_stride", 1));
    filter_config.max_voxels = static_cast<std::size_t>(std::max<std::int64_t>(
      1000, declare_parameter<int>("filter.max_voxels", 1500000)));
    filter_config.dynamic_confirmations = static_cast<std::uint16_t>(
      std::max<std::int64_t>(1, declare_parameter<int>("filter.dynamic_confirmations", 1)));
    filter_config.dynamic_hold_scans = static_cast<std::uint16_t>(
      std::max<std::int64_t>(1, declare_parameter<int>("filter.dynamic_hold_scans", 12)));
    filter_config.vacated_hold_scans = static_cast<std::uint16_t>(
      std::max<std::int64_t>(1, declare_parameter<int>("filter.vacated_hold_scans", 8)));
    filter_config.static_vacate_confirmations = static_cast<std::uint16_t>(
      std::max<std::int64_t>(1, declare_parameter<int>("filter.static_vacate_confirmations", 1)));
    filter_config.dynamic_track_radius_voxels = std::max<std::int64_t>(
      0, declare_parameter<int>("filter.dynamic_track_radius_voxels", 1));
    filter_config.vacated_surface_radius_voxels = std::max<std::int64_t>(
      0, declare_parameter<int>("filter.vacated_surface_radius_voxels", 1));
    filter_config.static_support_radius_voxels = std::max<std::int64_t>(
      0, declare_parameter<int>("filter.static_support_radius_voxels", 1));
    filter_config.min_static_neighbor_voxels = static_cast<std::size_t>(
      std::max<std::int64_t>(0, declare_parameter<int>("filter.min_static_neighbor_voxels", 0)));
    filter_config.far_range_m = declare_parameter<double>("filter.far_range_m", 15.0);
    filter_config.far_static_confirmations = static_cast<std::uint16_t>(
      std::max<std::int64_t>(1, declare_parameter<int>("filter.far_static_confirmations", 12)));
    min_range_m_ = filter_config.min_range_m;
    max_range_m_ = filter_config.max_range_m;
    observer_ = std::make_unique<VisibilityAwareDynamicObserver>(filter_config);

    CausalDeskewConfig deskew_config;
    deskew_config.max_imu_gap_s = declare_parameter<double>("deskew.max_imu_gap_s", 0.025);
    max_imu_gap_s_ = deskew_config.max_imu_gap_s;
    deskew_config.max_prediction_horizon_s = declare_parameter<double>(
      "deskew.max_prediction_horizon_s", 0.20);
    max_prediction_horizon_s_ = deskew_config.max_prediction_horizon_s;
    const auto gravity = declare_parameter<std::vector<double>>(
      "deskew.gravity_world", {0.0, 0.0, -9.80665});
    if (gravity.size() != 3U) {
      throw std::invalid_argument("deskew.gravity_world must contain three values");
    }
    deskew_config.gravity_world = {gravity[0], gravity[1], gravity[2]};
    deskew_ = std::make_unique<CausalImuDeskew>(deskew_config);

    const auto translation = declare_parameter<std::vector<double>>(
      "extrinsic.body_from_lidar_translation", {0.0, 0.0, 0.0});
    const auto quaternion = declare_parameter<std::vector<double>>(
      "extrinsic.body_from_lidar_quaternion_xyzw", {0.0, 0.0, 0.0, 1.0});
    if (translation.size() != 3U || quaternion.size() != 4U) {
      throw std::invalid_argument("LiDAR extrinsic parameters must have 3 and 4 elements");
    }
    Eigen::Quaterniond rotation(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
    if (rotation.norm() < 1.0e-9) {
      throw std::invalid_argument("LiDAR extrinsic quaternion is invalid");
    }
    rotation.normalize();
    body_from_lidar_ = Eigen::Isometry3d::Identity();
    body_from_lidar_.linear() = rotation.toRotationMatrix();
    body_from_lidar_.translation() = Eigen::Vector3d(
      translation[0], translation[1], translation[2]);

    clean_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
      clean_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/dynamic_observer/clean/status", rclcpp::QoS(20).reliable());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/dynamic_observer/clean/diagnostics", rclcpp::QoS(20).reliable());

    if (!enabled) {
      RCLCPP_INFO(
        get_logger(),
        "Clean Scan Gateway disabled: no raw LiDAR/IMU/state subscriptions were created.");
      return;
    }

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(64);
    state_sub_ = create_subscription<uf_dynamic_interfaces::msg::PreviousFastLioState>(
      state_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      [this](uf_dynamic_interfaces::msg::PreviousFastLioState::ConstSharedPtr message) {
        on_state(*message);
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, sensor_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {on_imu(*message);});
    raw_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      raw_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {on_raw(*message);});
    drain_timer_ = create_wall_timer(std::chrono::milliseconds(2), [this]() {drain_pending();});
    RCLCPP_WARN(
      get_logger(),
      "Clean Scan Gateway enabled as an opt-in candidate: raw=%s clean=%s state=%s. "
      "Production /livox/lidar remains untouched.",
      raw_topic_.c_str(), clean_topic_.c_str(), state_topic.c_str());
  }

private:
  struct PreviousState
  {
    CausalPose pose;
    std::uint64_t scan_sequence{0U};
    std::uint32_t reset_counter{0U};
  };

  struct PendingScan
  {
    livox_ros_driver2::msg::CustomMsg raw;
    std::int64_t start_ns{0};
    std::int64_t end_ns{0};
    std::chrono::steady_clock::time_point arrival;
  };

  void on_state(const uf_dynamic_interfaces::msg::PreviousFastLioState & message)
  {
    const auto state_stamp_ns = stamp_ns(message.header.stamp);
    if (!message.valid || state_stamp_ns <= 0 ||
      message.map_frame != expected_map_frame_ || message.body_frame != expected_body_frame_)
    {
      ++invalid_state_count_;
      return;
    }
    PreviousState state;
    state.pose.stamp_ns = state_stamp_ns;
    state.pose.position = {message.position[0], message.position[1], message.position[2]};
    state.pose.velocity = {
      message.velocity_map[0], message.velocity_map[1], message.velocity_map[2]};
    state.pose.orientation = Eigen::Quaterniond(
      message.orientation_xyzw[3], message.orientation_xyzw[0],
      message.orientation_xyzw[1], message.orientation_xyzw[2]);
    state.pose.has_calibrated_bias = true;
    state.pose.accel_bias = {
      message.accel_bias[0], message.accel_bias[1], message.accel_bias[2]};
    state.pose.gyro_bias = {
      message.gyro_bias[0], message.gyro_bias[1], message.gyro_bias[2]};
    state.scan_sequence = message.scan_sequence;
    state.reset_counter = message.reset_counter;
    if (!state.pose.position.allFinite() || !state.pose.velocity.allFinite() ||
      !state.pose.accel_bias.allFinite() || !state.pose.gyro_bias.allFinite() ||
      state.pose.orientation.norm() < 1.0e-9)
    {
      ++invalid_state_count_;
      return;
    }
    state.pose.orientation.normalize();
    if (!states_.empty() && state.pose.stamp_ns <= states_.back().pose.stamp_ns) {
      ++state_regression_count_;
      return;
    }
    if (epoch_initialized_ && state.reset_counter != current_reset_counter_) {
      while (!pending_.empty()) {
        fail_open(std::move(pending_.front()), "state_epoch_change", 0.0);
        pending_.pop_front();
      }
      states_.clear();
      imu_.clear();
      observer_->reset();
    }
    epoch_initialized_ = true;
    current_reset_counter_ = state.reset_counter;
    states_.push_back(std::move(state));
    while (states_.size() > 256U) {
      states_.pop_front();
    }
  }

  void on_imu(const sensor_msgs::msg::Imu & message)
  {
    CausalImuSample sample;
    sample.stamp_ns = stamp_ns(message.header.stamp);
    sample.linear_acceleration = {
      message.linear_acceleration.x, message.linear_acceleration.y,
      message.linear_acceleration.z};
    sample.angular_velocity = {
      message.angular_velocity.x, message.angular_velocity.y,
      message.angular_velocity.z};
    if (sample.stamp_ns <= 0 || !sample.linear_acceleration.allFinite() ||
      !sample.angular_velocity.allFinite() ||
      (!imu_.empty() && sample.stamp_ns <= imu_.back().stamp_ns))
    {
      ++imu_regression_count_;
      return;
    }
    imu_.push_back(sample);
    while (imu_.size() > 4000U) {
      imu_.pop_front();
    }
  }

  void on_raw(const livox_ros_driver2::msg::CustomMsg & message)
  {
    PendingScan scan;
    scan.raw = message;
    scan.start_ns = stamp_ns(message.header.stamp);
    scan.end_ns = scan.start_ns;
    scan.arrival = std::chrono::steady_clock::now();
    for (const auto & point : message.points) {
      scan.end_ns = std::max(
        scan.end_ns, scan.start_ns + static_cast<std::int64_t>(point.offset_time));
    }
    if (scan.start_ns <= 0 ||
      (last_raw_stamp_ns_ > 0 && scan.start_ns <= last_raw_stamp_ns_))
    {
      ++input_regression_count_;
      fail_open(std::move(scan), "input_timestamp_regression", 0.0);
      return;
    }
    last_raw_stamp_ns_ = scan.start_ns;
    if (scan.raw.points.empty()) {
      fail_open(std::move(scan), "empty_raw_scan", 0.0);
      return;
    }
    if (pending_.size() >= max_pending_scans_) {
      ++queue_overflow_count_;
      fail_open(std::move(pending_.front()), "queue_overflow", 0.0);
      pending_.pop_front();
    }
    pending_.push_back(std::move(scan));
  }

  void drain_pending()
  {
    while (!pending_.empty()) {
      const double residence_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - pending_.front().arrival).count();
      const bool expired = residence_ms > max_state_wait_ms_;
      auto anchor = std::upper_bound(
        states_.begin(), states_.end(), pending_.front().start_ns,
        [](std::int64_t stamp, const PreviousState & state) {
          return stamp < state.pose.stamp_ns;
        });
      const bool anchor_ready = anchor != states_.begin();
      const auto terminal_gap_ns = static_cast<std::int64_t>(max_imu_gap_s_ * 1.0e9);
      const bool imu_ready = !imu_.empty() &&
        imu_.back().stamp_ns + terminal_gap_ns >= pending_.front().end_ns;
      if (!anchor_ready || !imu_ready) {
        if (!expired) {
          return;
        }
        ++pose_timeout_count_;
        PendingScan scan = std::move(pending_.front());
        pending_.pop_front();
        fail_open(
          std::move(scan), anchor_ready ? "imu_coverage_timeout" : "previous_state_timeout",
          residence_ms);
        continue;
      }
      --anchor;
      const double prediction_horizon_s =
        static_cast<double>(pending_.front().end_ns - anchor->pose.stamp_ns) * 1.0e-9;
      if (prediction_horizon_s > max_prediction_horizon_s_) {
        // A newer causal previous-scan posterior may still arrive. Wait within
        // the existing bounded queue contract instead of converting callback
        // scheduling order into an immediate deskew rejection. A state at or
        // after the current scan is never selected.
        if (!expired) {
          return;
        }
        ++pose_timeout_count_;
        PendingScan scan = std::move(pending_.front());
        pending_.pop_front();
        fail_open(std::move(scan), "previous_state_stale_timeout", residence_ms);
        continue;
      }
      PendingScan scan = std::move(pending_.front());
      pending_.pop_front();
      try {
        process_scan(scan, *anchor, residence_ms);
      } catch (const std::exception & error) {
        ++internal_exception_count_;
        fail_open(std::move(scan), std::string("internal_exception:") + error.what(), residence_ms);
      } catch (...) {
        ++internal_exception_count_;
        fail_open(std::move(scan), "internal_exception:unknown", residence_ms);
      }
    }
  }

  void process_scan(PendingScan scan, const PreviousState & anchor, double residence_ms)
  {
    std::vector<std::pair<std::int64_t, std::size_t>> queries;
    queries.reserve(scan.raw.points.size());
    for (std::size_t index = 0U; index < scan.raw.points.size(); ++index) {
      queries.emplace_back(
        scan.start_ns + static_cast<std::int64_t>(scan.raw.points[index].offset_time), index);
    }
    std::sort(queries.begin(), queries.end());
    std::vector<std::int64_t> query_stamps;
    query_stamps.reserve(queries.size());
    for (const auto & query : queries) {
      query_stamps.push_back(query.first);
    }
    std::vector<CausalImuSample> imu_samples;
    for (const auto & sample : imu_) {
      if (sample.stamp_ns <= scan.end_ns) {
        imu_samples.push_back(sample);
      } else {
        break;
      }
    }
    const auto trajectory = deskew_->propagate(anchor.pose, imu_samples, query_stamps);
    if (!trajectory.valid || trajectory.poses.size() != queries.size()) {
      ++deskew_reject_count_;
      fail_open(std::move(scan), "deskew:" + trajectory.reason, residence_ms);
      return;
    }

    const auto processing_start = std::chrono::steady_clock::now();
    std::vector<Point> classified_world_points;
    std::vector<std::size_t> classified_source_indices;
    classified_world_points.reserve(scan.raw.points.size());
    classified_source_indices.reserve(scan.raw.points.size());
    Eigen::Isometry3d first_world_from_body = Eigen::Isometry3d::Identity();
    first_world_from_body.linear() = trajectory.poses.front().orientation.toRotationMatrix();
    first_world_from_body.translation() = trajectory.poses.front().position;
    const Eigen::Vector3d origin_vector = (first_world_from_body * body_from_lidar_).translation();
    const Point sensor_origin{origin_vector.x(), origin_vector.y(), origin_vector.z(), 0.0F};

    for (std::size_t ordered_index = 0U; ordered_index < queries.size(); ++ordered_index) {
      const auto source_index = queries[ordered_index].second;
      const auto & source = scan.raw.points[source_index];
      if (!std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(source.z)) {
        continue;
      }
      Eigen::Isometry3d world_from_body = Eigen::Isometry3d::Identity();
      world_from_body.linear() = trajectory.poses[ordered_index].orientation.toRotationMatrix();
      world_from_body.translation() = trajectory.poses[ordered_index].position;
      const Eigen::Vector3d world = world_from_body * body_from_lidar_ *
        Eigen::Vector3d(source.x, source.y, source.z);
      const double range = (world - origin_vector).norm();
      if (range < min_range_m_ || range > max_range_m_) {
        continue;
      }
      classified_world_points.push_back({
        world.x(), world.y(), world.z(), static_cast<float>(source.reflectivity)});
      classified_source_indices.push_back(source_index);
    }

    const auto filter_result = observer_->process(classified_world_points, sensor_origin);
    const auto admission = admission_.apply(
      scan.raw.points.size(), classified_source_indices, filter_result.points);
    const double processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - processing_start).count();
    if (!admission.healthy) {
      fail_open(std::move(scan), "admission:" + admission.reason, residence_ms, processing_ms);
      return;
    }
    if (processing_ms > max_processing_ms_) {
      ++latency_fail_open_count_;
      fail_open(std::move(scan), "observer_latency_exceeded", residence_ms, processing_ms);
      return;
    }

    auto clean = scan.raw;
    clean.points.clear();
    clean.points.reserve(scan.raw.points.size() - admission.dynamic_removed);
    for (std::size_t index = 0U; index < scan.raw.points.size(); ++index) {
      if (admission.keep[index]) {
        clean.points.push_back(scan.raw.points[index]);
      }
    }
    clean.point_num = static_cast<std::uint32_t>(clean.points.size());
    clean_pub_->publish(clean);
    ++clean_scan_count_;
    removed_point_count_ += admission.dynamic_removed;
    publish_status(
      scan, false, "ok", residence_ms, processing_ms, scan.raw.points.size(),
      clean.points.size(), admission.dynamic_removed, admission.static_points,
      admission.unknown_points, anchor);
  }

  void fail_open(
    PendingScan scan, const std::string & reason, double residence_ms,
    double processing_ms = 0.0)
  {
    clean_pub_->publish(scan.raw);
    ++fail_open_scan_count_;
    PreviousState empty_anchor;
    publish_status(
      scan, true, reason, residence_ms, processing_ms, scan.raw.points.size(),
      scan.raw.points.size(), 0U, 0U, scan.raw.points.size(), empty_anchor);
  }

  void publish_status(
    const PendingScan & scan, bool fail_open, const std::string & reason,
    double residence_ms, double processing_ms, std::size_t raw_points,
    std::size_t output_points, std::size_t removed_points, std::size_t static_points,
    std::size_t unknown_points, const PreviousState & anchor)
  {
    std_msgs::msg::String status;
    std::ostringstream json;
    json << "{\"source_stamp_ns\":" << scan.start_ns <<
      ",\"healthy\":" << (fail_open ? "false" : "true") <<
      ",\"degraded\":" << (fail_open ? "true" : "false") <<
      ",\"fail_open\":" << (fail_open ? "true" : "false") <<
      ",\"reason\":\"" << json_escape(reason) << "\"" <<
      ",\"raw_points\":" << raw_points <<
      ",\"output_points\":" << output_points <<
      ",\"dynamic_removed\":" << removed_points <<
      ",\"static_points\":" << static_points <<
      ",\"unknown_points\":" << unknown_points <<
      ",\"processing_ms\":" << number(processing_ms) <<
      ",\"queue_residence_ms\":" << number(residence_ms) <<
      ",\"queue_depth\":" << pending_.size() <<
      ",\"queue_overflow\":" << queue_overflow_count_ <<
      ",\"pose_timeout\":" << pose_timeout_count_ <<
      ",\"deskew_reject\":" << deskew_reject_count_ <<
      ",\"input_regression\":" << input_regression_count_ <<
      ",\"imu_regression\":" << imu_regression_count_ <<
      ",\"state_regression\":" << state_regression_count_ <<
      ",\"internal_exception\":" << internal_exception_count_ <<
      ",\"clean_scans\":" << clean_scan_count_ <<
      ",\"fail_open_scans\":" << fail_open_scan_count_ <<
      ",\"anchor_scan_sequence\":" << anchor.scan_sequence <<
      ",\"anchor_reset_counter\":" << anchor.reset_counter <<
      ",\"raw_topic_unchanged\":true} ";
    status.data = json.str();
    status_pub_->publish(status);

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    diagnostic.name = "uf_dynamic_observer/clean_scan_gateway";
    diagnostic.hardware_id = "mid360-clean-gateway";
    diagnostic.level = fail_open ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostic.message = fail_open ? "degraded fail-open raw passthrough" : "clean scan healthy";
    diagnostic.values.push_back(key_value("reason", reason));
    diagnostic.values.push_back(key_value("processing_ms", number(processing_ms)));
    diagnostic.values.push_back(key_value("queue_residence_ms", number(residence_ms)));
    diagnostic.values.push_back(key_value("raw_points", std::to_string(raw_points)));
    diagnostic.values.push_back(key_value("output_points", std::to_string(output_points)));
    diagnostic.values.push_back(key_value("dynamic_removed", std::to_string(removed_points)));
    diagnostic.values.push_back(key_value("static_points", std::to_string(static_points)));
    diagnostic.values.push_back(key_value("unknown_points", std::to_string(unknown_points)));
    diagnostic.values.push_back(key_value("raw_topic_unchanged", "true"));
    diagnostics.status.push_back(std::move(diagnostic));
    diagnostics_pub_->publish(diagnostics);
  }

  std::string raw_topic_;
  std::string clean_topic_;
  std::string expected_map_frame_;
  std::string expected_body_frame_;
  std::size_t max_pending_scans_{8U};
  double max_state_wait_ms_{250.0};
  double max_processing_ms_{20.0};
  double max_imu_gap_s_{0.025};
  double max_prediction_horizon_s_{0.20};
  double min_range_m_{0.5};
  double max_range_m_{35.0};
  std::int64_t last_raw_stamp_ns_{0};
  bool epoch_initialized_{false};
  std::uint32_t current_reset_counter_{0U};
  std::uint64_t clean_scan_count_{0U};
  std::uint64_t fail_open_scan_count_{0U};
  std::uint64_t removed_point_count_{0U};
  std::uint64_t queue_overflow_count_{0U};
  std::uint64_t pose_timeout_count_{0U};
  std::uint64_t deskew_reject_count_{0U};
  std::uint64_t input_regression_count_{0U};
  std::uint64_t imu_regression_count_{0U};
  std::uint64_t state_regression_count_{0U};
  std::uint64_t invalid_state_count_{0U};
  std::uint64_t internal_exception_count_{0U};
  std::uint64_t latency_fail_open_count_{0U};
  Eigen::Isometry3d body_from_lidar_{Eigen::Isometry3d::Identity()};
  CleanScanAdmission admission_;
  std::unique_ptr<VisibilityAwareDynamicObserver> observer_;
  std::unique_ptr<CausalImuDeskew> deskew_;
  std::deque<PreviousState> states_;
  std::deque<CausalImuSample> imu_;
  std::deque<PendingScan> pending_;

  rclcpp::Subscription<uf_dynamic_interfaces::msg::PreviousFastLioState>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr raw_sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr clean_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr drain_timer_;
};

}  // namespace uf_dynamic_observer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<uf_dynamic_observer::CleanScanGatewayNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("clean_scan_gateway"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
