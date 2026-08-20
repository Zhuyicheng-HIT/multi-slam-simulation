#include "uf_dynamic_observer/long_term_static_map.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <uf_dynamic_interfaces/msg/previous_fast_lio_state.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace uf_dynamic_observer
{
namespace
{

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

std::string number(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

diagnostic_msgs::msg::KeyValue pair(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

}  // namespace

class LongTermStaticMapNode : public rclcpp::Node
{
public:
  LongTermStaticMapNode()
  : Node("long_term_static_map_refinement")
  {
    enabled_ = declare_parameter<bool>("enabled", false);
    const auto scored_topic = declare_parameter<std::string>(
      "scored_cloud_topic", "/dynamic_observer/scored_cloud");
    const auto previous_state_topic = declare_parameter<std::string>(
      "previous_state_topic", "/clean_fast_lio/previous_state");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/mapping/long_term_static/points");
    relocalization_topic_ = declare_parameter<std::string>(
      "relocalization_topic", "/mapping/long_term_static/relocalization_points");
    loop_closure_topic_ = declare_parameter<std::string>(
      "loop_closure_topic", "/mapping/long_term_static/loop_closure_points");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/mapping/long_term_static/status");
    expected_frame_id_ = declare_parameter<std::string>("expected_frame_id", "");
    maximum_state_age_s_ = declare_parameter<double>("maximum_previous_state_age_s", 0.30);
    maximum_state_history_ = static_cast<std::size_t>(std::max<std::int64_t>(
      4, declare_parameter<int>("maximum_state_history", 128)));
    const double publish_period_s = declare_parameter<double>("publish_period_s", 1.0);
    semantic_enabled_ = declare_parameter<bool>("semantic_auxiliary.enabled", false);
    semantic_shadow_only_ = declare_parameter<bool>("semantic_auxiliary.shadow_only", true);
    const auto semantic_topic = declare_parameter<std::string>(
      "semantic_auxiliary.topic", "/semantic/dynamic_evidence");

    LongTermMapConfig config;
    config.voxel_size_m = declare_parameter<double>("map.voxel_size_m", 0.25);
    config.min_range_m = declare_parameter<double>("map.min_range_m", 0.5);
    config.max_range_m = declare_parameter<double>("map.max_range_m", 35.0);
    config.static_candidate_observations = positive_u16(
      declare_parameter<int>("map.static_candidate_observations", 2));
    config.static_confirmed_observations = positive_u16(
      declare_parameter<int>("map.static_confirmed_observations", 6));
    config.static_confirmed_duration_s = declare_parameter<double>(
      "map.static_confirmed_duration_s", 1.0);
    config.static_confirmed_view_bins = positive_u8(
      declare_parameter<int>("map.static_confirmed_view_bins", 2));
    config.static_consistency_ratio = declare_parameter<double>(
      "map.static_consistency_ratio", 0.65);
    config.candidate_free_contradictions = positive_u16(
      declare_parameter<int>("map.candidate_free_contradictions", 2));
    config.dynamic_candidate_free_traversals = positive_u16(
      declare_parameter<int>("map.dynamic_candidate_free_traversals", 3));
    config.dynamic_confirmed_free_traversals = positive_u16(
      declare_parameter<int>("map.dynamic_confirmed_free_traversals", 6));
    config.dynamic_confirmed_view_bins = positive_u8(
      declare_parameter<int>("map.dynamic_confirmed_view_bins", 2));
    config.dynamic_confirmed_duration_s = declare_parameter<double>(
      "map.dynamic_confirmed_duration_s", 0.4);
    config.dynamic_label_confirmations = positive_u16(
      declare_parameter<int>("map.dynamic_label_confirmations", 2));
    config.dynamic_recovery_static_observations = positive_u16(
      declare_parameter<int>("map.dynamic_recovery_static_observations", 12));
    config.dynamic_recovery_duration_s = declare_parameter<double>(
      "map.dynamic_recovery_duration_s", 2.0);
    config.far_static_confirmed_observations = positive_u16(
      declare_parameter<int>("map.far_static_confirmed_observations", 60));
    config.far_static_confirmed_duration_s = declare_parameter<double>(
      "map.far_static_confirmed_duration_s", 15.0);
    config.far_static_confirmed_view_bins = positive_u8(
      declare_parameter<int>("map.far_static_confirmed_view_bins", 6));
    config.far_range_m = declare_parameter<double>("map.far_range_m", 12.0);
    config.endpoint_guard_voxels = std::max(
      0, static_cast<int>(declare_parameter<int>("map.endpoint_guard_voxels", 1)));
    config.ray_stride = std::max(
      1, static_cast<int>(declare_parameter<int>("map.ray_stride", 2)));
    config.max_voxels = static_cast<std::size_t>(std::max<std::int64_t>(
      1000, declare_parameter<int>("map.max_voxels", 1500000)));
    config.stale_dynamic_after_scans = static_cast<std::uint64_t>(
      std::max<std::int64_t>(1, declare_parameter<int>("map.stale_dynamic_after_scans", 1200)));
    config.semantic_dynamic_threshold = static_cast<float>(declare_parameter<double>(
      "semantic_auxiliary.dynamic_threshold", 0.70));
    map_ = std::make_unique<LongTermStaticMap>(config);

    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, output_qos);
    relocalization_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      relocalization_topic_, output_qos);
    loop_closure_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      loop_closure_topic_, output_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(20).reliable());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/mapping/long_term_static/diagnostics", rclcpp::QoS(20).reliable());

    if (!enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "Long-term static refinement is disabled; no map or relocalization input is changed.");
      return;
    }
    state_sub_ = create_subscription<uf_dynamic_interfaces::msg::PreviousFastLioState>(
      previous_state_topic, rclcpp::QoS(128).reliable(),
      [this](uf_dynamic_interfaces::msg::PreviousFastLioState::ConstSharedPtr message) {
        if (!message->valid) {
          return;
        }
        states_.push_back(*message);
        while (states_.size() > maximum_state_history_) {
          states_.pop_front();
        }
      });
    scored_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      scored_topic, rclcpp::SensorDataQoS().keep_last(4),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {on_scored_cloud(*message);});
    if (semantic_enabled_) {
      semantic_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        semantic_topic, rclcpp::SensorDataQoS().keep_last(4),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {on_semantic(*message);});
    }
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(publish_period_s)),
      [this]() {publish_map();});
    RCLCPP_INFO(
      get_logger(),
      "Long-term refinement enabled. Only STATIC_CONFIRMED is published; production LiDAR is untouched.");
  }

private:
  static std::uint16_t positive_u16(int value)
  {
    return static_cast<std::uint16_t>(std::max(1, value));
  }

  static std::uint8_t positive_u8(int value)
  {
    return static_cast<std::uint8_t>(std::clamp(value, 1, 64));
  }

  const uf_dynamic_interfaces::msg::PreviousFastLioState * previous_state(double stamp_s) const
  {
    const uf_dynamic_interfaces::msg::PreviousFastLioState * selected = nullptr;
    for (auto it = states_.rbegin(); it != states_.rend(); ++it) {
      const double candidate_s = stamp_seconds(it->header.stamp);
      // Strictly previous: a state completed at or after this cloud cannot be
      // used. This keeps the refinement causal even though it is downstream.
      if (candidate_s < stamp_s) {
        selected = &*it;
        break;
      }
    }
    if (selected == nullptr || stamp_s - stamp_seconds(selected->header.stamp) >
      maximum_state_age_s_)
    {
      return nullptr;
    }
    return selected;
  }

  void on_scored_cloud(const sensor_msgs::msg::PointCloud2 & message)
  {
    const auto wall_start = std::chrono::steady_clock::now();
    const double stamp_s = stamp_seconds(message.header.stamp);
    if (!expected_frame_id_.empty() && message.header.frame_id != expected_frame_id_) {
      ++frame_rejections_;
      publish_status("DEGRADED", "frame_mismatch", 0.0);
      return;
    }
    const auto * state = previous_state(stamp_s);
    if (state == nullptr) {
      ++state_wait_rejections_;
      // Fail-open at the system boundary: retain the last good long-term map,
      // publish an explicit hold, and never block any scan or estimator.
      publish_status("DEGRADED", "previous_state_unavailable_map_held", 0.0);
      return;
    }
    std::vector<LabeledPoint> observations;
    observations.reserve(static_cast<std::size_t>(message.width) * message.height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
      sensor_msgs::PointCloud2ConstIterator<float> intensity(message, "intensity");
      sensor_msgs::PointCloud2ConstIterator<float> score(message, "dynamic_score");
      for (; x != x.end(); ++x, ++y, ++z, ++intensity, ++score) {
        LabeledPoint observation;
        observation.point = {*x, *y, *z, *intensity};
        observation.dynamic_score = *score;
        observation.label = *score >= 0.60F ? PointLabel::kDynamic :
          (*score <= 0.25F ? PointLabel::kStatic : PointLabel::kUnknown);
        observations.push_back(observation);
      }
    } catch (const std::exception & error) {
      ++decode_rejections_;
      publish_status("DEGRADED", std::string("cloud_decode_error:") + error.what(), 0.0);
      return;
    }
    const Point origin{
      state->position[0], state->position[1], state->position[2], 0.0F};
    const auto update = map_->integrate(observations, origin, stamp_s);
    last_frame_id_ = message.header.frame_id;
    last_stamp_ = message.header.stamp;
    last_update_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - wall_start).count();
    publish_status(
      update.accepted ? "HEALTHY" : "DEGRADED", update.reason, last_update_ms_);
  }

  void on_semantic(const sensor_msgs::msg::PointCloud2 & message)
  {
    const double stamp_s = stamp_seconds(message.header.stamp);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
      sensor_msgs::PointCloud2ConstIterator<float> confidence(message, "dynamic_confidence");
      for (; x != x.end(); ++x, ++y, ++z, ++confidence) {
        map_->add_semantic_evidence(
          {*x, *y, *z, 0.0F}, *confidence, stamp_s, semantic_shadow_only_);
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "semantic shadow input rejected: %s", error.what());
    }
  }

  sensor_msgs::msg::PointCloud2 make_map_cloud() const
  {
    const auto points = map_->static_confirmed_points();
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp = last_stamp_;
    message.header.frame_id = last_frame_id_;
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2Fields(
      6, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
      "static_confidence", 1, sensor_msgs::msg::PointField::FLOAT32,
      "support", 1, sensor_msgs::msg::PointField::UINT32);
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(message, "x");
    sensor_msgs::PointCloud2Iterator<float> y(message, "y");
    sensor_msgs::PointCloud2Iterator<float> z(message, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(message, "intensity");
    sensor_msgs::PointCloud2Iterator<float> confidence(message, "static_confidence");
    sensor_msgs::PointCloud2Iterator<std::uint32_t> support(message, "support");
    for (const auto & point : points) {
      *x = static_cast<float>(point.point.x);
      *y = static_cast<float>(point.point.y);
      *z = static_cast<float>(point.point.z);
      *intensity = point.point.intensity;
      *confidence = point.confidence;
      *support = point.support;
      ++x;
      ++y;
      ++z;
      ++intensity;
      ++confidence;
      ++support;
    }
    return message;
  }

  void publish_map()
  {
    if (last_frame_id_.empty()) {
      return;
    }
    const auto message = make_map_cloud();
    // These are three explicit admissions of the exact same immutable
    // STATIC_CONFIRMED snapshot. Candidates, UNKNOWN and dynamic tombstones
    // cannot reach long-term relocalization or loop-closure data paths.
    map_pub_->publish(message);
    relocalization_pub_->publish(message);
    loop_closure_pub_->publish(message);
  }

  void publish_status(const std::string & health, const std::string & reason, double update_ms)
  {
    const auto statistics = map_->stats();
    std_msgs::msg::String message;
    std::ostringstream json;
    json << "{\"health\":\"" << health << "\",\"reason\":\"" << reason <<
      "\",\"enabled\":true,\"production_lidar_modified\":false" <<
      ",\"future_pose_used\":false,\"output_policy\":\"STATIC_CONFIRMED_ONLY\"" <<
      ",\"scan_index\":" << statistics.scan_index <<
      ",\"allocated_voxels\":" << statistics.allocated_voxels <<
      ",\"static_confirmed_voxels\":" << statistics.static_confirmed_voxels <<
      ",\"static_candidate_voxels\":" << statistics.static_candidate_voxels <<
      ",\"dynamic_confirmed_voxels\":" << statistics.dynamic_confirmed_voxels <<
      ",\"unknown_voxels\":" << statistics.unknown_voxels <<
      ",\"removed_ghost_voxels\":" << statistics.removed_ghost_voxels <<
      ",\"mean_admission_delay_s\":" << number(statistics.mean_admission_delay_s) <<
      ",\"promoted_static_ratio\":" << number(statistics.promoted_static_ratio, 6) <<
      ",\"permanent_rejection_ratio\":" << number(
        statistics.permanent_rejection_ratio, 6) <<
      ",\"semantic_shadow_hits\":" << statistics.semantic_shadow_hits <<
      ",\"capacity_rejected_voxels\":" << statistics.capacity_rejected_voxels <<
      ",\"update_ms\":" << number(update_ms) <<
      ",\"state_wait_rejections\":" << state_wait_rejections_ <<
      ",\"frame_rejections\":" << frame_rejections_ <<
      ",\"decode_rejections\":" << decode_rejections_ << "}";
    message.data = json.str();
    status_pub_->publish(message);

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "uf_dynamic_observer/long_term_static_map";
    status.hardware_id = "project-owned-long-term-map";
    status.level = health == "HEALTHY" ?
      diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = reason;
    status.values.push_back(pair("update_ms", number(update_ms)));
    status.values.push_back(pair(
      "static_confirmed_voxels", std::to_string(statistics.static_confirmed_voxels)));
    status.values.push_back(pair(
      "removed_ghost_voxels", std::to_string(statistics.removed_ghost_voxels)));
    status.values.push_back(pair("production_lidar_modified", "false"));
    status.values.push_back(pair("future_pose_used", "false"));
    status.values.push_back(pair("semantic_shadow_only", semantic_shadow_only_ ? "true" : "false"));
    diagnostics.status.push_back(std::move(status));
    diagnostics_pub_->publish(diagnostics);
  }

  bool enabled_{false};
  bool semantic_enabled_{false};
  bool semantic_shadow_only_{true};
  double maximum_state_age_s_{0.30};
  std::size_t maximum_state_history_{128U};
  std::string output_topic_;
  std::string relocalization_topic_;
  std::string loop_closure_topic_;
  std::string status_topic_;
  std::string expected_frame_id_;
  std::string last_frame_id_;
  builtin_interfaces::msg::Time last_stamp_;
  double last_update_ms_{0.0};
  std::uint64_t state_wait_rejections_{0U};
  std::uint64_t frame_rejections_{0U};
  std::uint64_t decode_rejections_{0U};
  std::unique_ptr<LongTermStaticMap> map_;
  std::deque<uf_dynamic_interfaces::msg::PreviousFastLioState> states_;
  rclcpp::Subscription<uf_dynamic_interfaces::msg::PreviousFastLioState>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scored_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr semantic_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr relocalization_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loop_closure_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace uf_dynamic_observer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<uf_dynamic_observer::LongTermStaticMapNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("long_term_static_map_refinement"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
