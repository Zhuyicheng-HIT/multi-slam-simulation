#include "uf_relocalization/descriptor_core.hpp"
#include "uf_relocalization/keyframe_database.hpp"
#include "uf_relocalization/keyframe_synchronization.hpp"
#include "uf_relocalization/multi_frame_consistency_gate.hpp"
#include "uf_relocalization/registration_core.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "uf_interfaces/msg/lio_diagnostics.hpp"
#include "uf_interfaces/msg/reliability_score.hpp"
#include "uf_interfaces/msg/relocalization_result.hpp"
#include "uf_interfaces/msg/scheduler_state.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace uf_relocalization
{
namespace
{

using ResultMessage = uf_interfaces::msg::RelocalizationResult;
using CloudMessage = sensor_msgs::msg::PointCloud2;

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1.0e-9;
}

std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

Eigen::Isometry3d pose_to_isometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  if (quaternion.norm() <= 1.0e-12) {
    quaternion = Eigen::Quaterniond::Identity();
  } else {
    quaternion.normalize();
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

geometry_msgs::msg::Pose isometry_to_pose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Quaterniond quaternion(transform.rotation());
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

double rotation_angle(const Eigen::Matrix3d & rotation)
{
  return Eigen::AngleAxisd(rotation).angle();
}

double tilt_difference(
  const Eigen::Matrix3d & first, const Eigen::Matrix3d & second)
{
  const double cosine = std::clamp(
    first.col(2).dot(second.col(2)), -1.0, 1.0);
  return std::acos(cosine);
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

}  // namespace

class RelocalizationNode final : public rclcpp::Node
{
public:
  RelocalizationNode()
  : Node("relocalization_node"), database_(make_database_config())
  {
    declare_parameter("keyframe_cloud_topic", "/lio/local_map");
    declare_parameter("query_cloud_topic", "/lidar/points_deskewed");
    declare_parameter("fused_pose_topic", "/fusion/unified/map_pose");
    declare_parameter("source_lio_pose_topic", "/lio/odom");
    declare_parameter("diagnostics_topic", "/lio/diagnostics");
    declare_parameter("lidar_score_topic", "/reliability/lidar_score");
    declare_parameter("scheduler_topic", "/reliability/scheduler_state");
    declare_parameter(
      "keyframe_allowed_scheduler_states",
      std::vector<std::string>{"NORMAL", "DEGRADED", "RECOVERED"});
    declare_parameter("request_topic", "/relocalization/request");
    declare_parameter("result_topic", "/relocalization/result");
    declare_parameter("ready_topic", "/relocalization/ready");
    declare_parameter("keyframe_attempt_period_s", 0.5);
    declare_parameter("query_attempt_period_s", 0.5);
    declare_parameter("maximum_candidates", 3);
    declare_parameter("exclude_recent_keyframes", 3);
    declare_parameter("maximum_descriptor_distance", 0.35);
    declare_parameter("registration_method", "icp");
    declare_parameter("maximum_registration_fitness", 0.25);
    declare_parameter("maximum_alignment_translation_m", 30.0);
    declare_parameter("maximum_alignment_rotation_rad", 1.6);
    declare_parameter("maximum_cloud_points", 1800);
    declare_parameter("voxel_size_m", 0.25);
    declare_parameter("minimum_registration_points", 30);
    declare_parameter("maximum_search_attempts", 10);
    declare_parameter("search_timeout_s", 6.0);
    declare_parameter("minimum_database_keyframes", 4);
    declare_parameter("maximum_roll_pitch_correction_rad", 0.35);
    declare_parameter("minimum_registration_overlap_ratio", 0.35);
    declare_parameter("minimum_registration_correspondences", 120);
    declare_parameter("minimum_registration_reciprocal_ratio", 0.20);
    declare_parameter("minimum_registration_reciprocal_correspondences", 80);
    declare_parameter("maximum_registration_inlier_rmse_m", 0.35);
    declare_parameter("maximum_registration_cycle_translation_m", 0.30);
    declare_parameter("maximum_registration_cycle_rotation_rad", 0.20);
    declare_parameter("minimum_verified_score_margin", 0.05);
    declare_parameter("equivalent_candidate_translation_m", 0.50);
    declare_parameter("equivalent_candidate_rotation_rad", 0.25);
    declare_parameter("success_consistency_required_queries", 3);
    declare_parameter("success_consistency_translation_m", 0.15);
    declare_parameter("success_consistency_rotation_rad", 0.05);
    declare_parameter("keyframe_pose_tolerance_s", 0.12);
    declare_parameter("query_pose_tolerance_s", 0.08);
    declare_parameter("keyframe_sync_timeout_s", 0.5);
    declare_parameter("maximum_quality_age_s", 2.0);
    declare_parameter("pose_history_size", 200);
    declare_parameter("query_cloud_history_size", 32);
    declare_parameter("keyframe_consistency_diagnostics_enabled", true);
    declare_parameter("expected_keyframe_frame_id", "camera_init");
    declare_parameter("expected_query_frame_id", "body");

    keyframe_attempt_period_s_ = get_parameter("keyframe_attempt_period_s").as_double();
    query_attempt_period_s_ = get_parameter("query_attempt_period_s").as_double();
    maximum_candidates_ = std::max(
      1, static_cast<int>(get_parameter("maximum_candidates").as_int()));
    exclude_recent_keyframes_ = std::max(
      0, static_cast<int>(get_parameter("exclude_recent_keyframes").as_int()));
    maximum_descriptor_distance_ = get_parameter("maximum_descriptor_distance").as_double();
    registration_method_ = get_parameter("registration_method").as_string();
    maximum_registration_fitness_ = get_parameter("maximum_registration_fitness").as_double();
    maximum_alignment_translation_m_ = get_parameter("maximum_alignment_translation_m").as_double();
    maximum_alignment_rotation_rad_ = get_parameter("maximum_alignment_rotation_rad").as_double();
    maximum_cloud_points_ = std::max(
      30, static_cast<int>(get_parameter("maximum_cloud_points").as_int()));
    voxel_size_m_ = get_parameter("voxel_size_m").as_double();
    minimum_registration_points_ = std::max(
      10, static_cast<int>(get_parameter("minimum_registration_points").as_int()));
    maximum_search_attempts_ = std::max(
      1, static_cast<int>(get_parameter("maximum_search_attempts").as_int()));
    search_timeout_s_ = get_parameter("search_timeout_s").as_double();
    minimum_database_keyframes_ = std::max(
      exclude_recent_keyframes_ + 2,
      static_cast<int>(get_parameter("minimum_database_keyframes").as_int()));
    maximum_roll_pitch_correction_rad_ =
      get_parameter("maximum_roll_pitch_correction_rad").as_double();
    minimum_registration_overlap_ratio_ =
      get_parameter("minimum_registration_overlap_ratio").as_double();
    minimum_registration_correspondences_ = std::max(
      10, static_cast<int>(
        get_parameter("minimum_registration_correspondences").as_int()));
    minimum_registration_reciprocal_ratio_ =
      get_parameter("minimum_registration_reciprocal_ratio").as_double();
    minimum_registration_reciprocal_correspondences_ = std::max(
      10, static_cast<int>(
        get_parameter("minimum_registration_reciprocal_correspondences").as_int()));
    maximum_registration_inlier_rmse_m_ =
      get_parameter("maximum_registration_inlier_rmse_m").as_double();
    maximum_registration_cycle_translation_m_ =
      get_parameter("maximum_registration_cycle_translation_m").as_double();
    maximum_registration_cycle_rotation_rad_ =
      get_parameter("maximum_registration_cycle_rotation_rad").as_double();
    minimum_verified_score_margin_ =
      get_parameter("minimum_verified_score_margin").as_double();
    equivalent_candidate_translation_m_ =
      get_parameter("equivalent_candidate_translation_m").as_double();
    equivalent_candidate_rotation_rad_ =
      get_parameter("equivalent_candidate_rotation_rad").as_double();
    success_consistency_required_queries_ = std::max(
      1, static_cast<int>(
        get_parameter("success_consistency_required_queries").as_int()));
    success_consistency_translation_m_ =
      get_parameter("success_consistency_translation_m").as_double();
    success_consistency_rotation_rad_ =
      get_parameter("success_consistency_rotation_rad").as_double();
    keyframe_pose_tolerance_s_ =
      get_parameter("keyframe_pose_tolerance_s").as_double();
    query_pose_tolerance_s_ =
      get_parameter("query_pose_tolerance_s").as_double();
    keyframe_sync_timeout_s_ =
      get_parameter("keyframe_sync_timeout_s").as_double();
    maximum_quality_age_s_ =
      get_parameter("maximum_quality_age_s").as_double();
    pose_history_size_ = std::max(
      10, static_cast<int>(get_parameter("pose_history_size").as_int()));
    query_cloud_history_size_ = std::max(
      4, static_cast<int>(get_parameter("query_cloud_history_size").as_int()));
    keyframe_consistency_diagnostics_enabled_ =
      get_parameter("keyframe_consistency_diagnostics_enabled").as_bool();
    expected_keyframe_frame_id_ =
      get_parameter("expected_keyframe_frame_id").as_string();
    expected_query_frame_id_ =
      get_parameter("expected_query_frame_id").as_string();
    if (registration_method_ != "icp" && registration_method_ != "gicp" &&
      registration_method_ != "point_to_plane")
    {
      throw std::invalid_argument(
              "registration_method must be 'icp', 'gicp', or 'point_to_plane'");
    }
    keyframe_allowed_scheduler_states_ =
      get_parameter("keyframe_allowed_scheduler_states").as_string_array();
    if (keyframe_attempt_period_s_ <= 0.0 || query_attempt_period_s_ <= 0.0 ||
      maximum_descriptor_distance_ <= 0.0 ||
      maximum_registration_fitness_ <= 0.0 || maximum_alignment_translation_m_ <= 0.0 ||
      maximum_alignment_rotation_rad_ <= 0.0 || voxel_size_m_ <= 0.0 ||
      search_timeout_s_ <= 0.0 ||
      maximum_roll_pitch_correction_rad_ <= 0.0 ||
      minimum_registration_overlap_ratio_ <= 0.0 ||
      minimum_registration_overlap_ratio_ > 1.0 ||
      minimum_registration_reciprocal_ratio_ <= 0.0 ||
      minimum_registration_reciprocal_ratio_ > 1.0 ||
      maximum_registration_inlier_rmse_m_ <= 0.0 ||
      maximum_registration_cycle_translation_m_ <= 0.0 ||
      maximum_registration_cycle_rotation_rad_ <= 0.0 ||
      minimum_verified_score_margin_ < 0.0 ||
      equivalent_candidate_translation_m_ <= 0.0 ||
      equivalent_candidate_rotation_rad_ <= 0.0 ||
      success_consistency_translation_m_ <= 0.0 ||
      success_consistency_rotation_rad_ <= 0.0 ||
      keyframe_pose_tolerance_s_ <= 0.0 || query_pose_tolerance_s_ <= 0.0 ||
      keyframe_sync_timeout_s_ <= 0.0 ||
      maximum_quality_age_s_ <= 0.0)
    {
      throw std::invalid_argument("invalid relocalization node limits");
    }
    success_consistency_gate_ = MultiFrameConsistencyGate(
      MultiFrameConsistencyConfig{
        static_cast<std::size_t>(success_consistency_required_queries_),
        success_consistency_translation_m_, success_consistency_rotation_rad_});

    result_pub_ = create_publisher<ResultMessage>(
      get_parameter("result_topic").as_string(), rclcpp::QoS(10).reliable());
    auto readiness_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    readiness_qos.reliable().transient_local();
    readiness_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("ready_topic").as_string(), readiness_qos);
    request_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("request_topic").as_string(), rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        if (!message->data) {
          request_asserted_ = false;
          request_active_ = false;
          pending_query_cloud_.reset();
          success_consistency_gate_.reset();
          return;
        }
        // /relocalization/request is a level, not a transaction counter. A
        // reliable writer can redeliver the asserted level, and multiple
        // supervisors may legitimately assert it at the same time. Start one
        // transaction per false -> true edge and require an explicit release
        // before another search.
        if (request_asserted_) {
          return;
        }
        request_asserted_ = true;
        request_active_ = true;
        pending_query_cloud_.reset();
        success_consistency_gate_.reset();
        const auto request_time = now();
        const auto request_ns = static_cast<uint64_t>(std::max<int64_t>(
          1, request_time.nanoseconds()));
        active_transaction_id_ = std::max(
          last_transaction_id_ + 1U, request_ns);
        last_transaction_id_ = active_transaction_id_;
        search_attempt_count_ = 0;
        request_started_s_ = request_time.seconds();
        RCLCPP_WARN(
          get_logger(),
          "relocalization transaction=%llu requested with %zu static keyframes",
          static_cast<unsigned long long>(active_transaction_id_),
          database_.keyframes().size());
        if (database_.keyframes().size() <
          static_cast<std::size_t>(minimum_database_keyframes_))
        {
          publish_status(ResultMessage::FAILED, false, false, 0U, 0.0, 0.0,
            "database_not_ready");
          request_active_ = false;
          return;
        }
        publish_status(ResultMessage::SEARCHING, true, false, 0U, 0.0, 0.0,
          "searching_static_keyframes");
      });
    fused_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("fused_pose_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        latest_fused_pose_ = message;
        fused_pose_history_.push_back(message);
        while (fused_pose_history_.size() > static_cast<std::size_t>(pose_history_size_)) {
          fused_pose_history_.pop_front();
        }
        process_pending_keyframe_cloud();
      });
    lio_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("source_lio_pose_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        latest_lio_pose_ = message;
        lio_pose_history_.push_back(message);
        while (lio_pose_history_.size() > static_cast<std::size_t>(pose_history_size_)) {
          lio_pose_history_.pop_front();
        }
        process_pending_keyframe_cloud();
        process_pending_query_cloud();
      });
    diagnostics_sub_ = create_subscription<uf_interfaces::msg::LioDiagnostics>(
      get_parameter("diagnostics_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const uf_interfaces::msg::LioDiagnostics::SharedPtr message) {
        latest_diagnostics_ = message;
      });
    lidar_score_sub_ = create_subscription<uf_interfaces::msg::ReliabilityScore>(
      get_parameter("lidar_score_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const uf_interfaces::msg::ReliabilityScore::SharedPtr message) {
        latest_lidar_score_ = message;
      });
    scheduler_sub_ = create_subscription<uf_interfaces::msg::SchedulerState>(
      get_parameter("scheduler_topic").as_string(), rclcpp::QoS(10),
      [this](const uf_interfaces::msg::SchedulerState::SharedPtr message) {
        latest_scheduler_state_ = message;
        scheduler_health_ = message->health_state;
        scheduler_lidar_enabled_ = false;
        for (std::size_t index = 0; index < message->modality_names.size(); ++index) {
          if (message->modality_names[index] == "lidar" &&
            index < message->factor_enabled.size())
          {
            scheduler_lidar_enabled_ = message->factor_enabled[index];
          }
        }
      });
    keyframe_cloud_sub_ = create_subscription<CloudMessage>(
      get_parameter("keyframe_cloud_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&RelocalizationNode::keyframe_cloud_callback, this, std::placeholders::_1));
    query_cloud_sub_ = create_subscription<CloudMessage>(
      get_parameter("query_cloud_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&RelocalizationNode::query_cloud_callback, this, std::placeholders::_1));
    search_timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RelocalizationNode::search_timeout_callback, this));
    publish_database_readiness(true);
    RCLCPP_INFO(get_logger(),
      "online relocalization active: map-frame static submaps + body-frame query scan + "
      "ESF retrieval + reciprocal %s verification",
      registration_method_.c_str());
  }

private:
  RegistrationResult align_registration(
    const Cloud::ConstPtr & source,
    const Cloud::ConstPtr & target,
    const Eigen::Matrix4f & initial,
    const RegistrationConfig & config) const
  {
    if (registration_method_ == "gicp") {
      return align_gicp(source, target, initial, config);
    }
    if (registration_method_ == "point_to_plane") {
      return align_point_to_plane(source, target, initial, config);
    }
    return align_icp(source, target, initial, config);
  }

  static KeyframeDatabaseConfig make_database_config()
  {
    KeyframeDatabaseConfig config;
    config.minimum_feature_repeatability = 0.70;
    config.maximum_dynamic_ratio = 0.15;
    config.maximum_lidar_degradation = 0.75;
    config.minimum_translation_spacing_m = 1.0;
    config.minimum_rotation_spacing_rad = 0.26;
    config.maximum_keyframes = 500;
    return config;
  }

  void publish_database_readiness(const bool force = false)
  {
    const bool ready = database_.keyframes().size() >=
      static_cast<std::size_t>(minimum_database_keyframes_);
    if (!force && readiness_published_ && ready == database_ready_) {
      return;
    }
    database_ready_ = ready;
    readiness_published_ = true;
    std_msgs::msg::Bool message;
    message.data = ready;
    readiness_pub_->publish(message);
    RCLCPP_INFO(
      get_logger(), "relocalization database ready=%d keyframes=%zu required=%d",
      ready, database_.keyframes().size(), minimum_database_keyframes_);
  }

  static bool stamp_close(const builtin_interfaces::msg::Time & left,
    const builtin_interfaces::msg::Time & right, double maximum_age_s)
  {
    const double left_s = static_cast<double>(left.sec) + left.nanosec * 1.0e-9;
    const double right_s = static_cast<double>(right.sec) + right.nanosec * 1.0e-9;
    return std::isfinite(left_s) && std::isfinite(right_s) &&
      std::abs(left_s - right_s) <= maximum_age_s;
  }

  static nav_msgs::msg::Odometry::SharedPtr nearest_pose(
    const std::deque<nav_msgs::msg::Odometry::SharedPtr> & history,
    const builtin_interfaces::msg::Time & stamp,
    const double maximum_delta_s)
  {
    nav_msgs::msg::Odometry::SharedPtr best;
    double best_delta_s = std::numeric_limits<double>::infinity();
    const double requested_s = stamp_seconds(stamp);
    for (const auto & candidate : history) {
      const double delta_s = std::abs(
        stamp_seconds(candidate->header.stamp) - requested_s);
      if (delta_s < best_delta_s) {
        best_delta_s = delta_s;
        best = candidate;
      }
    }
    return best_delta_s <= maximum_delta_s ? best : nullptr;
  }

  static CloudMessage::SharedPtr nearest_cloud(
    const std::deque<CloudMessage::SharedPtr> & history,
    const builtin_interfaces::msg::Time & stamp,
    const double maximum_delta_s)
  {
    CloudMessage::SharedPtr best;
    double best_delta_s = std::numeric_limits<double>::infinity();
    const double requested_s = stamp_seconds(stamp);
    for (const auto & candidate : history) {
      const double delta_s = std::abs(
        stamp_seconds(candidate->header.stamp) - requested_s);
      if (delta_s < best_delta_s) {
        best_delta_s = delta_s;
        best = candidate;
      }
    }
    return best_delta_s <= maximum_delta_s ? best : nullptr;
  }

  static std::optional<double> latest_pose_stamp(
    const std::deque<nav_msgs::msg::Odometry::SharedPtr> & history)
  {
    std::optional<double> latest;
    for (const auto & candidate : history) {
      const double stamp_s = stamp_seconds(candidate->header.stamp);
      if (std::isfinite(stamp_s) && (!latest || stamp_s > *latest)) {
        latest = stamp_s;
      }
    }
    return latest;
  }

  static std::optional<double> latest_cloud_stamp(
    const std::deque<CloudMessage::SharedPtr> & history)
  {
    std::optional<double> latest;
    for (const auto & candidate : history) {
      const double stamp_s = stamp_seconds(candidate->header.stamp);
      if (std::isfinite(stamp_s) && (!latest || stamp_s > *latest)) {
        latest = stamp_s;
      }
    }
    return latest;
  }

  bool quality_is_fresh(const builtin_interfaces::msg::Time & stamp) const
  {
    return latest_diagnostics_ && latest_lidar_score_ && latest_scheduler_state_ &&
      stamp_close(stamp, latest_diagnostics_->header.stamp, maximum_quality_age_s_) &&
      stamp_close(stamp, latest_lidar_score_->header.stamp, maximum_quality_age_s_) &&
      stamp_close(stamp, latest_scheduler_state_->header.stamp, maximum_quality_age_s_);
  }

  Cloud::Ptr convert_cloud(const CloudMessage & message) const
  {
    auto raw = std::make_shared<Cloud>();
    pcl::fromROSMsg(message, *raw);
    auto finite = std::make_shared<Cloud>();
    finite->reserve(raw->size());
    for (const auto & point : *raw) {
      if (pcl::isFinite(point)) {
        finite->push_back(point);
      }
    }
    finite->width = static_cast<std::uint32_t>(finite->size());
    finite->height = 1U;
    finite->is_dense = true;
    if (finite->empty()) {
      return finite;
    }
    auto filtered = std::make_shared<Cloud>();
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(finite);
    voxel.setLeafSize(
      static_cast<float>(voxel_size_m_), static_cast<float>(voxel_size_m_),
      static_cast<float>(voxel_size_m_));
    voxel.filter(*filtered);
    auto clean_filtered = std::make_shared<Cloud>();
    clean_filtered->reserve(filtered->size());
    for (const auto & point : *filtered) {
      if (pcl::isFinite(point)) {
        clean_filtered->push_back(point);
      }
    }
    clean_filtered->width = static_cast<std::uint32_t>(clean_filtered->size());
    clean_filtered->height = 1U;
    clean_filtered->is_dense = true;
    if (clean_filtered->size() > static_cast<std::size_t>(maximum_cloud_points_)) {
      auto limited = std::make_shared<Cloud>();
      limited->reserve(static_cast<std::size_t>(maximum_cloud_points_));
      const double stride = static_cast<double>(clean_filtered->size()) /
        static_cast<double>(maximum_cloud_points_);
      for (int index = 0; index < maximum_cloud_points_; ++index) {
        limited->push_back((*clean_filtered)[static_cast<std::size_t>(index * stride)]);
      }
      limited->width = static_cast<std::uint32_t>(limited->size());
      limited->height = 1U;
      limited->is_dense = true;
      return limited;
    }
    return clean_filtered;
  }

  KeyframeQuality current_quality() const
  {
    KeyframeQuality quality;
    if (latest_diagnostics_) {
      quality.map_quality = latest_diagnostics_->map_quality;
      quality.feature_repeatability = latest_diagnostics_->feature_repeatability;
      quality.dynamic_ratio = latest_diagnostics_->dynamic_ratio;
    }
    if (latest_lidar_score_) {
      quality.lidar_degradation = latest_lidar_score_->degradation_score;
    } else {
      quality.lidar_degradation = 1.0;
    }
    quality.scheduler_lidar_enabled = scheduler_lidar_enabled_ &&
      std::find(
      keyframe_allowed_scheduler_states_.begin(),
      keyframe_allowed_scheduler_states_.end(), scheduler_health_) !=
      keyframe_allowed_scheduler_states_.end();
    return quality;
  }

  void publish_status(uint8_t state, bool request_active, bool accepted,
    uint32_t candidate_id, double descriptor_distance, double fitness,
    const std::string & reason, const Eigen::Isometry3d * pose = nullptr,
    const Eigen::Isometry3d * source_pose = nullptr,
    const Eigen::Isometry3d * map_from_lio = nullptr,
    const builtin_interfaces::msg::Time * result_stamp = nullptr)
  {
    ResultMessage message;
    if (result_stamp != nullptr) {
      message.header.stamp = *result_stamp;
    } else {
      message.header.stamp = now();
    }
    message.header.frame_id = latest_fused_pose_ ? latest_fused_pose_->header.frame_id : "camera_init";
    message.state = state;
    message.state_name = state == ResultMessage::SUCCESS ? "CANDIDATE_ACCEPTED" :
      (state == ResultMessage::FAILED ? "FAILED" :
      (state == ResultMessage::SEARCHING ? "SEARCHING" : "IDLE"));
    message.request_active = request_active;
    message.accepted = accepted;
    message.transaction_id = active_transaction_id_;
    message.candidate_id = candidate_id;
    message.descriptor_distance = static_cast<float>(descriptor_distance);
    message.registration_fitness = static_cast<float>(fitness);
    // The relocalizer proposes a pose; only the unified backend owns and
    // increments the estimator reset counter after a transactional commit.
    message.reset_counter = 0U;
    message.reason = reason;
    if (pose != nullptr) {
      message.pose.pose = isometry_to_pose(*pose);
      message.pose.covariance[0] = 0.50 * 0.50;
      message.pose.covariance[7] = 0.50 * 0.50;
      message.pose.covariance[14] = 0.70 * 0.70;
      message.pose.covariance[21] = 0.20 * 0.20;
      message.pose.covariance[28] = 0.20 * 0.20;
      message.pose.covariance[35] = 0.35 * 0.35;
    }
    if (source_pose != nullptr) {
      message.source_lio_pose = isometry_to_pose(*source_pose);
    }
    if (map_from_lio != nullptr) {
      message.map_from_lio = isometry_to_pose(*map_from_lio);
    }
    result_pub_->publish(message);
  }

  void keyframe_cloud_callback(const CloudMessage::SharedPtr message)
  {
    if (request_active_ || !message || message->width * message->height == 0U) {
      return;
    }
    if (!expected_keyframe_frame_id_.empty() &&
      message->header.frame_id != expected_keyframe_frame_id_)
    {
      ++keyframe_frame_rejections_;
      if (keyframe_frame_rejections_ % 20U == 1U) {
        RCLCPP_ERROR(
          get_logger(), "rejecting keyframe cloud frame '%s'; expected '%s'",
          message->header.frame_id.c_str(), expected_keyframe_frame_id_.c_str());
      }
      return;
    }
    if (pending_keyframe_cloud_ &&
      stamp_nanoseconds(pending_keyframe_cloud_->header.stamp) !=
      stamp_nanoseconds(message->header.stamp))
    {
      process_pending_keyframe_cloud();
      if (pending_keyframe_cloud_) {
        ++keyframe_sync_waiting_cloud_skips_;
        if (keyframe_sync_waiting_cloud_skips_ % 20U == 1U) {
          RCLCPP_WARN(
            get_logger(),
            "retaining LiDAR cloud stamp=%.3f while synchronized evidence is pending; "
            "ignored %zu newer map clouds",
            stamp_seconds(pending_keyframe_cloud_->header.stamp),
            keyframe_sync_waiting_cloud_skips_);
        }
        return;
      }
    }
    pending_keyframe_cloud_ = message;
    pending_keyframe_started_s_ = now().seconds();
    process_pending_keyframe_cloud();
  }

  void process_pending_keyframe_cloud()
  {
    if (request_active_ || !pending_keyframe_cloud_) {
      return;
    }
    const auto message = pending_keyframe_cloud_;
    const double stamp_s = stamp_seconds(message->header.stamp);
    if (!std::isfinite(stamp_s)) {
      pending_keyframe_cloud_.reset();
      pending_keyframe_started_s_ = std::numeric_limits<double>::quiet_NaN();
      return;
    }
    double now_s = now().seconds();
    if (!std::isfinite(now_s)) {
      return;
    }
    if (!std::isfinite(pending_keyframe_started_s_) ||
      now_s < pending_keyframe_started_s_)
    {
      pending_keyframe_started_s_ = now_s;
    }
    const double waiting_age_s = std::max(0.0, now_s - pending_keyframe_started_s_);
    const auto fused_pose_message = nearest_pose(
      fused_pose_history_, message->header.stamp, keyframe_pose_tolerance_s_);
    const auto lio_pose_message = nearest_pose(
      lio_pose_history_, message->header.stamp, keyframe_pose_tolerance_s_);
    const auto body_message = nearest_cloud(
      query_cloud_history_, message->header.stamp, query_pose_tolerance_s_);
    const auto synchronization = decide_keyframe_synchronization(
      stamp_s,
      TimestampEvidence{
        static_cast<bool>(fused_pose_message), latest_pose_stamp(fused_pose_history_),
        keyframe_pose_tolerance_s_},
      TimestampEvidence{
        static_cast<bool>(lio_pose_message), latest_pose_stamp(lio_pose_history_),
        keyframe_pose_tolerance_s_},
      TimestampEvidence{
        static_cast<bool>(body_message), latest_cloud_stamp(query_cloud_history_),
        query_pose_tolerance_s_},
      waiting_age_s, keyframe_sync_timeout_s_);
    if (synchronization.state == KeyframeSynchronizationState::WAITING) {
      return;
    }
    pending_keyframe_cloud_.reset();
    pending_keyframe_started_s_ = std::numeric_limits<double>::quiet_NaN();
    if (synchronization.state == KeyframeSynchronizationState::EXPIRED) {
      ++keyframe_pose_sync_rejections_;
      if (keyframe_pose_sync_rejections_ % 20U == 1U) {
        RCLCPP_WARN(
          get_logger(),
          "keyframe synchronization expired %zu clouds: reason=%s stamp=%.3f",
          keyframe_pose_sync_rejections_, synchronization.reason.c_str(), stamp_s);
      }
      return;
    }
    if (!fused_pose_message || !lio_pose_message ||
      !body_message ||
      !finite_pose(fused_pose_message->pose.pose) ||
      !finite_pose(lio_pose_message->pose.pose) ||
      lio_pose_message->header.frame_id != message->header.frame_id)
    {
      ++keyframe_pose_sync_rejections_;
      if (keyframe_pose_sync_rejections_ % 20U == 1U) {
        RCLCPP_WARN(
          get_logger(),
          "keyframe synchronized evidence rejected %zu clouds: invalid pose or frame",
          keyframe_pose_sync_rejections_);
      }
      return;
    }
    if (stamp_s - last_keyframe_attempt_s_ < keyframe_attempt_period_s_)
    {
      return;
    }
    last_keyframe_attempt_s_ = stamp_s;
    if (!quality_is_fresh(message->header.stamp)) {
      ++keyframe_quality_stale_rejections_;
      return;
    }
    const auto quality = current_quality();
    const auto quality_admission = database_.quality_admission(quality);
    if (!quality_admission.accepted) {
      ++keyframe_quality_rejections_;
      if (keyframe_quality_rejections_ % 20U == 0U) {
        RCLCPP_WARN(
          get_logger(),
          "keyframe gate rejected %zu attempts: reason=%s scheduler=%s enabled=%d "
          "map=%.3f repeat=%.3f dynamic=%.3f lidar_D=%.3f",
          keyframe_quality_rejections_, quality_admission.reason.c_str(),
          scheduler_health_.c_str(),
          quality.scheduler_lidar_enabled, quality.map_quality,
          quality.feature_repeatability, quality.dynamic_ratio,
          quality.lidar_degradation);
      }
      return;
    }
    const auto fused_pose = pose_to_isometry(fused_pose_message->pose.pose);
    const auto lio_pose = pose_to_isometry(lio_pose_message->pose.pose);
    if (has_last_keyframe_pose_) {
      const double translation =
        (fused_pose.translation() - last_keyframe_pose_.translation()).norm();
      const double rotation = rotation_angle(
        last_keyframe_pose_.rotation().transpose() * fused_pose.rotation());
      if (translation < 1.0 && rotation < 0.26) {
        return;
      }
    }
    const auto lio_map_cloud = convert_cloud(*message);
    if (!lio_map_cloud ||
      lio_map_cloud->size() < static_cast<std::size_t>(minimum_registration_points_))
    {
      return;
    }
    const Eigen::Isometry3d map_from_lio = fused_pose * lio_pose.inverse();
    auto unified_map_cloud = std::make_shared<Cloud>();
    pcl::transformPointCloud(
      *lio_map_cloud, *unified_map_cloud,
      map_from_lio.matrix().cast<float>());
    const auto body_cloud = convert_cloud(*body_message);
    if (!body_cloud ||
      body_cloud->size() < static_cast<std::size_t>(minimum_registration_points_))
    {
      ++keyframe_pose_sync_rejections_;
      return;
    }
    if (keyframe_consistency_diagnostics_enabled_) {
      try {
        RegistrationConfig consistency_config;
        consistency_config.maximum_correspondence_distance_m = 1.5;
        consistency_config.maximum_iterations = 30;
        const auto consistency = align_icp(
          body_cloud, unified_map_cloud,
          fused_pose.matrix().cast<float>(), consistency_config);
        RCLCPP_INFO(
          get_logger(),
          "keyframe frame-consistency: converged=%d fitness=%.6f "
          "overlap=%.3f rmse=%.3f correspondences=%zu source=%zu target=%zu",
          consistency.converged, consistency.fitness,
          consistency.overlap_ratio, consistency.inlier_rmse,
          consistency.correspondence_points, consistency.source_points,
          consistency.target_points);
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          get_logger(), "keyframe frame-consistency failed: %s", error.what());
      }
    }
    try {
      // Retrieval descriptors must have the same sampling distribution as an
      // online query. Store ESF from the synchronized single body-frame scan,
      // while retaining the denser static submap for geometric verification.
      const auto descriptor = compute_esf_descriptor(body_cloud);
      const auto admission = database_.try_insert(
        stamp_s, fused_pose, unified_map_cloud, descriptor, quality);
      if (admission.accepted) {
        last_keyframe_pose_ = fused_pose;
        has_last_keyframe_pose_ = true;
        const double yaw_deg = std::atan2(
          fused_pose.rotation()(1, 0), fused_pose.rotation()(0, 0)) *
          180.0 / 3.14159265358979323846;
        RCLCPP_INFO(
          get_logger(),
          "static relocalization keyframe inserted: id=%zu total=%zu "
          "stamp=%.3f pose=(%.3f,%.3f,%.3f) yaw_deg=%.1f "
          "descriptor_points=%zu submap_points=%zu",
          admission.keyframe_id, database_.keyframes().size(), stamp_s,
          fused_pose.translation().x(), fused_pose.translation().y(),
          fused_pose.translation().z(), yaw_deg, body_cloud->size(),
          unified_map_cloud->size());
        publish_database_readiness();
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "keyframe descriptor rejected: %s", error.what());
    }
  }

  void search_failed_attempt(const std::string & reason)
  {
    // A query without a verified geometric hypothesis breaks the required
    // consecutive-query sequence. Partial but valid hypotheses bypass this
    // function and therefore do not consume the failure budget.
    success_consistency_gate_.reset();
    ++search_attempt_count_;
    if (search_attempt_count_ >= maximum_search_attempts_) {
      publish_status(ResultMessage::FAILED, false, false, 0U, 0.0, 0.0,
        "search_attempt_limit_reached:" + reason);
      RCLCPP_ERROR(
        get_logger(), "relocalization failed after %d attempts: %s",
        search_attempt_count_, reason.c_str());
      request_active_ = false;
      return;
    }
    publish_status(ResultMessage::SEARCHING, true, false, 0U, 0.0, 0.0, reason);
  }

  void query_cloud_callback(const CloudMessage::SharedPtr message)
  {
    if (!message || message->width * message->height == 0U) {
      return;
    }
    if (!expected_query_frame_id_.empty() &&
      message->header.frame_id != expected_query_frame_id_)
    {
      if (request_active_) {
        search_failed_attempt(
          "unexpected_query_frame:" + message->header.frame_id);
      }
      return;
    }
    query_cloud_history_.push_back(message);
    while (query_cloud_history_.size() >
      static_cast<std::size_t>(query_cloud_history_size_))
    {
      query_cloud_history_.pop_front();
    }
    // Body cloud can be the final member of the map pose/LIO pose/body cloud
    // tuple. Wake keyframe synchronization immediately on its arrival.
    process_pending_keyframe_cloud();
    if (!request_active_) {
      return;
    }
    pending_query_cloud_ = message;
    process_pending_query_cloud();
  }

  void process_pending_query_cloud()
  {
    if (!request_active_ || !pending_query_cloud_ || !latest_lio_pose_) {
      return;
    }
    const auto message = pending_query_cloud_;
    const double stamp_s = stamp_seconds(message->header.stamp);
    if (!std::isfinite(stamp_s)) {
      pending_query_cloud_.reset();
      search_failed_attempt("invalid_query_stamp");
      return;
    }
    if (stamp_seconds(latest_lio_pose_->header.stamp) < stamp_s) {
      return;
    }
    pending_query_cloud_.reset();
    if (stamp_s - last_query_attempt_s_ < query_attempt_period_s_)
    {
      return;
    }
    last_query_attempt_s_ = stamp_s;
    const auto source_pose_message = nearest_pose(
      lio_pose_history_, message->header.stamp, query_pose_tolerance_s_);
    if (!source_pose_message ||
      !finite_pose(source_pose_message->pose.pose) ||
      source_pose_message->child_frame_id != message->header.frame_id)
    {
      search_failed_attempt("missing_time_aligned_source_lio_pose");
      return;
    }
    const auto cloud = convert_cloud(*message);
    if (!cloud || cloud->size() < static_cast<std::size_t>(minimum_registration_points_)) {
      search_failed_attempt("insufficient_body_query_cloud");
      return;
    }

    std::vector<float> descriptor;
    try {
      descriptor = compute_esf_descriptor(cloud);
    } catch (const std::exception & error) {
      search_failed_attempt(std::string("descriptor_failed:") + error.what());
      return;
    }
    const auto candidates = database_.query(
      descriptor, static_cast<std::size_t>(maximum_candidates_), exclude_recent_keyframes_);
    if (candidates.empty()) {
      search_failed_attempt("no_descriptor_candidate");
      return;
    }
    std::ostringstream retrieval_summary;
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      if (index > 0U) {
        retrieval_summary << ",";
      }
      retrieval_summary << candidates[index].keyframe_id << ":"
                        << candidates[index].descriptor_distance;
    }
    RCLCPP_INFO(
      get_logger(), "relocalization retrieval candidates: %s",
      retrieval_summary.str().c_str());
    const auto source_pose = pose_to_isometry(source_pose_message->pose.pose);
    struct VerifiedCandidate
    {
      std::size_t keyframe_id{0U};
      double descriptor_distance{0.0};
      RegistrationResult registration;
      Eigen::Isometry3d recovered_pose{Eigen::Isometry3d::Identity()};
      Eigen::Isometry3d map_from_lio{Eigen::Isometry3d::Identity()};
      double score{std::numeric_limits<double>::infinity()};
    };
    std::vector<VerifiedCandidate> verified_candidates;
    std::size_t descriptor_gate_rejections = 0U;
    std::size_t missing_keyframe_rejections = 0U;
    std::size_t forward_registration_rejections = 0U;
    std::size_t reciprocal_support_rejections = 0U;
    std::size_t reverse_registration_rejections = 0U;
    std::size_t cycle_rejections = 0U;
    std::size_t alignment_rejections = 0U;
    std::size_t registration_exceptions = 0U;
    struct CandidateDiagnostic
    {
      std::size_t keyframe_id{0U};
      double descriptor_distance{0.0};
      double yaw_offset{0.0};
      double forward_fitness{std::numeric_limits<double>::infinity()};
      double forward_overlap{0.0};
      double forward_rmse{std::numeric_limits<double>::infinity()};
      double forward_euclidean_rmse{std::numeric_limits<double>::infinity()};
      std::size_t forward_correspondences{0U};
      std::size_t forward_source_points{0U};
      std::size_t forward_target_points{0U};
      std::size_t reciprocal_correspondences{0U};
      double reciprocal_ratio{0.0};
      double plane_error_p90_m{std::numeric_limits<double>::infinity()};
      bool forward_converged{false};
      bool passed_forward_gate{false};
      bool passed_reciprocal_gate{false};
      int forward_effective_rank{0};
      double forward_condition_number{std::numeric_limits<double>::infinity()};
      double reverse_fitness{std::numeric_limits<double>::infinity()};
      double reverse_overlap{0.0};
      double reverse_rmse{std::numeric_limits<double>::infinity()};
      double reverse_euclidean_rmse{std::numeric_limits<double>::infinity()};
      std::size_t reverse_correspondences{0U};
      std::size_t reverse_source_points{0U};
      std::size_t reverse_target_points{0U};
      bool reverse_converged{false};
      bool passed_reverse_gate{false};
      int reverse_effective_rank{0};
      double reverse_condition_number{std::numeric_limits<double>::infinity()};
      double cycle_translation_m{std::numeric_limits<double>::infinity()};
      double cycle_rotation_rad{std::numeric_limits<double>::infinity()};
      double alignment_translation_m{std::numeric_limits<double>::infinity()};
      double alignment_rotation_rad{std::numeric_limits<double>::infinity()};
      double alignment_tilt_rad{std::numeric_limits<double>::infinity()};
      int stage_depth{0};
      std::string rejection_stage{"forward"};
    };
    std::optional<CandidateDiagnostic> best_forward_attempt;
    const double source_yaw = std::atan2(
      source_pose.rotation()(1, 0), source_pose.rotation()(0, 0));
    const Eigen::Matrix3d source_tilt =
      Eigen::AngleAxisd(-source_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      source_pose.rotation();
    constexpr double quarter_turn = 0.5 * 3.14159265358979323846;
    const std::array<double, 4> yaw_offsets = {
      0.0, quarter_turn, -quarter_turn, 2.0 * quarter_turn};
    for (const auto & candidate : candidates) {
      if (candidate.descriptor_distance > maximum_descriptor_distance_) {
        ++descriptor_gate_rejections;
        continue;
      }
      const auto * keyframe = database_.find(candidate.keyframe_id);
      if (keyframe == nullptr || !keyframe->cloud) {
        ++missing_keyframe_rejections;
        continue;
      }
      RegistrationConfig config;
      config.maximum_correspondence_distance_m = 1.5;
      config.maximum_iterations = 50;
      const Eigen::Matrix4f candidate_pose =
        keyframe->world_from_sensor.matrix().cast<float>();
      // PCL 1.12 NDT can dereference an invalid radius-search result for
      // sparse or degenerate online scans. Descriptor retrieval already gives
      // a map-frame pose seed, so keep the safety-critical process on the
      // bounded reciprocal-ICP path. NDT remains available in registration_core
      // for isolated offline experiments.
      std::optional<VerifiedCandidate> best_for_keyframe;
      std::optional<CandidateDiagnostic> best_attempt_for_keyframe;
      std::size_t candidate_registration_exceptions = 0U;
      const auto candidate_started = std::chrono::steady_clock::now();
      const auto record_attempt = [&best_attempt_for_keyframe](
        const CandidateDiagnostic & diagnostic)
        {
          if (!best_attempt_for_keyframe ||
            diagnostic.stage_depth > best_attempt_for_keyframe->stage_depth ||
            (diagnostic.stage_depth == best_attempt_for_keyframe->stage_depth &&
            diagnostic.forward_rmse < best_attempt_for_keyframe->forward_rmse))
          {
            best_attempt_for_keyframe = diagnostic;
          }
        };
      for (const double yaw_offset : yaw_offsets) {
        Eigen::Matrix4f initial = candidate_pose;
        const Eigen::Matrix3d rotation =
          Eigen::AngleAxisd(
          source_yaw + yaw_offset,
          Eigen::Vector3d::UnitZ()).toRotationMatrix() * source_tilt;
        initial.block<3, 3>(0, 0) = rotation.cast<float>();
        try {
          const auto registration = align_registration(
            cloud, keyframe->cloud, initial, config);
          const bool passed_forward_gate =
            registration.converged && std::isfinite(registration.fitness) &&
            registration.target_from_source.allFinite() &&
            registration.fitness <= maximum_registration_fitness_ &&
            registration.correspondence_points >=
            static_cast<std::size_t>(minimum_registration_correspondences_) &&
            registration.overlap_ratio >= minimum_registration_overlap_ratio_ &&
            std::isfinite(registration.inlier_rmse) &&
            registration.inlier_rmse <= maximum_registration_inlier_rmse_m_;
          CandidateDiagnostic diagnostic;
          diagnostic.keyframe_id = candidate.keyframe_id;
          diagnostic.descriptor_distance = candidate.descriptor_distance;
          diagnostic.yaw_offset = yaw_offset;
          diagnostic.forward_fitness = registration.fitness;
          diagnostic.forward_overlap = registration.overlap_ratio;
          diagnostic.forward_rmse = registration.inlier_rmse;
          diagnostic.forward_euclidean_rmse = registration.euclidean_inlier_rmse;
          diagnostic.forward_correspondences = registration.correspondence_points;
          diagnostic.forward_source_points = registration.source_points;
          diagnostic.forward_target_points = registration.target_points;
          diagnostic.reciprocal_correspondences =
            registration.reciprocal_correspondence_points;
          diagnostic.reciprocal_ratio = registration.reciprocal_ratio;
          diagnostic.plane_error_p90_m = registration.absolute_plane_error_p90_m;
          diagnostic.forward_converged = registration.converged;
          diagnostic.passed_forward_gate = passed_forward_gate;
          diagnostic.forward_effective_rank = registration.effective_rank;
          diagnostic.forward_condition_number = registration.condition_number;
          if (std::isfinite(registration.inlier_rmse) &&
            (!best_forward_attempt ||
            registration.inlier_rmse < best_forward_attempt->forward_rmse))
          {
            best_forward_attempt = diagnostic;
          }
          if (!passed_forward_gate) {
            ++forward_registration_rejections;
            record_attempt(diagnostic);
            continue;
          }
          Eigen::Isometry3d recovered_pose = Eigen::Isometry3d::Identity();
          recovered_pose.matrix() = registration.target_from_source.cast<double>();
          const Eigen::Isometry3d map_from_lio = recovered_pose * source_pose.inverse();
          diagnostic.alignment_translation_m = map_from_lio.translation().norm();
          diagnostic.alignment_rotation_rad = rotation_angle(map_from_lio.rotation());
          diagnostic.alignment_tilt_rad =
            tilt_difference(map_from_lio.rotation(), Eigen::Matrix3d::Identity());

          if (registration_method_ == "point_to_plane") {
            // The point-to-plane objective is intentionally asymmetric: the
            // dense static submap owns the surface normals. Re-optimizing the
            // entire submap against a sparse single scan changes visibility,
            // sampling, and the objective itself. Validate fixed-transform
            // mutual support instead of constructing a false inverse cycle.
            diagnostic.stage_depth = 1;
            diagnostic.rejection_stage = "reciprocal_support";
            diagnostic.passed_reciprocal_gate =
              registration.reciprocal_correspondence_points >=
              static_cast<std::size_t>(minimum_registration_reciprocal_correspondences_) &&
              registration.reciprocal_ratio >= minimum_registration_reciprocal_ratio_;
            if (!diagnostic.passed_reciprocal_gate) {
              ++reciprocal_support_rejections;
              record_attempt(diagnostic);
              continue;
            }
          } else {
            diagnostic.stage_depth = 1;
            diagnostic.rejection_stage = "reverse";
            const auto reverse = align_registration(
              keyframe->cloud, cloud, registration.target_from_source.inverse(), config);
            diagnostic.reverse_fitness = reverse.fitness;
            diagnostic.reverse_overlap = reverse.overlap_ratio;
            diagnostic.reverse_rmse = reverse.inlier_rmse;
            diagnostic.reverse_euclidean_rmse = reverse.euclidean_inlier_rmse;
            diagnostic.reverse_correspondences = reverse.correspondence_points;
            diagnostic.reverse_source_points = reverse.source_points;
            diagnostic.reverse_target_points = reverse.target_points;
            diagnostic.reverse_converged = reverse.converged;
            diagnostic.reverse_effective_rank = reverse.effective_rank;
            diagnostic.reverse_condition_number = reverse.condition_number;
            diagnostic.passed_reverse_gate =
              reverse.converged && std::isfinite(reverse.fitness) &&
              reverse.target_from_source.allFinite() &&
              reverse.fitness <= maximum_registration_fitness_ &&
              reverse.correspondence_points >=
              static_cast<std::size_t>(minimum_registration_correspondences_) &&
              reverse.overlap_ratio >= minimum_registration_overlap_ratio_ &&
              std::isfinite(reverse.inlier_rmse) &&
              reverse.inlier_rmse <= maximum_registration_inlier_rmse_m_;
            if (!diagnostic.passed_reverse_gate) {
              ++reverse_registration_rejections;
              record_attempt(diagnostic);
              continue;
            }
            diagnostic.stage_depth = 2;
            diagnostic.rejection_stage = "cycle";
            const Eigen::Matrix4f cycle =
              registration.target_from_source * reverse.target_from_source;
            diagnostic.cycle_translation_m = cycle.block<3, 1>(0, 3).norm();
            diagnostic.cycle_rotation_rad =
              Eigen::AngleAxisf(cycle.block<3, 3>(0, 0)).angle();
            if (diagnostic.cycle_translation_m >
              maximum_registration_cycle_translation_m_ ||
              diagnostic.cycle_rotation_rad >
              maximum_registration_cycle_rotation_rad_)
            {
              ++cycle_rejections;
              record_attempt(diagnostic);
              continue;
            }
          }
          diagnostic.stage_depth = 3;
          diagnostic.rejection_stage = "alignment";
          if (diagnostic.alignment_translation_m > maximum_alignment_translation_m_ ||
            diagnostic.alignment_rotation_rad > maximum_alignment_rotation_rad_ ||
            diagnostic.alignment_tilt_rad >
            maximum_roll_pitch_correction_rad_)
          {
            ++alignment_rejections;
            record_attempt(diagnostic);
            continue;
          }
          diagnostic.stage_depth = 4;
          diagnostic.rejection_stage = "verified";
          record_attempt(diagnostic);
          const double score =
            0.55 * (registration.inlier_rmse / maximum_registration_inlier_rmse_m_) +
            0.30 * (1.0 - registration.overlap_ratio) +
            0.15 * (candidate.descriptor_distance / maximum_descriptor_distance_);
          VerifiedCandidate verified{
            candidate.keyframe_id, candidate.descriptor_distance, registration,
            recovered_pose, map_from_lio, score};
          if (!best_for_keyframe || score < best_for_keyframe->score) {
            best_for_keyframe = std::move(verified);
          }
        } catch (const std::exception &) {
          ++registration_exceptions;
          ++candidate_registration_exceptions;
        }
      }
      const double candidate_elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - candidate_started).count();
      if (best_attempt_for_keyframe) {
        RCLCPP_INFO(
          get_logger(),
          "relocalization candidate chain: id=%zu method=%s descriptor=%.4f "
          "yaw_offset_deg=%.1f stage=%s forward=(converged=%d,passed=%d,points=%zu/%zu,"
          "matches=%zu,fitness=%.6f,overlap=%.3f,objective_rmse=%.3f,euclidean_rmse=%.3f,"
          "rank=%d,condition=%.3g) reciprocal=(passed=%d,matches=%zu,ratio=%.3f,p90=%.3f) "
          "reverse=(converged=%d,passed=%d,points=%zu/%zu,"
          "matches=%zu,fitness=%.6f,overlap=%.3f,objective_rmse=%.3f,euclidean_rmse=%.3f,"
          "rank=%d,condition=%.3g) cycle=(translation=%.3f,rotation=%.3f) "
          "alignment=(translation=%.3f,rotation=%.3f,tilt=%.3f) "
          "exceptions=%zu elapsed_ms=%.1f",
          best_attempt_for_keyframe->keyframe_id, registration_method_.c_str(),
          best_attempt_for_keyframe->descriptor_distance,
          best_attempt_for_keyframe->yaw_offset * 180.0 / 3.14159265358979323846,
          best_attempt_for_keyframe->rejection_stage.c_str(),
          best_attempt_for_keyframe->forward_converged,
          best_attempt_for_keyframe->passed_forward_gate,
          best_attempt_for_keyframe->forward_source_points,
          best_attempt_for_keyframe->forward_target_points,
          best_attempt_for_keyframe->forward_correspondences,
          best_attempt_for_keyframe->forward_fitness,
          best_attempt_for_keyframe->forward_overlap,
          best_attempt_for_keyframe->forward_rmse,
          best_attempt_for_keyframe->forward_euclidean_rmse,
          best_attempt_for_keyframe->forward_effective_rank,
          best_attempt_for_keyframe->forward_condition_number,
          best_attempt_for_keyframe->passed_reciprocal_gate,
          best_attempt_for_keyframe->reciprocal_correspondences,
          best_attempt_for_keyframe->reciprocal_ratio,
          best_attempt_for_keyframe->plane_error_p90_m,
          best_attempt_for_keyframe->reverse_converged,
          best_attempt_for_keyframe->passed_reverse_gate,
          best_attempt_for_keyframe->reverse_source_points,
          best_attempt_for_keyframe->reverse_target_points,
          best_attempt_for_keyframe->reverse_correspondences,
          best_attempt_for_keyframe->reverse_fitness,
          best_attempt_for_keyframe->reverse_overlap,
          best_attempt_for_keyframe->reverse_rmse,
          best_attempt_for_keyframe->reverse_euclidean_rmse,
          best_attempt_for_keyframe->reverse_effective_rank,
          best_attempt_for_keyframe->reverse_condition_number,
          best_attempt_for_keyframe->cycle_translation_m,
          best_attempt_for_keyframe->cycle_rotation_rad,
          best_attempt_for_keyframe->alignment_translation_m,
          best_attempt_for_keyframe->alignment_rotation_rad,
          best_attempt_for_keyframe->alignment_tilt_rad,
          candidate_registration_exceptions, candidate_elapsed_ms);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "relocalization candidate diagnostic: id=%zu method=%s descriptor=%.4f "
          "no finite forward alignment exceptions=%zu elapsed_ms=%.1f",
          candidate.keyframe_id, registration_method_.c_str(),
          candidate.descriptor_distance, candidate_registration_exceptions,
          candidate_elapsed_ms);
      }
      if (best_for_keyframe) {
        RCLCPP_INFO(
          get_logger(),
          "relocalization candidate verified: id=%zu descriptor=%.4f score=%.4f "
          "fitness=%.6f overlap=%.3f objective_rmse=%.3f euclidean_rmse=%.3f "
          "rank=%d condition=%.3g",
          best_for_keyframe->keyframe_id, best_for_keyframe->descriptor_distance,
          best_for_keyframe->score, best_for_keyframe->registration.fitness,
          best_for_keyframe->registration.overlap_ratio,
          best_for_keyframe->registration.inlier_rmse,
          best_for_keyframe->registration.euclidean_inlier_rmse,
          best_for_keyframe->registration.effective_rank,
          best_for_keyframe->registration.condition_number);
        verified_candidates.push_back(std::move(*best_for_keyframe));
      }
    }
    if (!verified_candidates.empty()) {
      std::sort(
        verified_candidates.begin(), verified_candidates.end(),
        [](const VerifiedCandidate & left, const VerifiedCandidate & right) {
          if (left.score == right.score) {
            return left.keyframe_id < right.keyframe_id;
          }
          return left.score < right.score;
        });
      const auto & best = verified_candidates.front();
      for (std::size_t index = 1U; index < verified_candidates.size(); ++index) {
        const auto & alternative = verified_candidates[index];
        const Eigen::Isometry3d separation =
          best.recovered_pose.inverse() * alternative.recovered_pose;
        const bool equivalent_hypotheses =
          separation.translation().norm() <= equivalent_candidate_translation_m_ &&
          rotation_angle(separation.rotation()) <= equivalent_candidate_rotation_rad_;
        if (equivalent_hypotheses) {
          continue;
        }
        if (alternative.score - best.score < minimum_verified_score_margin_) {
          std::ostringstream ambiguous;
          ambiguous << "ambiguous_verified_candidates:best=" << best.keyframe_id
                    << ",alternative=" << alternative.keyframe_id << ",score_margin="
                    << alternative.score - best.score << ",translation_separation="
                    << separation.translation().norm() << ",rotation_separation="
                    << rotation_angle(separation.rotation());
          search_failed_attempt(ambiguous.str());
          return;
        }
        break;
      }
      const auto consistency = success_consistency_gate_.observe(
        stamp_nanoseconds(message->header.stamp), best.map_from_lio);
      if (consistency.status == MultiFrameConsistencyStatus::STALE_OR_DUPLICATE) {
        publish_status(
          ResultMessage::SEARCHING, true, false, static_cast<uint32_t>(best.keyframe_id),
          best.descriptor_distance, best.registration.fitness,
          "multi_frame_consistency_stale_or_duplicate_query");
        return;
      }
      if (consistency.status != MultiFrameConsistencyStatus::CONFIRMED) {
        std::ostringstream pending_reason;
        pending_reason <<
          (consistency.status == MultiFrameConsistencyStatus::RESTARTED ?
          "multi_frame_consistency_restarted" : "multi_frame_consistency_accumulating")
                       << ":consistent_queries=" << consistency.consistent_queries
                       << "/" << success_consistency_required_queries_
                       << ",maximum_translation_delta_m="
                       << consistency.maximum_translation_delta_m
                       << ",maximum_rotation_delta_rad="
                       << consistency.maximum_rotation_delta_rad;
        publish_status(
          ResultMessage::SEARCHING, true, false, static_cast<uint32_t>(best.keyframe_id),
          best.descriptor_distance, best.registration.fitness, pending_reason.str());
        RCLCPP_INFO(
          get_logger(),
          "relocalization multi-frame gate: candidate=%zu status=%s queries=%zu/%d "
          "translation_delta=%.3f rotation_delta=%.3f",
          best.keyframe_id,
          consistency.status == MultiFrameConsistencyStatus::RESTARTED ?
          "restarted" : "accumulating",
          consistency.consistent_queries, success_consistency_required_queries_,
          consistency.maximum_translation_delta_m,
          consistency.maximum_rotation_delta_rad);
        return;
      }
      publish_status(
        ResultMessage::SUCCESS, false, true, static_cast<uint32_t>(best.keyframe_id),
        best.descriptor_distance, best.registration.fitness,
        "registration_candidate_accepted_awaiting_backend_epoch", &best.recovered_pose,
        &source_pose, &best.map_from_lio, &message->header.stamp);
      RCLCPP_WARN(
        get_logger(),
        "relocalization candidate accepted: id=%zu descriptor=%.4f score=%.4f "
        "fitness=%.6f; awaiting unified backend reset acknowledgement",
        best.keyframe_id, best.descriptor_distance, best.score,
        best.registration.fitness);
      request_active_ = false;
      success_consistency_gate_.reset();
      return;
    }
    std::ostringstream reason;
    reason << "no_candidate_passed_registration_gate:descriptor="
           << descriptor_gate_rejections << ",missing="
           << missing_keyframe_rejections << ",forward_registration="
           << forward_registration_rejections << ",reciprocal_support="
           << reciprocal_support_rejections << ",reverse_registration="
           << reverse_registration_rejections << ",cycle=" << cycle_rejections
           << ",alignment=" << alignment_rejections << ",exceptions="
           << registration_exceptions;
    if (best_forward_attempt) {
      reason << ",best_candidate=" << best_forward_attempt->keyframe_id
             << ",registration_method=" << registration_method_
             << ",best_descriptor=" << best_forward_attempt->descriptor_distance
             << ",best_yaw_offset_deg="
             << best_forward_attempt->yaw_offset * 180.0 / 3.14159265358979323846
             << ",best_converged=" << best_forward_attempt->forward_converged
             << ",best_fitness=" << best_forward_attempt->forward_fitness
             << ",best_overlap=" << best_forward_attempt->forward_overlap
             << ",best_correspondences=" << best_forward_attempt->forward_correspondences
             << ",best_objective_rmse=" << best_forward_attempt->forward_rmse
             << ",best_euclidean_rmse=" << best_forward_attempt->forward_euclidean_rmse
             << ",best_reciprocal_correspondences="
             << best_forward_attempt->reciprocal_correspondences
             << ",best_reciprocal_ratio=" << best_forward_attempt->reciprocal_ratio
             << ",best_plane_error_p90_m=" << best_forward_attempt->plane_error_p90_m
             << ",best_effective_rank=" << best_forward_attempt->forward_effective_rank
             << ",best_condition_number=" << best_forward_attempt->forward_condition_number;
    } else {
      reason << ",best_candidate=none";
    }
    search_failed_attempt(reason.str());
  }

  void search_timeout_callback()
  {
    // This wall timer only wakes the check. Expiration itself uses ROS time,
    // so paused or time-dilated simulation does not age keyframe evidence.
    process_pending_keyframe_cloud();
    if (!request_active_) {
      return;
    }
    const double now_s = now().seconds();
    if (now_s < request_started_s_) {
      request_started_s_ = now_s;
      success_consistency_gate_.reset();
      return;
    }
    if (now_s - request_started_s_ >= search_timeout_s_) {
      publish_status(ResultMessage::FAILED, false, false, 0U, 0.0, 0.0,
        "search_timeout");
      RCLCPP_ERROR(
        get_logger(), "relocalization timed out after %.2f ROS seconds",
        now_s - request_started_s_);
      request_active_ = false;
      success_consistency_gate_.reset();
    }
  }

  StaticKeyframeDatabase database_;
  rclcpp::Publisher<ResultMessage>::SharedPtr result_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr readiness_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr request_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fused_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_pose_sub_;
  rclcpp::Subscription<uf_interfaces::msg::LioDiagnostics>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<uf_interfaces::msg::ReliabilityScore>::SharedPtr lidar_score_sub_;
  rclcpp::Subscription<uf_interfaces::msg::SchedulerState>::SharedPtr scheduler_sub_;
  rclcpp::Subscription<CloudMessage>::SharedPtr keyframe_cloud_sub_;
  rclcpp::Subscription<CloudMessage>::SharedPtr query_cloud_sub_;
  rclcpp::TimerBase::SharedPtr search_timeout_timer_;
  nav_msgs::msg::Odometry::SharedPtr latest_fused_pose_;
  nav_msgs::msg::Odometry::SharedPtr latest_lio_pose_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> fused_pose_history_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> lio_pose_history_;
  std::deque<CloudMessage::SharedPtr> query_cloud_history_;
  CloudMessage::SharedPtr pending_keyframe_cloud_;
  CloudMessage::SharedPtr pending_query_cloud_;
  uf_interfaces::msg::LioDiagnostics::SharedPtr latest_diagnostics_;
  uf_interfaces::msg::ReliabilityScore::SharedPtr latest_lidar_score_;
  uf_interfaces::msg::SchedulerState::SharedPtr latest_scheduler_state_;
  std::string scheduler_health_ = "UNAVAILABLE";
  std::vector<std::string> keyframe_allowed_scheduler_states_;
  bool scheduler_lidar_enabled_{false};
  bool database_ready_{false};
  bool readiness_published_{false};
  bool request_active_{false};
  bool request_asserted_{false};
  uint64_t active_transaction_id_{0U};
  uint64_t last_transaction_id_{0U};
  double last_keyframe_attempt_s_{-std::numeric_limits<double>::infinity()};
  double last_query_attempt_s_{-std::numeric_limits<double>::infinity()};
  double request_started_s_{0.0};
  double keyframe_attempt_period_s_{0.5};
  double query_attempt_period_s_{0.5};
  double search_timeout_s_{6.0};
  int maximum_candidates_{3};
  int exclude_recent_keyframes_{3};
  int maximum_cloud_points_{1800};
  int minimum_registration_points_{30};
  int maximum_search_attempts_{10};
  int minimum_database_keyframes_{4};
  int search_attempt_count_{0};
  int success_consistency_required_queries_{3};
  double maximum_descriptor_distance_{0.35};
  std::string registration_method_{"icp"};
  double maximum_registration_fitness_{0.25};
  double maximum_alignment_translation_m_{30.0};
  double maximum_alignment_rotation_rad_{1.6};
  double maximum_roll_pitch_correction_rad_{0.35};
  double minimum_registration_overlap_ratio_{0.35};
  int minimum_registration_correspondences_{120};
  double minimum_registration_reciprocal_ratio_{0.20};
  int minimum_registration_reciprocal_correspondences_{80};
  double maximum_registration_inlier_rmse_m_{0.35};
  double maximum_registration_cycle_translation_m_{0.30};
  double maximum_registration_cycle_rotation_rad_{0.20};
  double minimum_verified_score_margin_{0.05};
  double equivalent_candidate_translation_m_{0.50};
  double equivalent_candidate_rotation_rad_{0.25};
  double success_consistency_translation_m_{0.15};
  double success_consistency_rotation_rad_{0.05};
  double keyframe_pose_tolerance_s_{0.12};
  double query_pose_tolerance_s_{0.08};
  double keyframe_sync_timeout_s_{0.5};
  double pending_keyframe_started_s_{std::numeric_limits<double>::quiet_NaN()};
  double maximum_quality_age_s_{2.0};
  double voxel_size_m_{0.25};
  int pose_history_size_{200};
  int query_cloud_history_size_{32};
  bool keyframe_consistency_diagnostics_enabled_{true};
  std::string expected_keyframe_frame_id_{"camera_init"};
  std::string expected_query_frame_id_{"body"};
  std::size_t keyframe_quality_rejections_{0U};
  std::size_t keyframe_quality_stale_rejections_{0U};
  std::size_t keyframe_frame_rejections_{0U};
  std::size_t keyframe_pose_sync_rejections_{0U};
  std::size_t keyframe_sync_waiting_cloud_skips_{0U};
  Eigen::Isometry3d last_keyframe_pose_{Eigen::Isometry3d::Identity()};
  bool has_last_keyframe_pose_{false};
  MultiFrameConsistencyGate success_consistency_gate_;
};

}  // namespace uf_relocalization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<uf_relocalization::RelocalizationNode>());
  } catch (const std::exception & error) {
    fprintf(stderr, "relocalization_node failed: %s\n", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
