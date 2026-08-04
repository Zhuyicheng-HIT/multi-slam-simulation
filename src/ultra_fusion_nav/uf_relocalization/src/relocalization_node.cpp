#include "uf_relocalization/descriptor_core.hpp"
#include "uf_relocalization/keyframe_database.hpp"
#include "uf_relocalization/registration_core.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace uf_relocalization
{
namespace
{

using ResultMessage = uf_interfaces::msg::RelocalizationResult;
using CloudMessage = sensor_msgs::msg::PointCloud2;

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
    declare_parameter("cloud_topic", "/lidar/static_cloud");
    declare_parameter("fused_pose_topic", "/fusion/unified/odom");
    declare_parameter("source_lio_pose_topic", "/lio/odom");
    declare_parameter("diagnostics_topic", "/lio/diagnostics");
    declare_parameter("lidar_score_topic", "/reliability/lidar_score");
    declare_parameter("scheduler_topic", "/reliability/scheduler_state");
    declare_parameter("request_topic", "/relocalization/request");
    declare_parameter("result_topic", "/relocalization/result");
    declare_parameter("keyframe_attempt_period_s", 0.5);
    declare_parameter("maximum_candidates", 3);
    declare_parameter("exclude_recent_keyframes", 3);
    declare_parameter("maximum_descriptor_distance", 0.35);
    declare_parameter("maximum_registration_fitness", 0.25);
    declare_parameter("maximum_alignment_translation_m", 30.0);
    declare_parameter("maximum_alignment_rotation_rad", 1.6);
    declare_parameter("maximum_cloud_points", 1800);
    declare_parameter("voxel_size_m", 0.25);
    declare_parameter("minimum_registration_points", 30);
    declare_parameter("maximum_search_attempts", 10);

    keyframe_attempt_period_s_ = get_parameter("keyframe_attempt_period_s").as_double();
    maximum_candidates_ = std::max(
      1, static_cast<int>(get_parameter("maximum_candidates").as_int()));
    exclude_recent_keyframes_ = std::max(
      0, static_cast<int>(get_parameter("exclude_recent_keyframes").as_int()));
    maximum_descriptor_distance_ = get_parameter("maximum_descriptor_distance").as_double();
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
    if (keyframe_attempt_period_s_ <= 0.0 || maximum_descriptor_distance_ <= 0.0 ||
      maximum_registration_fitness_ <= 0.0 || maximum_alignment_translation_m_ <= 0.0 ||
      maximum_alignment_rotation_rad_ <= 0.0 || voxel_size_m_ <= 0.0)
    {
      throw std::invalid_argument("invalid relocalization node limits");
    }

    result_pub_ = create_publisher<ResultMessage>(
      get_parameter("result_topic").as_string(), rclcpp::QoS(10).reliable());
    request_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("request_topic").as_string(), rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        request_active_ = message->data;
        if (request_active_) {
          search_attempt_count_ = 0;
          publish_status(ResultMessage::SEARCHING, false, false, 0U, 0.0, 0.0,
            "searching_static_keyframes");
        }
      });
    fused_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("fused_pose_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        latest_fused_pose_ = message;
      });
    lio_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("source_lio_pose_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        latest_lio_pose_ = message;
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
    cloud_sub_ = create_subscription<CloudMessage>(
      get_parameter("cloud_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&RelocalizationNode::cloud_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(),
      "online relocalization active: static keyframes + ESF retrieval + NDT/ICP verification; "
      "automatic request remains external");
  }

private:
  static KeyframeDatabaseConfig make_database_config()
  {
    KeyframeDatabaseConfig config;
    config.minimum_map_quality = 0.60;
    config.minimum_feature_repeatability = 0.70;
    config.maximum_dynamic_ratio = 0.15;
    config.maximum_lidar_degradation = 0.75;
    config.minimum_translation_spacing_m = 1.0;
    config.minimum_rotation_spacing_rad = 0.26;
    config.maximum_keyframes = 500;
    return config;
  }

  static bool stamp_close(const builtin_interfaces::msg::Time & left,
    const builtin_interfaces::msg::Time & right, double maximum_age_s)
  {
    const double left_s = static_cast<double>(left.sec) + left.nanosec * 1.0e-9;
    const double right_s = static_cast<double>(right.sec) + right.nanosec * 1.0e-9;
    return std::isfinite(left_s) && std::isfinite(right_s) &&
      std::abs(left_s - right_s) <= maximum_age_s;
  }

  Cloud::Ptr convert_cloud(const CloudMessage & message) const
  {
    auto cloud = std::make_shared<Cloud>();
    pcl::fromROSMsg(message, *cloud);
    if (cloud->empty()) {
      return cloud;
    }
    auto filtered = std::make_shared<Cloud>();
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(
      static_cast<float>(voxel_size_m_), static_cast<float>(voxel_size_m_),
      static_cast<float>(voxel_size_m_));
    voxel.filter(*filtered);
    if (filtered->size() > static_cast<std::size_t>(maximum_cloud_points_)) {
      auto limited = std::make_shared<Cloud>();
      limited->reserve(static_cast<std::size_t>(maximum_cloud_points_));
      const double stride = static_cast<double>(filtered->size()) /
        static_cast<double>(maximum_cloud_points_);
      for (int index = 0; index < maximum_cloud_points_; ++index) {
        limited->push_back((*filtered)[static_cast<std::size_t>(index * stride)]);
      }
      return limited;
    }
    return filtered;
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
      (scheduler_health_ == "NORMAL" || scheduler_health_ == "RECOVERED");
    return quality;
  }

  void publish_status(uint8_t state, bool request_active, bool accepted,
    uint32_t candidate_id, double descriptor_distance, double fitness,
    const std::string & reason, const Eigen::Isometry3d * pose = nullptr,
    const Eigen::Isometry3d * source_pose = nullptr,
    const Eigen::Isometry3d * map_from_lio = nullptr)
  {
    ResultMessage message;
    if (source_pose != nullptr && latest_lio_pose_) {
      message.header.stamp = latest_lio_pose_->header.stamp;
    } else {
      message.header.stamp = now();
    }
    message.header.frame_id = latest_fused_pose_ ? latest_fused_pose_->header.frame_id : "camera_init";
    message.state = state;
    message.state_name = state == ResultMessage::SUCCESS ? "SUCCESS" :
      (state == ResultMessage::FAILED ? "FAILED" :
      (state == ResultMessage::SEARCHING ? "SEARCHING" : "IDLE"));
    message.request_active = request_active;
    message.accepted = accepted;
    message.candidate_id = candidate_id;
    message.descriptor_distance = static_cast<float>(descriptor_distance);
    message.registration_fitness = static_cast<float>(fitness);
    message.reset_counter = reset_counter_;
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

  void cloud_callback(const CloudMessage::SharedPtr message)
  {
    if (!message || message->width * message->height == 0U) {
      return;
    }
    auto cloud = convert_cloud(*message);
    if (!cloud || cloud->size() < static_cast<std::size_t>(minimum_registration_points_)) {
      if (request_active_) {
        publish_status(ResultMessage::FAILED, false, false, 0U, 0.0, 0.0,
          "insufficient_static_cloud");
        request_active_ = false;
      }
      return;
    }
    if (!latest_fused_pose_ || !latest_lio_pose_ ||
      !stamp_close(message->header.stamp, latest_fused_pose_->header.stamp, 0.50) ||
      !stamp_close(message->header.stamp, latest_lio_pose_->header.stamp, 0.50) ||
      !finite_pose(latest_fused_pose_->pose.pose) ||
      !finite_pose(latest_lio_pose_->pose.pose))
    {
      return;
    }
    const double stamp_s = static_cast<double>(message->header.stamp.sec) +
      message->header.stamp.nanosec * 1.0e-9;
    if (!std::isfinite(stamp_s) || stamp_s - last_keyframe_attempt_s_ < keyframe_attempt_period_s_) {
      return;
    }
    last_keyframe_attempt_s_ = stamp_s;

    KeyframeQuality quality;
    Eigen::Isometry3d fused_pose = Eigen::Isometry3d::Identity();
    if (!request_active_) {
      quality = current_quality();
      if (!quality.scheduler_lidar_enabled ||
        quality.map_quality < 0.60 || quality.feature_repeatability < 0.70 ||
        quality.dynamic_ratio > 0.15 || quality.lidar_degradation > 0.75)
      {
        return;
      }
      fused_pose = pose_to_isometry(latest_fused_pose_->pose.pose);
      if (has_last_keyframe_pose_) {
        const double translation =
          (fused_pose.translation() - last_keyframe_pose_.translation()).norm();
        const double rotation = rotation_angle(
          last_keyframe_pose_.rotation().transpose() * fused_pose.rotation());
        if (translation < 1.0 && rotation < 0.26) {
          return;
        }
      }
    }

    std::vector<float> descriptor;
    try {
      descriptor = compute_esf_descriptor(cloud);
    } catch (const std::exception & error) {
      if (request_active_) {
        publish_status(ResultMessage::FAILED, false, false, 0U, 0.0, 0.0,
          std::string("descriptor_failed:") + error.what());
        request_active_ = false;
      }
      return;
    }

    if (!request_active_) {
      const auto admission = database_.try_insert(
        stamp_s, fused_pose, cloud, descriptor, quality);
      if (admission.accepted) {
        last_keyframe_pose_ = fused_pose;
        has_last_keyframe_pose_ = true;
      }
      return;
    }

    const auto candidates = database_.query(
      descriptor, static_cast<std::size_t>(maximum_candidates_), exclude_recent_keyframes_);
    const auto source_pose = pose_to_isometry(latest_lio_pose_->pose.pose);
    for (const auto & candidate : candidates) {
      if (candidate.descriptor_distance > maximum_descriptor_distance_) {
        continue;
      }
      const auto * keyframe = database_.find(candidate.keyframe_id);
      if (keyframe == nullptr || !keyframe->cloud) {
        continue;
      }
      RegistrationConfig config;
      config.maximum_correspondence_distance_m = 1.5;
      config.maximum_iterations = 50;
      const auto ndt = align_ndt(
        cloud, keyframe->cloud, Eigen::Matrix4f::Identity(), config);
      const Eigen::Matrix4f initial = ndt.converged ?
        ndt.target_from_source : Eigen::Matrix4f::Identity();
      const auto icp = align_icp(cloud, keyframe->cloud, initial, config);
      if (!icp.converged || icp.fitness > maximum_registration_fitness_) {
        continue;
      }
      Eigen::Isometry3d map_from_lio = Eigen::Isometry3d::Identity();
      map_from_lio.matrix() = icp.target_from_source.cast<double>();
      if (map_from_lio.translation().norm() > maximum_alignment_translation_m_ ||
        rotation_angle(map_from_lio.rotation()) > maximum_alignment_rotation_rad_)
      {
        continue;
      }
      const Eigen::Isometry3d recovered_pose = map_from_lio * source_pose;
      ++reset_counter_;
      publish_status(
        ResultMessage::SUCCESS, false, true, static_cast<uint32_t>(candidate.keyframe_id),
        candidate.descriptor_distance, icp.fitness, "icp_verified", &recovered_pose,
        &source_pose, &map_from_lio);
      request_active_ = false;
      return;
    }
    ++search_attempt_count_;
    if (search_attempt_count_ >= maximum_search_attempts_) {
      publish_status(ResultMessage::FAILED, false, false, 0U, 0.0, 0.0,
        "search_attempt_limit_reached");
      request_active_ = false;
    } else {
      publish_status(ResultMessage::SEARCHING, true, false, 0U, 0.0, 0.0,
        "no_candidate_passed_registration_gate");
    }
  }

  StaticKeyframeDatabase database_;
  rclcpp::Publisher<ResultMessage>::SharedPtr result_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr request_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fused_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_pose_sub_;
  rclcpp::Subscription<uf_interfaces::msg::LioDiagnostics>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<uf_interfaces::msg::ReliabilityScore>::SharedPtr lidar_score_sub_;
  rclcpp::Subscription<uf_interfaces::msg::SchedulerState>::SharedPtr scheduler_sub_;
  rclcpp::Subscription<CloudMessage>::SharedPtr cloud_sub_;
  nav_msgs::msg::Odometry::SharedPtr latest_fused_pose_;
  nav_msgs::msg::Odometry::SharedPtr latest_lio_pose_;
  uf_interfaces::msg::LioDiagnostics::SharedPtr latest_diagnostics_;
  uf_interfaces::msg::ReliabilityScore::SharedPtr latest_lidar_score_;
  std::string scheduler_health_ = "UNAVAILABLE";
  bool scheduler_lidar_enabled_{false};
  bool request_active_{false};
  double last_keyframe_attempt_s_{-std::numeric_limits<double>::infinity()};
  double keyframe_attempt_period_s_{0.5};
  int maximum_candidates_{3};
  int exclude_recent_keyframes_{3};
  int maximum_cloud_points_{1800};
  int minimum_registration_points_{30};
  int maximum_search_attempts_{10};
  int search_attempt_count_{0};
  double maximum_descriptor_distance_{0.35};
  double maximum_registration_fitness_{0.25};
  double maximum_alignment_translation_m_{30.0};
  double maximum_alignment_rotation_rad_{1.6};
  double voxel_size_m_{0.25};
  uint32_t reset_counter_{0};
  Eigen::Isometry3d last_keyframe_pose_{Eigen::Isometry3d::Identity()};
  bool has_last_keyframe_pose_{false};
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
