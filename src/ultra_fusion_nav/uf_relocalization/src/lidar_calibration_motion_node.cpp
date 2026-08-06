#include "uf_relocalization/registration_core.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <rclcpp/rclcpp.hpp>
#include <uf_interfaces/msg/lidar_calibration_motion.hpp>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace uf_relocalization
{
namespace
{

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

builtin_interfaces::msg::Time scan_end_stamp(
  const livox_ros_driver2::msg::CustomMsg & message)
{
  std::uint64_t maximum_offset_ns = 0U;
  for (const auto & point : message.points) {
    maximum_offset_ns = std::max<std::uint64_t>(maximum_offset_ns, point.offset_time);
  }
  const std::uint64_t begin_ns =
    static_cast<std::uint64_t>(std::max<std::int32_t>(0, message.header.stamp.sec)) *
    1000000000ULL + message.header.stamp.nanosec;
  const std::uint64_t end_ns = begin_ns + maximum_offset_ns;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(end_ns / 1000000000ULL);
  stamp.nanosec = static_cast<std::uint32_t>(end_ns % 1000000000ULL);
  return stamp;
}

struct InlierEvidence
{
  std::size_t count{0};
  double residual_rms_m{std::numeric_limits<double>::infinity()};
  Eigen::Vector3d rotation_eigenvalues{Eigen::Vector3d::Zero()};
  double rotation_condition{std::numeric_limits<double>::infinity()};
};

bool finite_transform(const Eigen::Matrix4f & transform)
{
  if (!transform.allFinite()) {
    return false;
  }
  const Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
  return std::abs(rotation.determinant() - 1.0F) < 0.05F &&
         (rotation.transpose() * rotation - Eigen::Matrix3f::Identity()).norm() < 0.05F;
}

}  // namespace

class LidarCalibrationMotionNode final : public rclcpp::Node
{
public:
  using CloudMessage = livox_ros_driver2::msg::CustomMsg;
  using MotionMessage = uf_interfaces::msg::LidarCalibrationMotion;

  LidarCalibrationMotionNode()
  : Node("lidar_calibration_motion_node")
  {
    declare_parameter("input_topic", "/livox/lidar");
    declare_parameter("output_topic", "/calibration/lidar_relative_motion");
    declare_parameter("minimum_interval_s", 0.35);
    declare_parameter("maximum_interval_s", 0.50);
    declare_parameter("minimum_range_m", 0.50);
    declare_parameter("maximum_range_m", 30.0);
    declare_parameter("voxel_size_m", 0.30);
    declare_parameter("minimum_points", 150);
    declare_parameter("maximum_points", 2500);
    declare_parameter("maximum_correspondence_distance_m", 1.00);
    declare_parameter("maximum_iterations", 25);
    declare_parameter("maximum_fitness_score", 0.12);
    declare_parameter("minimum_inlier_ratio", 0.35);
    declare_parameter("maximum_translation_m", 1.5);
    declare_parameter("maximum_rotation_rad", 0.50);
    declare_parameter("maximum_rotation_information_condition", 200.0);

    minimum_interval_s_ = get_parameter("minimum_interval_s").as_double();
    maximum_interval_s_ = get_parameter("maximum_interval_s").as_double();
    minimum_range_m_ = get_parameter("minimum_range_m").as_double();
    maximum_range_m_ = get_parameter("maximum_range_m").as_double();
    voxel_size_m_ = get_parameter("voxel_size_m").as_double();
    minimum_points_ = std::max(20, static_cast<int>(get_parameter("minimum_points").as_int()));
    maximum_points_ = std::max(
      minimum_points_, static_cast<int>(get_parameter("maximum_points").as_int()));
    maximum_fitness_score_ = get_parameter("maximum_fitness_score").as_double();
    minimum_inlier_ratio_ = get_parameter("minimum_inlier_ratio").as_double();
    maximum_translation_m_ = get_parameter("maximum_translation_m").as_double();
    maximum_rotation_rad_ = get_parameter("maximum_rotation_rad").as_double();
    maximum_rotation_information_condition_ =
      get_parameter("maximum_rotation_information_condition").as_double();
    registration_config_.maximum_correspondence_distance_m =
      get_parameter("maximum_correspondence_distance_m").as_double();
    registration_config_.maximum_iterations = std::max(
      1, static_cast<int>(get_parameter("maximum_iterations").as_int()));

    if (minimum_interval_s_ <= 0.0 || maximum_interval_s_ <= minimum_interval_s_ ||
      minimum_range_m_ < 0.0 || maximum_range_m_ <= minimum_range_m_ ||
      voxel_size_m_ <= 0.0 || registration_config_.maximum_correspondence_distance_m <= 0.0 ||
      maximum_fitness_score_ <= 0.0 || minimum_inlier_ratio_ <= 0.0 ||
      minimum_inlier_ratio_ > 1.0 || maximum_translation_m_ <= 0.0 ||
      maximum_rotation_rad_ <= 0.0 || maximum_rotation_information_condition_ <= 1.0)
    {
      throw std::invalid_argument("invalid LiDAR calibration motion limits");
    }

    publisher_ = create_publisher<MotionMessage>(
      get_parameter("output_topic").as_string(), rclcpp::QoS(10).best_effort());
    subscription_ = create_subscription<CloudMessage>(
      get_parameter("input_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&LidarCalibrationMotionNode::cloud_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(),
      "raw-LiDAR-only calibration motion active: %s -> %s at <= %.2f Hz",
      get_parameter("input_topic").as_string().c_str(),
      get_parameter("output_topic").as_string().c_str(), 1.0 / minimum_interval_s_);
  }

private:
  Cloud::Ptr convert_cloud(const CloudMessage & message) const
  {
    auto ranged = std::make_shared<Cloud>();
    ranged->reserve(message.points.size());
    const double minimum_squared = minimum_range_m_ * minimum_range_m_;
    const double maximum_squared = maximum_range_m_ * maximum_range_m_;
    for (const auto & point : message.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const double squared = static_cast<double>(point.x) * point.x +
        static_cast<double>(point.y) * point.y + static_cast<double>(point.z) * point.z;
      if (squared >= minimum_squared && squared <= maximum_squared) {
        ranged->push_back(pcl::PointXYZ{point.x, point.y, point.z});
      }
    }

    auto filtered = std::make_shared<Cloud>();
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(ranged);
    const float leaf = static_cast<float>(voxel_size_m_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*filtered);
    if (filtered->size() <= static_cast<std::size_t>(maximum_points_)) {
      return filtered;
    }
    auto limited = std::make_shared<Cloud>();
    limited->reserve(static_cast<std::size_t>(maximum_points_));
    const double stride = static_cast<double>(filtered->size()) /
      static_cast<double>(maximum_points_);
    for (int index = 0; index < maximum_points_; ++index) {
      limited->push_back((*filtered)[static_cast<std::size_t>(index * stride)]);
    }
    return limited;
  }

  InlierEvidence inlier_evidence(
    const Cloud::ConstPtr & source, const Cloud::ConstPtr & target,
    const Eigen::Matrix4f & target_from_source) const
  {
    Cloud aligned;
    pcl::transformPointCloud(*source, aligned, target_from_source);
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(target);
    std::vector<int> index(1);
    std::vector<float> squared_distance(1);
    const float maximum_squared = static_cast<float>(
      registration_config_.maximum_correspondence_distance_m *
      registration_config_.maximum_correspondence_distance_m);
    InlierEvidence evidence;
    double squared_residual_sum = 0.0;
    Eigen::Matrix3d rotation_information = Eigen::Matrix3d::Zero();
    for (const auto & point : aligned) {
      if (tree.nearestKSearch(point, 1, index, squared_distance) == 1 &&
        squared_distance.front() <= maximum_squared)
      {
        ++evidence.count;
        squared_residual_sum += squared_distance.front();
        const Eigen::Vector3d point_vector(
          static_cast<double>(point.x), static_cast<double>(point.y),
          static_cast<double>(point.z));
        const Eigen::Matrix3d skew = (Eigen::Matrix3d() <<
          0.0, -point_vector.z(), point_vector.y(),
          point_vector.z(), 0.0, -point_vector.x(),
          -point_vector.y(), point_vector.x(), 0.0).finished();
        rotation_information += skew.transpose() * skew;
      }
    }
    if (evidence.count > 0U) {
      evidence.residual_rms_m = std::sqrt(
        squared_residual_sum / static_cast<double>(evidence.count));
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(rotation_information);
      if (solver.info() == Eigen::Success) {
        evidence.rotation_eigenvalues = solver.eigenvalues().cwiseMax(0.0);
        const double minimum = evidence.rotation_eigenvalues.x();
        const double maximum = evidence.rotation_eigenvalues.z();
        if (minimum > 1.0e-12) {
          evidence.rotation_condition = maximum / minimum;
        }
      }
    }
    return evidence;
  }

  void reset_reference(const Cloud::Ptr & cloud, const CloudMessage & message, double stamp_s)
  {
    previous_cloud_ = cloud;
    previous_stamp_ = scan_end_stamp(message);
    previous_stamp_s_ = stamp_s;
    previous_frame_ = message.header.frame_id;
    next_initial_guess_ = Eigen::Matrix4f::Identity();
  }

  void cloud_callback(const CloudMessage::SharedPtr message)
  {
    if (!message || message->points.empty()) {
      return;
    }
    const auto end_stamp = scan_end_stamp(*message);
    const double stamp_s = stamp_seconds(end_stamp);
    if (!std::isfinite(stamp_s) || stamp_s <= 0.0) {
      return;
    }
    if (previous_cloud_ && stamp_s - previous_stamp_s_ < minimum_interval_s_) {
      return;
    }
    auto cloud = convert_cloud(*message);
    if (!previous_cloud_ || message->header.frame_id != previous_frame_) {
      reset_reference(cloud, *message, stamp_s);
      return;
    }

    MotionMessage output;
    output.header = message->header;
    output.header.stamp = end_stamp;
    output.start_stamp = previous_stamp_;
    output.provenance = MotionMessage::RAW_LIDAR_SCAN_TO_SCAN;
    output.imu_aided = false;
    output.backend_aided = false;
    output.source_points = static_cast<uint32_t>(cloud->size());
    output.target_points = static_cast<uint32_t>(previous_cloud_->size());
    output.method = "raw_scan_to_scan_gicp";
    output.rotation_convention = "R_L_previous_from_L_current";
    output.relative_rotation.w = 1.0;

    const double interval_s = stamp_s - previous_stamp_s_;
    if (interval_s <= 0.0 || interval_s > maximum_interval_s_) {
      output.reason = "invalid_scan_interval";
      publisher_->publish(output);
      ++rejected_;
      reset_reference(cloud, *message, stamp_s);
      return;
    }
    if (cloud->size() < static_cast<std::size_t>(minimum_points_) ||
      previous_cloud_->size() < static_cast<std::size_t>(minimum_points_))
    {
      output.reason = "insufficient_points";
      publisher_->publish(output);
      ++rejected_;
      reset_reference(cloud, *message, stamp_s);
      return;
    }

    RegistrationResult result;
    try {
      result = align_gicp(cloud, previous_cloud_, next_initial_guess_, registration_config_);
    } catch (const std::exception & error) {
      output.reason = std::string("registration_exception:") + error.what();
      publisher_->publish(output);
      ++rejected_;
      reset_reference(cloud, *message, stamp_s);
      return;
    }
    output.converged = result.converged;
    output.fitness_score = result.fitness;
    if (finite_transform(result.target_from_source)) {
      const auto evidence = inlier_evidence(
        cloud, previous_cloud_, result.target_from_source);
      output.inlier_points = static_cast<uint32_t>(evidence.count);
      output.inlier_ratio =
        static_cast<double>(output.inlier_points) /
        static_cast<double>(std::max<std::size_t>(1, cloud->size()));
      output.residual_rms_m = evidence.residual_rms_m;
      for (std::size_t index = 0; index < 3U; ++index) {
        output.rotation_information_eigenvalues[index] =
          evidence.rotation_eigenvalues[static_cast<Eigen::Index>(index)];
      }
      output.rotation_information_condition = evidence.rotation_condition;
      const Eigen::Vector3f translation = result.target_from_source.block<3, 1>(0, 3);
      const Eigen::Quaternionf rotation(result.target_from_source.block<3, 3>(0, 0));
      output.relative_translation.x = translation.x();
      output.relative_translation.y = translation.y();
      output.relative_translation.z = translation.z();
      output.relative_rotation.x = rotation.x();
      output.relative_rotation.y = rotation.y();
      output.relative_rotation.z = rotation.z();
      output.relative_rotation.w = rotation.w();
      const double rotation_angle = Eigen::AngleAxisf(rotation.normalized()).angle();
      output.accepted = result.converged && std::isfinite(result.fitness) &&
        result.fitness <= maximum_fitness_score_ &&
        output.inlier_ratio >= minimum_inlier_ratio_ &&
        translation.norm() <= maximum_translation_m_ &&
        rotation_angle <= maximum_rotation_rad_ &&
        evidence.rotation_condition <= maximum_rotation_information_condition_;
      if (!result.converged) {
        output.reason = "icp_not_converged";
      } else if (!std::isfinite(result.fitness) || result.fitness > maximum_fitness_score_) {
        output.reason = "fitness_gate";
      } else if (output.inlier_ratio < minimum_inlier_ratio_) {
        output.reason = "inlier_ratio_gate";
      } else if (translation.norm() > maximum_translation_m_) {
        output.reason = "translation_gate";
      } else if (rotation_angle > maximum_rotation_rad_) {
        output.reason = "rotation_gate";
      } else if (evidence.rotation_condition > maximum_rotation_information_condition_) {
        output.reason = "rotation_information_gate";
      } else {
        output.reason = "accepted";
      }
    } else {
      output.reason = "nonfinite_transform";
    }

    publisher_->publish(output);
    if (output.accepted) {
      ++accepted_;
      next_initial_guess_ = result.target_from_source;
    } else {
      ++rejected_;
      next_initial_guess_ = Eigen::Matrix4f::Identity();
    }
    if ((accepted_ + rejected_) % 50U == 0U) {
      RCLCPP_INFO(
        get_logger(), "calibration motion summary: accepted=%zu rejected=%zu last=%s",
        accepted_, rejected_, output.reason.c_str());
    }
    previous_cloud_ = cloud;
    previous_stamp_ = end_stamp;
    previous_stamp_s_ = stamp_s;
    previous_frame_ = message->header.frame_id;
  }

  RegistrationConfig registration_config_;
  rclcpp::Publisher<MotionMessage>::SharedPtr publisher_;
  rclcpp::Subscription<CloudMessage>::SharedPtr subscription_;
  Cloud::Ptr previous_cloud_;
  builtin_interfaces::msg::Time previous_stamp_;
  std::string previous_frame_;
  double previous_stamp_s_{-std::numeric_limits<double>::infinity()};
  Eigen::Matrix4f next_initial_guess_{Eigen::Matrix4f::Identity()};
  double minimum_interval_s_{0.35};
  double maximum_interval_s_{0.50};
  double minimum_range_m_{0.50};
  double maximum_range_m_{30.0};
  double voxel_size_m_{0.30};
  int minimum_points_{150};
  int maximum_points_{2500};
  double maximum_fitness_score_{0.12};
  double minimum_inlier_ratio_{0.35};
  double maximum_translation_m_{1.5};
  double maximum_rotation_rad_{0.50};
  double maximum_rotation_information_condition_{200.0};
  std::size_t accepted_{0};
  std::size_t rejected_{0};
};

}  // namespace uf_relocalization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<uf_relocalization::LidarCalibrationMotionNode>());
  } catch (const std::exception & error) {
    fprintf(stderr, "lidar_calibration_motion_node failed: %s\n", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
