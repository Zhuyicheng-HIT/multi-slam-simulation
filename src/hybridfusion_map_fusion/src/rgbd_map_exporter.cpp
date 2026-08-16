#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace hybridfusion_map_fusion
{

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using ExactPolicy = message_filters::sync_policies::ExactTime<
  sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

class RgbdMapExporter final : public rclcpp::Node
{
public:
  RgbdMapExporter()
  : Node("hybridfusion_rgbd_map_exporter"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    enabled_ = declare_parameter("enabled", false);
    rgb_topic_ = declare_parameter("rgb_topic", "/sensors/rgbd/color");
    depth_topic_ = declare_parameter("depth_topic", "/sensors/rgbd/depth");
    camera_info_topic_ = declare_parameter(
      "camera_info_topic", "/front/d435i/color/camera_info");
    global_frame_ = declare_parameter("global_frame", "odom");
    camera_frame_ = declare_parameter("camera_frame", "front_d435i_color_optical_frame");
    output_dir_ = expand_home(declare_parameter(
      "output_dir", "~/.ros/hybridfusion_export/visual"));
    min_keyframe_translation_m_ = declare_parameter("min_keyframe_translation_m", 0.20);
    min_keyframe_rotation_deg_ = declare_parameter("min_keyframe_rotation_deg", 7.5);
    min_keyframe_period_s_ = declare_parameter("min_keyframe_period_s", 0.75);
    depth_min_m_ = declare_parameter("depth_min_m", 0.30);
    depth_max_m_ = declare_parameter("depth_max_m", 6.0);
    pixel_stride_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter("pixel_stride", 3)));
    max_points_per_keyframe_ = static_cast<int>(std::max<std::int64_t>(
        1000, declare_parameter("max_points_per_keyframe", 45000)));
    map_voxel_leaf_m_ = declare_parameter("map_voxel_leaf_m", 0.06);
    save_keyframes_ = declare_parameter("save_keyframes", true);

    save_service_ = create_service<std_srvs::srv::Trigger>(
      "~/save", std::bind(
        &RgbdMapExporter::save_service, this, std::placeholders::_1, std::placeholders::_2));

    if (!enabled_) {
      RCLCPP_INFO(get_logger(), "RGB-D map export is disabled (default); no source subscriptions created");
      return;
    }
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        camera_info_ = std::move(message);
      });
    std::filesystem::create_directories(std::filesystem::path(output_dir_) / "keyframes");
    rgb_sub_.subscribe(this, rgb_topic_, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
    synchronizer_ = std::make_shared<message_filters::Synchronizer<ExactPolicy>>(
      ExactPolicy(8), rgb_sub_, depth_sub_);
    synchronizer_->registerCallback(std::bind(
      &RgbdMapExporter::rgbd_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(
      get_logger(), "Enabled exact RGB-D keyframe export: %s + %s -> %s",
      rgb_topic_.c_str(), depth_topic_.c_str(), output_dir_.c_str());
  }

private:
  static std::string expand_home(const std::string & path)
  {
    if (path == "~" || path.rfind("~/", 0) == 0) {
      const char * home = std::getenv("HOME");
      if (home == nullptr) {
        throw std::runtime_error("HOME is not set; use an absolute output_dir");
      }
      return std::string(home) + path.substr(1);
    }
    return path;
  }

  static double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  static double rotation_angle_deg(const Eigen::Matrix3d & a, const Eigen::Matrix3d & b)
  {
    constexpr double kRadToDeg = 57.2957795130823208768;
    return std::abs(Eigen::AngleAxisd(a.transpose() * b).angle()) * kRadToDeg;
  }

  float depth_meters(const sensor_msgs::msg::Image & depth, int u, int v) const
  {
    const std::size_t offset = static_cast<std::size_t>(v) * depth.step;
    if (depth.encoding == "16UC1" || depth.encoding == "mono16") {
      const auto * row = reinterpret_cast<const std::uint16_t *>(depth.data.data() + offset);
      return static_cast<float>(row[u]) * 0.001F;
    }
    if (depth.encoding == "32FC1") {
      const auto * row = reinterpret_cast<const float *>(depth.data.data() + offset);
      return row[u];
    }
    return std::numeric_limits<float>::quiet_NaN();
  }

  void color_at(
    const sensor_msgs::msg::Image & rgb, int u, int v,
    std::uint8_t & red, std::uint8_t & green, std::uint8_t & blue) const
  {
    const auto * row = rgb.data.data() + static_cast<std::size_t>(v) * rgb.step;
    if (rgb.encoding == "rgb8" || rgb.encoding == "rgba8") {
      const int channels = rgb.encoding == "rgba8" ? 4 : 3;
      red = row[u * channels];
      green = row[u * channels + 1];
      blue = row[u * channels + 2];
    } else if (rgb.encoding == "bgr8" || rgb.encoding == "bgra8") {
      const int channels = rgb.encoding == "bgra8" ? 4 : 3;
      blue = row[u * channels];
      green = row[u * channels + 1];
      red = row[u * channels + 2];
    } else if (rgb.encoding == "mono8") {
      red = green = blue = row[u];
    } else {
      red = green = blue = 180;
    }
  }

  bool accept_keyframe(const Eigen::Isometry3d & pose, double stamp)
  {
    if (!have_keyframe_) {
      return true;
    }
    const double translation = (pose.translation() - last_keyframe_pose_.translation()).norm();
    const double rotation = rotation_angle_deg(last_keyframe_pose_.rotation(), pose.rotation());
    return (stamp - last_keyframe_stamp_) >= min_keyframe_period_s_ &&
           (translation >= min_keyframe_translation_m_ || rotation >= min_keyframe_rotation_deg_);
  }

  void rgbd_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth)
  {
    if (!enabled_ || rgb->header.stamp != depth->header.stamp ||
      rgb->width != depth->width || rgb->height != depth->height)
    {
      return;
    }
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      info = camera_info_;
    }
    if (!info || info->k[0] <= 0.0 || info->k[4] <= 0.0) {
      ++missing_calibration_;
      return;
    }

    geometry_msgs::msg::TransformStamped transform_message;
    try {
      transform_message = tf_buffer_.lookupTransform(
        global_frame_, camera_frame_, rclcpp::Time(rgb->header.stamp),
        rclcpp::Duration::from_seconds(0.15));
    } catch (const tf2::TransformException & error) {
      ++missing_transform_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Skipping RGB-D frame without %s <- %s TF: %s",
        global_frame_.c_str(), camera_frame_.c_str(), error.what());
      return;
    }
    const Eigen::Isometry3d pose = tf2::transformToEigen(transform_message.transform);
    const double stamp = stamp_seconds(rgb->header.stamp);
    if (!accept_keyframe(pose, stamp)) {
      return;
    }

    int stride = pixel_stride_;
    const int estimated = static_cast<int>(rgb->width * rgb->height) / (stride * stride);
    if (estimated > max_points_per_keyframe_) {
      stride = std::max(stride, static_cast<int>(std::ceil(
          std::sqrt(static_cast<double>(rgb->width * rgb->height) /
          static_cast<double>(max_points_per_keyframe_)))));
    }
    CloudT keyframe;
    keyframe.reserve(static_cast<std::size_t>(max_points_per_keyframe_));
    const double fx = info->k[0];
    const double fy = info->k[4];
    const double cx = info->k[2];
    const double cy = info->k[5];
    for (int v = 0; v < static_cast<int>(depth->height); v += stride) {
      for (int u = 0; u < static_cast<int>(depth->width); u += stride) {
        const float z = depth_meters(*depth, u, v);
        if (!std::isfinite(z) || z < depth_min_m_ || z > depth_max_m_) {
          continue;
        }
        const Eigen::Vector3d camera_point(
          (static_cast<double>(u) - cx) * z / fx,
          (static_cast<double>(v) - cy) * z / fy, z);
        const Eigen::Vector3d global_point = pose * camera_point;
        PointT point;
        point.x = static_cast<float>(global_point.x());
        point.y = static_cast<float>(global_point.y());
        point.z = static_cast<float>(global_point.z());
        color_at(*rgb, u, v, point.r, point.g, point.b);
        keyframe.push_back(point);
      }
    }
    if (keyframe.empty()) {
      return;
    }
    keyframe.width = static_cast<std::uint32_t>(keyframe.size());
    keyframe.height = 1;
    keyframe.is_dense = true;

    std::lock_guard<std::mutex> lock(mutex_);
    const std::size_t index = keyframe_count_++;
    accumulated_ += keyframe;
    last_keyframe_pose_ = pose;
    last_keyframe_stamp_ = stamp;
    have_keyframe_ = true;
    if (save_keyframes_) {
      std::ostringstream name;
      name << "keyframe_" << std::setw(5) << std::setfill('0') << index << ".pcd";
      pcl::io::savePCDFileBinaryCompressed(
        (std::filesystem::path(output_dir_) / "keyframes" / name.str()).string(), keyframe);
    }
    append_keyframe_metadata(index, stamp, pose, keyframe.size(), *info);
  }

  void append_keyframe_metadata(
    std::size_t index, double stamp, const Eigen::Isometry3d & pose,
    std::size_t points, const sensor_msgs::msg::CameraInfo & info)
  {
    const auto path = std::filesystem::path(output_dir_) / "keyframes.csv";
    const bool create_header = !std::filesystem::exists(path);
    std::ofstream output(path, std::ios::app);
    if (create_header) {
      output << "index,stamp_s,tx,ty,tz,qx,qy,qz,qw,points,global_frame,camera_frame\n";
    }
    const Eigen::Quaterniond quaternion(pose.rotation());
    output << std::setprecision(12) << index << ',' << stamp << ','
           << pose.translation().x() << ',' << pose.translation().y() << ','
           << pose.translation().z() << ',' << quaternion.x() << ',' << quaternion.y() << ','
           << quaternion.z() << ',' << quaternion.w() << ',' << points << ','
           << global_frame_ << ',' << camera_frame_ << '\n';
    const auto calibration_path = std::filesystem::path(output_dir_) / "camera_calibration.yaml";
    if (!std::filesystem::exists(calibration_path)) {
      std::ofstream calibration(calibration_path);
      calibration << std::setprecision(12)
                  << "source_topic: " << camera_info_topic_ << "\n"
                  << "stamp_s: " << stamp << "\n"
                  << "frame_id: " << info.header.frame_id << "\n"
                  << "width: " << info.width << "\nheight: " << info.height << "\n"
                  << "distortion_model: " << info.distortion_model << "\n"
                  << "K: [";
      for (std::size_t i = 0; i < info.k.size(); ++i) {
        calibration << (i == 0 ? "" : ", ") << info.k[i];
      }
      calibration << "]\nD: [";
      for (std::size_t i = 0; i < info.d.size(); ++i) {
        calibration << (i == 0 ? "" : ", ") << info.d[i];
      }
      calibration << "]\n";
    }
  }

  void save_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_) {
      response->success = false;
      response->message = "exporter disabled";
      return;
    }
    if (accumulated_.empty()) {
      response->success = false;
      response->message = "no accepted RGB-D keyframes";
      return;
    }
    std::filesystem::create_directories(output_dir_);
    CloudT::Ptr input(new CloudT(accumulated_));
    CloudT::Ptr output(new CloudT);
    pcl::VoxelGrid<PointT> filter;
    filter.setInputCloud(input);
    filter.setLeafSize(map_voxel_leaf_m_, map_voxel_leaf_m_, map_voxel_leaf_m_);
    filter.filter(*output);
    const auto path = std::filesystem::path(output_dir_) / "visual_map.pcd";
    const int status = pcl::io::savePCDFileBinaryCompressed(path.string(), *output);
    std::ofstream metadata(std::filesystem::path(output_dir_) / "visual_map_metadata.yaml");
    metadata << "map_file: visual_map.pcd\n"
             << "global_frame: " << global_frame_ << "\n"
             << "camera_frame: " << camera_frame_ << "\n"
             << "rgb_topic: " << rgb_topic_ << "\n"
             << "depth_topic: " << depth_topic_ << "\n"
             << "camera_info_topic: " << camera_info_topic_ << "\n"
             << "keyframes: " << keyframe_count_ << "\n"
             << "points_before_voxel: " << accumulated_.size() << "\n"
             << "points_after_voxel: " << output->size() << "\n"
             << "voxel_leaf_m: " << map_voxel_leaf_m_ << "\n"
             << "missing_tf_frames: " << missing_transform_ << "\n"
             << "missing_calibration_frames: " << missing_calibration_ << "\n";
    response->success = status == 0;
    response->message = response->success ? path.string() : "PCD write failed";
  }

  bool enabled_{false};
  std::string rgb_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string global_frame_;
  std::string camera_frame_;
  std::string output_dir_;
  double min_keyframe_translation_m_{0.20};
  double min_keyframe_rotation_deg_{7.5};
  double min_keyframe_period_s_{0.75};
  double depth_min_m_{0.30};
  double depth_max_m_{6.0};
  int pixel_stride_{3};
  int max_points_per_keyframe_{45000};
  double map_voxel_leaf_m_{0.06};
  bool save_keyframes_{true};

  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<ExactPolicy>> synchronizer_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex mutex_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  CloudT accumulated_;
  bool have_keyframe_{false};
  Eigen::Isometry3d last_keyframe_pose_{Eigen::Isometry3d::Identity()};
  double last_keyframe_stamp_{0.0};
  std::size_t keyframe_count_{0};
  std::size_t missing_transform_{0};
  std::size_t missing_calibration_{0};
};

}  // namespace hybridfusion_map_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hybridfusion_map_fusion::RgbdMapExporter>());
  rclcpp::shutdown();
  return 0;
}
