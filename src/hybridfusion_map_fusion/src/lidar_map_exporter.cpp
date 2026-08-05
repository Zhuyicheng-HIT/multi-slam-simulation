#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace hybridfusion_map_fusion
{

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

class LidarMapExporter final : public rclcpp::Node
{
public:
  LidarMapExporter()
  : Node("hybridfusion_lidar_map_exporter")
  {
    enabled_ = declare_parameter("enabled", false);
    map_topic_ = declare_parameter("map_topic", "/fastlio_denoised_map");
    output_dir_ = expand_home(declare_parameter(
      "output_dir", "~/.ros/hybridfusion_export/lidar"));
    voxel_leaf_m_ = declare_parameter("voxel_leaf_m", 0.10);
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "~/save", std::bind(
        &LidarMapExporter::save_service, this, std::placeholders::_1, std::placeholders::_2));
    if (!enabled_) {
      RCLCPP_INFO(get_logger(), "LiDAR map export is disabled (default); no map subscription created");
      return;
    }
    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      map_topic_, rclcpp::QoS(1).reliable(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        CloudT cloud;
        try {
          pcl::fromROSMsg(*message, cloud);
        } catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(), "Ignoring undecodable LiDAR map: %s", error.what());
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = std::move(cloud);
        frame_id_ = message->header.frame_id;
        stamp_ = message->header.stamp;
        ++received_maps_;
      });
    RCLCPP_INFO(
      get_logger(), "Enabled FAST-LIO map export: %s -> %s",
      map_topic_.c_str(), output_dir_.c_str());
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
    if (latest_.empty()) {
      response->success = false;
      response->message = "no FAST-LIO map received";
      return;
    }
    std::filesystem::create_directories(output_dir_);
    CloudT::Ptr input(new CloudT(latest_));
    CloudT::Ptr output(new CloudT);
    pcl::VoxelGrid<PointT> filter;
    filter.setInputCloud(input);
    filter.setLeafSize(voxel_leaf_m_, voxel_leaf_m_, voxel_leaf_m_);
    filter.filter(*output);
    const auto map_path = std::filesystem::path(output_dir_) / "lidar_map.pcd";
    const int status = pcl::io::savePCDFileBinaryCompressed(map_path.string(), *output);
    std::ofstream metadata(std::filesystem::path(output_dir_) / "lidar_map_metadata.yaml");
    metadata << "map_file: lidar_map.pcd\n"
             << "source_topic: " << map_topic_ << "\n"
             << "frame_id: " << frame_id_ << "\n"
             << "stamp_sec: " << stamp_.sec << "\n"
             << "stamp_nanosec: " << stamp_.nanosec << "\n"
             << "received_map_messages: " << received_maps_ << "\n"
             << "points_before_voxel: " << latest_.size() << "\n"
             << "points_after_voxel: " << output->size() << "\n"
             << "voxel_leaf_m: " << voxel_leaf_m_ << "\n"
             << "source_map_modified: false\n";
    response->success = status == 0;
    response->message = response->success ? map_path.string() : "PCD write failed";
  }

  bool enabled_{false};
  std::string map_topic_;
  std::string output_dir_;
  double voxel_leaf_m_{0.10};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  std::mutex mutex_;
  CloudT latest_;
  std::string frame_id_;
  builtin_interfaces::msg::Time stamp_;
  std::size_t received_maps_{0};
};

}  // namespace hybridfusion_map_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hybridfusion_map_fusion::LidarMapExporter>());
  rclcpp::shutdown();
  return 0;
}
