#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace
{

struct PointXYZI
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float intensity{0.0f};
};

struct VoxelKey
{
  int32_t x{0};
  int32_t y{0};
  int32_t z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const uint64_t x = static_cast<uint32_t>(key.x);
    const uint64_t y = static_cast<uint32_t>(key.y);
    const uint64_t z = static_cast<uint32_t>(key.z);
    return static_cast<std::size_t>((x * 73856093ull) ^ (y * 19349663ull) ^ (z * 83492791ull));
  }
};

struct FieldInfo
{
  int offset{-1};
  uint8_t datatype{0};
};

struct Accumulator
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double intensity{0.0};
  uint32_t count{0};
};

struct MapVoxel
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float intensity{0.0f};
  uint32_t count{0};
  uint64_t last_update{0};
};

VoxelKey makeKey(const PointXYZI & point, const double voxel)
{
  return VoxelKey{
    static_cast<int32_t>(std::floor(point.x / voxel)),
    static_cast<int32_t>(std::floor(point.y / voxel)),
    static_cast<int32_t>(std::floor(point.z / voxel))};
}

FieldInfo findField(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name) {
      return FieldInfo{static_cast<int>(field.offset), field.datatype};
    }
  }
  return FieldInfo{};
}

float readFieldAsFloat(const uint8_t * point, const FieldInfo & field)
{
  if (field.offset < 0) {
    return 0.0f;
  }

  const uint8_t * src = point + field.offset;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::INT8:
      return static_cast<float>(*reinterpret_cast<const int8_t *>(src));
    case sensor_msgs::msg::PointField::UINT8:
      return static_cast<float>(*reinterpret_cast<const uint8_t *>(src));
    case sensor_msgs::msg::PointField::INT16:
      return static_cast<float>(*reinterpret_cast<const int16_t *>(src));
    case sensor_msgs::msg::PointField::UINT16:
      return static_cast<float>(*reinterpret_cast<const uint16_t *>(src));
    case sensor_msgs::msg::PointField::INT32:
      return static_cast<float>(*reinterpret_cast<const int32_t *>(src));
    case sensor_msgs::msg::PointField::UINT32:
      return static_cast<float>(*reinterpret_cast<const uint32_t *>(src));
    case sensor_msgs::msg::PointField::FLOAT32:
      return *reinterpret_cast<const float *>(src);
    case sensor_msgs::msg::PointField::FLOAT64:
      return static_cast<float>(*reinterpret_cast<const double *>(src));
    default:
      return 0.0f;
  }
}

void setField(sensor_msgs::msg::PointField & field, const std::string & name, const uint32_t offset)
{
  field.name = name;
  field.offset = offset;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
}

}  // namespace

class FastlioCloudMapper : public rclcpp::Node
{
public:
  FastlioCloudMapper()
  : Node("fastlio_cloud_mapper")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cloud_registered");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/cloud_registered_filtered");
    reliable_scan_topic_ = declare_parameter<std::string>("reliable_scan_topic", "/cloud_registered_reliable");
    map_topic_ = declare_parameter<std::string>("map_topic", "/fastlio_denoised_map");
    output_frame_ = declare_parameter<std::string>("output_frame", "camera_init");
    restamp_ = declare_parameter<bool>("restamp", true);

    min_range_ = declare_parameter<double>("min_range", 0.45);
    max_range_ = declare_parameter<double>("max_range", 35.0);
    z_min_ = declare_parameter<double>("z_min", -3.0);
    z_max_ = declare_parameter<double>("z_max", 5.0);

    scan_voxel_ = declare_parameter<double>("scan_voxel", 0.08);
    map_voxel_ = declare_parameter<double>("map_voxel", 0.12);
    radius_filter_ = declare_parameter<double>("radius_filter", 0.20);
    radius_min_neighbors_ = declare_parameter<int>("radius_min_neighbors", 2);
    min_map_hits_ = declare_parameter<int>("min_map_hits", 2);
    max_scan_points_ = declare_parameter<int>("max_scan_points", 45000);
    max_map_voxels_ = declare_parameter<int>("max_map_voxels", 160000);
    map_window_frames_ = declare_parameter<int>("map_window_frames", 90);
    map_publish_every_ = declare_parameter<int>("map_publish_every", 2);

    enable_quality_gate_ = declare_parameter<bool>("enable_quality_gate", true);
    require_odom_for_map_ = declare_parameter<bool>("require_odom_for_map", true);
    max_odom_speed_mps_ = declare_parameter<double>("max_odom_speed_mps", 2.5);
    max_odom_jump_m_ = declare_parameter<double>("max_odom_jump_m", 0.55);
    max_z_jump_m_ = declare_parameter<double>("max_z_jump_m", 0.35);
    min_scan_map_overlap_ = declare_parameter<double>("min_scan_map_overlap", 0.035);
    min_overlap_map_voxels_ = declare_parameter<int>("min_overlap_map_voxels", 800);
    overlap_warmup_frames_ = declare_parameter<int>("overlap_warmup_frames", 12);
    overlap_neighbor_voxels_ = declare_parameter<int>("overlap_neighbor_voxels", 2);
    overlap_sample_stride_ = declare_parameter<int>("overlap_sample_stride", 4);

    scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(scan_topic_, rclcpp::QoS(2));
    reliable_scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(reliable_scan_topic_, rclcpp::QoS(2));
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(map_topic_, rclcpp::QoS(1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&FastlioCloudMapper::odomCallback, this, std::placeholders::_1));

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::QoS(5),
      std::bind(&FastlioCloudMapper::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "FAST-LIO cloud mapper: %s -> scan %s, map %s, scan_voxel=%.3f, map_voxel=%.3f, radius=%.3f/%d",
      input_topic_.c_str(), scan_topic_.c_str(), map_topic_.c_str(), scan_voxel_, map_voxel_,
      radius_filter_, radius_min_neighbors_);
  }

private:
  std::vector<PointXYZI> extractPoints(const sensor_msgs::msg::PointCloud2 & msg)
  {
    const FieldInfo x_field = findField(msg, "x");
    const FieldInfo y_field = findField(msg, "y");
    const FieldInfo z_field = findField(msg, "z");
    const FieldInfo intensity_field = findField(msg, "intensity");
    if (x_field.offset < 0 || y_field.offset < 0 || z_field.offset < 0 || msg.point_step == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Input cloud has no x/y/z fields.");
      return {};
    }

    const uint64_t declared_points = static_cast<uint64_t>(msg.width) * msg.height;
    const uint64_t available_points = msg.data.size() / msg.point_step;
    const uint64_t point_count = std::min(declared_points, available_points);

    std::vector<PointXYZI> points;
    points.reserve(static_cast<std::size_t>(std::min<uint64_t>(point_count, 80000)));

    const double min_range_sq = min_range_ > 0.0 ? min_range_ * min_range_ : 0.0;
    const double max_range_sq = max_range_ > 0.0 ? max_range_ * max_range_ :
      std::numeric_limits<double>::infinity();

    for (uint64_t i = 0; i < point_count; ++i) {
      const uint8_t * raw = msg.data.data() + i * msg.point_step;
      PointXYZI point;
      point.x = readFieldAsFloat(raw, x_field);
      point.y = readFieldAsFloat(raw, y_field);
      point.z = readFieldAsFloat(raw, z_field);
      point.intensity = readFieldAsFloat(raw, intensity_field);

      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (point.z < z_min_ || point.z > z_max_) {
        continue;
      }
      const double range_sq =
        static_cast<double>(point.x) * point.x + static_cast<double>(point.y) * point.y +
        static_cast<double>(point.z) * point.z;
      if (range_sq < min_range_sq || range_sq > max_range_sq) {
        continue;
      }
      points.push_back(point);
    }

    return points;
  }

  std::vector<PointXYZI> voxelDownsample(
    const std::vector<PointXYZI> & points, const double voxel, const int max_points) const
  {
    if (points.empty() || voxel <= 0.0) {
      return points;
    }

    std::unordered_map<VoxelKey, Accumulator, VoxelKeyHash> grid;
    grid.reserve(points.size());

    for (const auto & point : points) {
      auto & acc = grid[makeKey(point, voxel)];
      acc.x += point.x;
      acc.y += point.y;
      acc.z += point.z;
      acc.intensity += point.intensity;
      ++acc.count;
    }

    std::vector<PointXYZI> out;
    out.reserve(grid.size());
    for (const auto & item : grid) {
      const auto & acc = item.second;
      if (acc.count == 0) {
        continue;
      }
      const double inv = 1.0 / static_cast<double>(acc.count);
      out.push_back(PointXYZI{
        static_cast<float>(acc.x * inv),
        static_cast<float>(acc.y * inv),
        static_cast<float>(acc.z * inv),
        static_cast<float>(acc.intensity * inv)});
    }

    if (max_points > 0 && static_cast<int>(out.size()) > max_points) {
      const std::size_t stride = static_cast<std::size_t>(
        std::ceil(static_cast<double>(out.size()) / static_cast<double>(max_points)));
      std::vector<PointXYZI> capped;
      capped.reserve(static_cast<std::size_t>(max_points));
      for (std::size_t i = 0; i < out.size() && capped.size() < static_cast<std::size_t>(max_points);
        i += stride)
      {
        capped.push_back(out[i]);
      }
      return capped;
    }

    return out;
  }

  std::vector<PointXYZI> radiusOutlierFilter(const std::vector<PointXYZI> & points) const
  {
    if (points.empty() || radius_filter_ <= 0.0 || radius_min_neighbors_ <= 0) {
      return points;
    }

    std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash> grid;
    grid.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      grid[makeKey(points[i], radius_filter_)].push_back(i);
    }

    const double radius_sq = radius_filter_ * radius_filter_;
    std::vector<PointXYZI> out;
    out.reserve(points.size());

    for (std::size_t i = 0; i < points.size(); ++i) {
      const VoxelKey base = makeKey(points[i], radius_filter_);
      int neighbors = 0;

      for (int dx = -1; dx <= 1 && neighbors < radius_min_neighbors_; ++dx) {
        for (int dy = -1; dy <= 1 && neighbors < radius_min_neighbors_; ++dy) {
          for (int dz = -1; dz <= 1 && neighbors < radius_min_neighbors_; ++dz) {
            const VoxelKey key{base.x + dx, base.y + dy, base.z + dz};
            const auto found = grid.find(key);
            if (found == grid.end()) {
              continue;
            }
            for (const auto other_index : found->second) {
              if (other_index == i) {
                continue;
              }
              const auto & other = points[other_index];
              const double ddx = static_cast<double>(points[i].x) - other.x;
              const double ddy = static_cast<double>(points[i].y) - other.y;
              const double ddz = static_cast<double>(points[i].z) - other.z;
              if (ddx * ddx + ddy * ddy + ddz * ddz <= radius_sq) {
                ++neighbors;
                if (neighbors >= radius_min_neighbors_) {
                  break;
                }
              }
            }
          }
        }
      }

      if (neighbors >= radius_min_neighbors_) {
        out.push_back(points[i]);
      }
    }

    return out;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    odom_z_ = msg->pose.pose.position.z;
    odom_stamp_ = rclcpp::Time(msg->header.stamp);
    if (odom_stamp_.nanoseconds() == 0) {
      odom_stamp_ = now();
    }
    have_odom_ = true;
  }

  bool hasMapNeighbor(const PointXYZI & point, const int radius) const
  {
    if (map_.empty()) {
      return false;
    }
    const VoxelKey base = makeKey(point, map_voxel_);
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dz = -radius; dz <= radius; ++dz) {
          const VoxelKey key{base.x + dx, base.y + dy, base.z + dz};
          if (map_.find(key) != map_.end()) {
            return true;
          }
        }
      }
    }
    return false;
  }

  double scanMapOverlap(const std::vector<PointXYZI> & points) const
  {
    if (points.empty() || map_.empty()) {
      return 1.0;
    }
    const int stride = std::max(1, overlap_sample_stride_);
    const int radius = std::max(0, overlap_neighbor_voxels_);
    int sampled = 0;
    int matched = 0;
    for (std::size_t i = 0; i < points.size(); i += static_cast<std::size_t>(stride)) {
      ++sampled;
      if (hasMapNeighbor(points[i], radius)) {
        ++matched;
      }
    }
    if (sampled == 0) {
      return 1.0;
    }
    return static_cast<double>(matched) / static_cast<double>(sampled);
  }

  bool isFrameReliable(const std::vector<PointXYZI> & points, std::string & reason, double & overlap) const
  {
    reason = "ok";
    overlap = 1.0;
    if (!enable_quality_gate_) {
      return true;
    }
    if (points.empty()) {
      reason = "empty_scan";
      return false;
    }
    if (require_odom_for_map_ && !have_odom_) {
      reason = "no_odom";
      return false;
    }
    if (have_odom_ && have_last_accepted_odom_) {
      double dt = (odom_stamp_ - last_accepted_odom_stamp_).seconds();
      if (!std::isfinite(dt) || dt <= 0.001) {
        dt = 0.1;
      }
      const double dx = odom_x_ - last_accepted_x_;
      const double dy = odom_y_ - last_accepted_y_;
      const double dz = odom_z_ - last_accepted_z_;
      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      const double speed = dist / dt;
      if (max_odom_speed_mps_ > 0.0 && speed > max_odom_speed_mps_) {
        reason = "odom_speed";
        return false;
      }
      if (max_odom_jump_m_ > 0.0 && dt < 0.40 && dist > max_odom_jump_m_) {
        reason = "odom_jump";
        return false;
      }
      if (max_z_jump_m_ > 0.0 && std::abs(dz) > max_z_jump_m_) {
        reason = "z_jump";
        return false;
      }
    }
    if (map_.size() >= static_cast<std::size_t>(std::max(0, min_overlap_map_voxels_)) &&
      accepted_frames_ >= static_cast<uint64_t>(std::max(0, overlap_warmup_frames_)))
    {
      overlap = scanMapOverlap(points);
      if (overlap < min_scan_map_overlap_) {
        reason = "low_overlap";
        return false;
      }
    }
    return true;
  }

  void rememberAcceptedOdom()
  {
    if (!have_odom_) {
      return;
    }
    last_accepted_x_ = odom_x_;
    last_accepted_y_ = odom_y_;
    last_accepted_z_ = odom_z_;
    last_accepted_odom_stamp_ = odom_stamp_;
    have_last_accepted_odom_ = true;
  }

  void appendMap(const std::vector<PointXYZI> & points)
  {
    for (const auto & point : points) {
      auto & voxel = map_[makeKey(point, map_voxel_)];
      const float count = static_cast<float>(std::min<uint32_t>(voxel.count + 1, 65535));
      if (voxel.count == 0) {
        voxel.x = point.x;
        voxel.y = point.y;
        voxel.z = point.z;
        voxel.intensity = point.intensity;
      } else {
        voxel.x += (point.x - voxel.x) / count;
        voxel.y += (point.y - voxel.y) / count;
        voxel.z += (point.z - voxel.z) / count;
        voxel.intensity += (point.intensity - voxel.intensity) / count;
      }
      voxel.count = static_cast<uint32_t>(count);
      voxel.last_update = accepted_frames_;
    }

    pruneMap();
  }

  void pruneMap()
  {
    if (map_window_frames_ > 0) {
      const uint64_t window = static_cast<uint64_t>(map_window_frames_);
      for (auto it = map_.begin(); it != map_.end(); ) {
        if (accepted_frames_ > it->second.last_update + window) {
          it = map_.erase(it);
        } else {
          ++it;
        }
      }
    }

    const std::size_t max_voxels = static_cast<std::size_t>(std::max(1000, max_map_voxels_));
    if (map_.size() <= max_voxels) {
      return;
    }

    for (auto it = map_.begin(); it != map_.end() && map_.size() > max_voxels; ) {
      if (static_cast<int>(it->second.count) < min_map_hits_) {
        it = map_.erase(it);
      } else {
        ++it;
      }
    }

    while (map_.size() > max_voxels && !map_.empty()) {
      map_.erase(map_.begin());
    }
  }
  void publishCloud(
    const sensor_msgs::msg::PointCloud2 & input_header_source,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const std::vector<PointXYZI> & points)
  {
    sensor_msgs::msg::PointCloud2 out;
    out.header = input_header_source.header;
    if (!output_frame_.empty()) {
      out.header.frame_id = output_frame_;
    }
    if (restamp_) {
      out.header.stamp = now();
    }

    out.height = 1;
    out.width = static_cast<uint32_t>(points.size());
    out.is_bigendian = false;
    out.is_dense = false;
    out.fields.resize(4);
    setField(out.fields[0], "x", 0);
    setField(out.fields[1], "y", 4);
    setField(out.fields[2], "z", 8);
    setField(out.fields[3], "intensity", 12);
    out.point_step = 16;
    out.row_step = out.point_step * out.width;
    out.data.resize(static_cast<std::size_t>(out.row_step));

    uint8_t * dst = out.data.data();
    for (std::size_t i = 0; i < points.size(); ++i) {
      const std::array<float, 4> values{points[i].x, points[i].y, points[i].z, points[i].intensity};
      std::memcpy(dst + i * out.point_step, values.data(), out.point_step);
    }

    pub->publish(out);
  }

  void publishMap(const sensor_msgs::msg::PointCloud2 & input_header_source)
  {
    if (map_publish_every_ > 1 && frame_index_ % static_cast<uint64_t>(map_publish_every_) != 0) {
      return;
    }

    std::vector<PointXYZI> points;
    points.reserve(map_.size());
    for (const auto & item : map_) {
      const auto & voxel = item.second;
      if (static_cast<int>(voxel.count) < min_map_hits_) {
        continue;
      }
      points.push_back(PointXYZI{voxel.x, voxel.y, voxel.z, voxel.intensity});
    }
    publishCloud(input_header_source, map_pub_, points);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    ++frame_index_;

    auto points = extractPoints(*msg);
    points = voxelDownsample(points, scan_voxel_, max_scan_points_);
    points = radiusOutlierFilter(points);

    publishCloud(*msg, scan_pub_, points);

    std::string reject_reason;
    double overlap = 1.0;
    const bool reliable = isFrameReliable(points, reject_reason, overlap);
    if (reliable) {
      ++accepted_frames_;
      rememberAcceptedOdom();
      publishCloud(*msg, reliable_scan_pub_, points);
      appendMap(points);
      publishMap(*msg);
    } else {
      ++rejected_frames_;
      publishCloud(*msg, reliable_scan_pub_, {});
      publishMap(*msg);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "rejected unstable scan: reason=%s overlap=%.3f accepted=%lu rejected=%lu",
        reject_reason.c_str(), overlap, accepted_frames_, rejected_frames_);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "filtered scan=%zu map_voxels=%zu accepted=%lu rejected=%lu overlap=%.3f",
      points.size(), map_.size(), accepted_frames_, rejected_frames_, overlap);
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr reliable_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  std::unordered_map<VoxelKey, MapVoxel, VoxelKeyHash> map_;

  std::string input_topic_;
  std::string odom_topic_;
  std::string scan_topic_;
  std::string reliable_scan_topic_;
  std::string map_topic_;
  std::string output_frame_;
  bool restamp_{true};
  double min_range_{0.45};
  double max_range_{35.0};
  double z_min_{-3.0};
  double z_max_{5.0};
  double scan_voxel_{0.08};
  double map_voxel_{0.12};
  double radius_filter_{0.20};
  int radius_min_neighbors_{2};
  int min_map_hits_{2};
  int max_scan_points_{45000};
  int max_map_voxels_{160000};
  int map_window_frames_{90};
  int map_publish_every_{2};
  bool enable_quality_gate_{true};
  bool require_odom_for_map_{true};
  double max_odom_speed_mps_{2.5};
  double max_odom_jump_m_{0.55};
  double max_z_jump_m_{0.35};
  double min_scan_map_overlap_{0.035};
  int min_overlap_map_voxels_{800};
  int overlap_warmup_frames_{12};
  int overlap_neighbor_voxels_{2};
  int overlap_sample_stride_{4};
  bool have_odom_{false};
  bool have_last_accepted_odom_{false};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_accepted_odom_stamp_{0, 0, RCL_ROS_TIME};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_z_{0.0};
  double last_accepted_x_{0.0};
  double last_accepted_y_{0.0};
  double last_accepted_z_{0.0};
  uint64_t frame_index_{0};
  uint64_t accepted_frames_{0};
  uint64_t rejected_frames_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastlioCloudMapper>());
  rclcpp::shutdown();
  return 0;
}
