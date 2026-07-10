#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace
{

struct FieldInfo
{
  int offset{-1};
  uint8_t datatype{0};
};

struct PointXYZ
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

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

int clampInt(const int value, const int low, const int high)
{
  return std::max(low, std::min(high, value));
}

}  // namespace

class PointCloudOccupancyGrid : public rclcpp::Node
{
public:
  PointCloudOccupancyGrid()
  : Node("pointcloud_occupancy_grid")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cloud_registered_filtered");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    grid_topic_ = declare_parameter<std::string>("grid_topic", "/fastlio_occupancy_grid");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/fastlio_occupancy_cells");
    free_marker_topic_ = declare_parameter<std::string>("free_marker_topic", "/fastlio_occupancy_free_cells");
    occupied_marker_topic_ = declare_parameter<std::string>("occupied_marker_topic", "/fastlio_occupancy_occupied_cells");
    output_frame_ = declare_parameter<std::string>("output_frame", "camera_init");

    resolution_ = declare_parameter<double>("resolution", 0.10);
    width_m_ = declare_parameter<double>("width_m", 30.0);
    height_m_ = declare_parameter<double>("height_m", 30.0);
    origin_x_ = declare_parameter<double>("origin_x", -15.0);
    origin_y_ = declare_parameter<double>("origin_y", -15.0);

    min_range_ = declare_parameter<double>("min_range", 0.45);
    max_range_ = declare_parameter<double>("max_range", 25.0);
    z_min_ = declare_parameter<double>("z_min", -0.30);
    z_max_ = declare_parameter<double>("z_max", 2.50);
    point_stride_ = declare_parameter<int>("point_stride", 1);
    publish_every_n_clouds_ = declare_parameter<int>("publish_every_n_clouds", 1);

    use_odometry_ = declare_parameter<bool>("use_odometry", true);
    raycast_free_space_ = declare_parameter<bool>("raycast_free_space", true);
    occupied_increment_ = declare_parameter<int>("occupied_increment", 12);
    free_decrement_ = declare_parameter<int>("free_decrement", 4);
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 45);
    free_threshold_ = declare_parameter<int>("free_threshold", -12);
    log_odds_decay_per_cloud_ = declare_parameter<int>("log_odds_decay_per_cloud", 1);
    stale_after_clouds_ = declare_parameter<int>("stale_after_clouds", 120);
    occupied_cell_inflation_ = declare_parameter<int>("occupied_cell_inflation", 1);

    if (resolution_ <= 0.0) {
      throw std::runtime_error("resolution must be positive");
    }
    width_cells_ = static_cast<int>(std::ceil(width_m_ / resolution_));
    height_cells_ = static_cast<int>(std::ceil(height_m_ / resolution_));
    if (width_cells_ <= 0 || height_cells_ <= 0) {
      throw std::runtime_error("grid dimensions must be positive");
    }

    const std::size_t cell_count = static_cast<std::size_t>(width_cells_) * height_cells_;
    log_odds_.assign(cell_count, 0);
    observed_.assign(cell_count, false);
    last_update_.assign(cell_count, 0);

    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      grid_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      marker_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    free_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      free_marker_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    occupied_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      occupied_marker_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    if (use_odometry_) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(20),
        std::bind(&PointCloudOccupancyGrid::odomCallback, this, std::placeholders::_1));
    }

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::QoS(5),
      std::bind(&PointCloudOccupancyGrid::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "PointCloud occupancy grid: %s -> grid %s, marker %s, %.2fm x %.2fm @ %.2fm",
      input_topic_.c_str(), grid_topic_.c_str(), marker_topic_.c_str(), width_m_, height_m_, resolution_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    sensor_x_ = msg->pose.pose.position.x;
    sensor_y_ = msg->pose.pose.position.y;
    have_odom_ = true;
  }

  bool worldToCell(const double x, const double y, int & cx, int & cy) const
  {
    cx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    cy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    return cx >= 0 && cy >= 0 && cx < width_cells_ && cy < height_cells_;
  }

  std::size_t index(const int cx, const int cy) const
  {
    return static_cast<std::size_t>(cy) * width_cells_ + cx;
  }

  void markFree(const int cx, const int cy)
  {
    if (cx < 0 || cy < 0 || cx >= width_cells_ || cy >= height_cells_) {
      return;
    }
    const auto i = index(cx, cy);
    observed_[i] = true;
    last_update_[i] = cloud_count_;
    log_odds_[i] = static_cast<int16_t>(clampInt(log_odds_[i] - free_decrement_, -100, 100));
  }

  void markOccupiedCell(const int cx, const int cy)
  {
    if (cx < 0 || cy < 0 || cx >= width_cells_ || cy >= height_cells_) {
      return;
    }
    const auto i = index(cx, cy);
    observed_[i] = true;
    last_update_[i] = cloud_count_;
    log_odds_[i] = static_cast<int16_t>(clampInt(log_odds_[i] + occupied_increment_, -100, 100));
  }

  void markOccupied(const int cx, const int cy)
  {
    const int inflation = std::max(0, occupied_cell_inflation_);
    for (int dy = -inflation; dy <= inflation; ++dy) {
      for (int dx = -inflation; dx <= inflation; ++dx) {
        if (dx * dx + dy * dy <= inflation * inflation) {
          markOccupiedCell(cx + dx, cy + dy);
        }
      }
    }
  }

  void raycastFree(const int sx, const int sy, const int ex, const int ey)
  {
    int x = sx;
    int y = sy;
    const int dx = std::abs(ex - sx);
    const int dy = -std::abs(ey - sy);
    const int step_x = sx < ex ? 1 : -1;
    const int step_y = sy < ey ? 1 : -1;
    int error = dx + dy;

    while (!(x == ex && y == ey)) {
      markFree(x, y);
      const int error2 = 2 * error;
      if (error2 >= dy) {
        error += dy;
        x += step_x;
      }
      if (error2 <= dx) {
        error += dx;
        y += step_y;
      }
    }
  }


  bool isCellFresh(const std::size_t i) const
  {
    if (!observed_[i]) {
      return false;
    }
    if (stale_after_clouds_ <= 0) {
      return true;
    }
    return cloud_count_ <= last_update_[i] + static_cast<uint64_t>(stale_after_clouds_);
  }

  void decayGrid()
  {
    const int decay = std::max(0, log_odds_decay_per_cloud_);
    const bool use_stale_reset = stale_after_clouds_ > 0;
    if (decay == 0 && !use_stale_reset) {
      return;
    }
    for (std::size_t i = 0; i < log_odds_.size(); ++i) {
      if (!observed_[i]) {
        continue;
      }
      if (use_stale_reset && cloud_count_ > last_update_[i] + static_cast<uint64_t>(stale_after_clouds_)) {
        observed_[i] = false;
        log_odds_[i] = 0;
        continue;
      }
      if (decay == 0) {
        continue;
      }
      if (log_odds_[i] > 0) {
        log_odds_[i] = static_cast<int16_t>(std::max<int>(0, log_odds_[i] - decay));
      } else if (log_odds_[i] < 0) {
        log_odds_[i] = static_cast<int16_t>(std::min<int>(0, log_odds_[i] + decay));
      }
      if (log_odds_[i] == 0) {
        observed_[i] = false;
      }
    }
  }

  std::vector<PointXYZ> extractPoints(const sensor_msgs::msg::PointCloud2 & msg)
  {
    const FieldInfo x_field = findField(msg, "x");
    const FieldInfo y_field = findField(msg, "y");
    const FieldInfo z_field = findField(msg, "z");
    if (x_field.offset < 0 || y_field.offset < 0 || z_field.offset < 0 || msg.point_step == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Input cloud has no x/y/z fields.");
      return {};
    }

    const uint64_t declared_points = static_cast<uint64_t>(msg.width) * msg.height;
    const uint64_t available_points = msg.data.size() / msg.point_step;
    const uint64_t point_count = std::min(declared_points, available_points);
    const int stride = std::max(1, point_stride_);

    std::vector<PointXYZ> points;
    points.reserve(static_cast<std::size_t>(std::min<uint64_t>(point_count / stride + 1, 60000)));

    for (uint64_t i = 0; i < point_count; i += static_cast<uint64_t>(stride)) {
      const uint8_t * raw = msg.data.data() + i * msg.point_step;
      PointXYZ point{
        readFieldAsFloat(raw, x_field),
        readFieldAsFloat(raw, y_field),
        readFieldAsFloat(raw, z_field)};

      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (point.z < z_min_ || point.z > z_max_) {
        continue;
      }
      points.push_back(point);
    }
    return points;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    ++cloud_count_;
    decayGrid();
    const bool can_raycast = raycast_free_space_ && (!use_odometry_ || have_odom_);

    int start_x = 0;
    int start_y = 0;
    if (can_raycast) {
      const double sx = use_odometry_ ? sensor_x_ : 0.0;
      const double sy = use_odometry_ ? sensor_y_ : 0.0;
      worldToCell(sx, sy, start_x, start_y);
    }

    const auto points = extractPoints(*msg);
    const double min_range_sq = min_range_ > 0.0 ? min_range_ * min_range_ : 0.0;
    const double max_range_sq = max_range_ > 0.0 ? max_range_ * max_range_ :
      std::numeric_limits<double>::infinity();

    int used_points = 0;
    for (const auto & point : points) {
      const double sx = use_odometry_ && have_odom_ ? sensor_x_ : 0.0;
      const double sy = use_odometry_ && have_odom_ ? sensor_y_ : 0.0;
      const double dx = static_cast<double>(point.x) - sx;
      const double dy = static_cast<double>(point.y) - sy;
      const double range_sq = dx * dx + dy * dy;
      if (range_sq < min_range_sq || range_sq > max_range_sq) {
        continue;
      }

      int end_x = 0;
      int end_y = 0;
      if (!worldToCell(point.x, point.y, end_x, end_y)) {
        continue;
      }
      if (can_raycast) {
        raycastFree(start_x, start_y, end_x, end_y);
      }
      markOccupied(end_x, end_y);
      ++used_points;
    }

    if (publish_every_n_clouds_ <= 1 ||
      cloud_count_ % static_cast<uint64_t>(publish_every_n_clouds_) == 0)
    {
      publishGrid(msg->header.stamp);
      publishMarker(msg->header.stamp);
      publishGridMarkers(msg->header.stamp);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "occupancy grid updated from %d points, odom=%s, clouds=%lu",
      used_points, have_odom_ ? "yes" : "no", cloud_count_);
  }

  void publishGrid(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = output_frame_;
    grid.info.map_load_time = now();
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = static_cast<uint32_t>(width_cells_);
    grid.info.height = static_cast<uint32_t>(height_cells_);
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(log_odds_.size(), -1);

    for (std::size_t i = 0; i < log_odds_.size(); ++i) {
      if (!isCellFresh(i)) {
        grid.data[i] = -1;
      } else if (log_odds_[i] >= occupied_threshold_) {
        grid.data[i] = 100;
      } else if (log_odds_[i] <= free_threshold_) {
        grid.data[i] = 0;
      } else {
        grid.data[i] = static_cast<int8_t>(clampInt(50 + log_odds_[i] / 2, 1, 99));
      }
    }
    grid_pub_->publish(grid);
  }

  void publishMarker(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = output_frame_;
    marker.ns = "fastlio_occupancy_grid";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = resolution_;
    marker.scale.y = resolution_;
    marker.scale.z = 0.035;
    marker.color.a = 0.72;
    marker.color.r = 1.0;
    marker.color.g = 0.28;
    marker.color.b = 0.04;

    marker.points.reserve(log_odds_.size() / 12);
    for (int cy = 0; cy < height_cells_; ++cy) {
      for (int cx = 0; cx < width_cells_; ++cx) {
        const auto i = index(cx, cy);
        if (!isCellFresh(i) || log_odds_[i] < occupied_threshold_) {
          continue;
        }
        geometry_msgs::msg::Point point;
        point.x = origin_x_ + (static_cast<double>(cx) + 0.5) * resolution_;
        point.y = origin_y_ + (static_cast<double>(cy) + 0.5) * resolution_;
        point.z = 0.02;
        marker.points.push_back(point);
      }
    }
    marker_pub_->publish(marker);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  visualization_msgs::msg::Marker makeGridCellMarker(
    const rclcpp::Time & stamp, const std::string & ns, const int id, const bool occupied,
    const float r, const float g, const float b, const float alpha, const double z) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = output_frame_;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = resolution_;
    marker.scale.y = resolution_;
    marker.scale.z = 0.018;
    marker.color.a = alpha;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;

    marker.points.reserve(log_odds_.size() / (occupied ? 12 : 3));
    for (int cy = 0; cy < height_cells_; ++cy) {
      for (int cx = 0; cx < width_cells_; ++cx) {
        const auto i = index(cx, cy);
        if (!isCellFresh(i)) {
          continue;
        }
        const bool include = occupied ?
          log_odds_[i] >= occupied_threshold_ : log_odds_[i] <= free_threshold_;
        if (!include) {
          continue;
        }
        geometry_msgs::msg::Point point;
        point.x = origin_x_ + (static_cast<double>(cx) + 0.5) * resolution_;
        point.y = origin_y_ + (static_cast<double>(cy) + 0.5) * resolution_;
        point.z = z;
        marker.points.push_back(point);
      }
    }
    return marker;
  }

  void publishGridMarkers(const rclcpp::Time & stamp) const
  {
    free_marker_pub_->publish(makeGridCellMarker(
      stamp, "fastlio_occupancy_free", 0, false, 0.96f, 0.96f, 0.96f, 0.90f, 0.006));
    occupied_marker_pub_->publish(makeGridCellMarker(
      stamp, "fastlio_occupancy_occupied", 1, true, 0.02f, 0.02f, 0.02f, 1.0f, 0.026));
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr free_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occupied_marker_pub_;

  std::vector<int16_t> log_odds_;
  std::vector<uint64_t> last_update_;
  std::vector<bool> observed_;

  std::string input_topic_;
  std::string odom_topic_;
  std::string grid_topic_;
  std::string marker_topic_;
  std::string free_marker_topic_;
  std::string occupied_marker_topic_;
  std::string output_frame_;
  double resolution_{0.10};
  double width_m_{30.0};
  double height_m_{30.0};
  double origin_x_{-15.0};
  double origin_y_{-15.0};
  double min_range_{0.45};
  double max_range_{25.0};
  double z_min_{-0.30};
  double z_max_{2.50};
  int point_stride_{1};
  int publish_every_n_clouds_{1};
  bool use_odometry_{true};
  bool raycast_free_space_{true};
  int occupied_increment_{12};
  int free_decrement_{4};
  int occupied_threshold_{45};
  int free_threshold_{-12};
  int log_odds_decay_per_cloud_{1};
  int stale_after_clouds_{120};
  int occupied_cell_inflation_{1};
  int width_cells_{0};
  int height_cells_{0};
  double sensor_x_{0.0};
  double sensor_y_{0.0};
  bool have_odom_{false};
  uint64_t cloud_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudOccupancyGrid>());
  rclcpp::shutdown();
  return 0;
}
