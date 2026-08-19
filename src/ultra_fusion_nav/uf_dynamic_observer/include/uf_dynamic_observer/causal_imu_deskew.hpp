#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <string>
#include <vector>

namespace uf_dynamic_observer
{

struct CausalPose
{
  std::int64_t stamp_ns{0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct CausalImuSample
{
  std::int64_t stamp_ns{0};
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

struct CausalDeskewConfig
{
  double max_imu_gap_s{0.025};
  double max_prediction_horizon_s{0.20};
  Eigen::Vector3d gravity_world{0.0, 0.0, -9.80665};
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
};

struct CausalDeskewResult
{
  bool valid{false};
  std::string reason;
  std::vector<CausalPose> poses;
  std::int64_t anchor_stamp_ns{0};
  std::int64_t latest_imu_consumed_ns{0};
  double max_observed_imu_gap_s{0.0};
};

// Propagates a state that predates the scan with only IMU samples available by
// each point time. It never accepts a pose anchor newer than the scan start.
// Livox CustomPoint::offset_time is nanoseconds, matching FAST-LIO's conversion
// offset_time / 1e6 -> milliseconds -> / 1000 -> seconds.
class CausalImuDeskew
{
public:
  explicit CausalImuDeskew(CausalDeskewConfig config = {});

  CausalDeskewResult propagate(
    const CausalPose & anchor, const std::vector<CausalImuSample> & imu,
    const std::vector<std::int64_t> & query_stamps_ns) const;

private:
  CausalDeskewConfig config_;
};

}  // namespace uf_dynamic_observer
