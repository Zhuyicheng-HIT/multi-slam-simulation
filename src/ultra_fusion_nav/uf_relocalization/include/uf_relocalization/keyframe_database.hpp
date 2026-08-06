#pragma once

#include "uf_relocalization/registration_core.hpp"

#include <Eigen/Geometry>

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

namespace uf_relocalization
{

struct KeyframeQuality
{
  double map_quality{0.0};
  double feature_repeatability{0.0};
  double dynamic_ratio{1.0};
  double lidar_degradation{1.0};
  bool scheduler_lidar_enabled{false};
};

struct KeyframeDatabaseConfig
{
  double minimum_feature_repeatability{0.70};
  double maximum_dynamic_ratio{0.15};
  double maximum_lidar_degradation{0.75};
  double minimum_translation_spacing_m{1.0};
  double minimum_rotation_spacing_rad{0.26};
  std::size_t maximum_keyframes{500};
};

struct AdmissionResult
{
  bool accepted{false};
  std::size_t keyframe_id{0};
  std::string reason;
};

struct CandidateMatch
{
  std::size_t keyframe_id{0};
  double descriptor_distance{0.0};
};

struct StaticKeyframe
{
  std::size_t id{0};
  double stamp_s{0.0};
  Eigen::Isometry3d world_from_sensor{Eigen::Isometry3d::Identity()};
  Cloud::ConstPtr cloud;
  std::vector<float> normalized_descriptor;
  KeyframeQuality quality;
};

class StaticKeyframeDatabase
{
public:
  explicit StaticKeyframeDatabase(
    const KeyframeDatabaseConfig & config = KeyframeDatabaseConfig{});

  AdmissionResult try_insert(
    double stamp_s,
    const Eigen::Isometry3d & world_from_sensor,
    const Cloud::ConstPtr & cloud,
    const std::vector<float> & descriptor,
    const KeyframeQuality & quality);

  AdmissionResult quality_admission(const KeyframeQuality & quality) const;

  std::vector<CandidateMatch> query(
    const std::vector<float> & descriptor,
    std::size_t maximum_candidates,
    std::size_t exclude_recent_keyframes = 0) const;

  const StaticKeyframe * find(std::size_t keyframe_id) const;
  const std::deque<StaticKeyframe> & keyframes() const;
  std::size_t descriptor_dimension() const;

private:
  KeyframeDatabaseConfig config_;
  std::deque<StaticKeyframe> keyframes_;
  std::size_t descriptor_dimension_{0};
  std::size_t next_keyframe_id_{0};
};

}  // namespace uf_relocalization
