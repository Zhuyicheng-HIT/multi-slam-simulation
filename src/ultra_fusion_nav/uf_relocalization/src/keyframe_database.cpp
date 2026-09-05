#include "uf_relocalization/keyframe_database.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <openssl/sha.h>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <stdexcept>
#include <utility>

namespace uf_relocalization
{
namespace
{

constexpr std::size_t kInvalidKeyframeId = std::numeric_limits<std::size_t>::max();

AdmissionResult reject(const char * reason)
{
  return AdmissionResult{false, kInvalidKeyframeId, reason};
}

bool normalize_descriptor(
  const std::vector<float> & descriptor,
  std::vector<float> & normalized)
{
  if (descriptor.empty()) {
    return false;
  }
  double squared_norm = 0.0;
  for (const float value : descriptor) {
    if (!std::isfinite(value)) {
      return false;
    }
    squared_norm += static_cast<double>(value) * static_cast<double>(value);
  }
  if (!std::isfinite(squared_norm) || squared_norm <= 1.0e-12) {
    return false;
  }
  const double inverse_norm = 1.0 / std::sqrt(squared_norm);
  normalized.resize(descriptor.size());
  std::transform(
    descriptor.begin(), descriptor.end(), normalized.begin(),
    [inverse_norm](const float value) {
      return static_cast<float>(static_cast<double>(value) * inverse_norm);
    });
  return true;
}

bool finite_quality(const KeyframeQuality & quality)
{
  return std::isfinite(quality.map_quality) &&
         std::isfinite(quality.feature_repeatability) &&
         std::isfinite(quality.dynamic_ratio) &&
         std::isfinite(quality.lidar_degradation);
}

std::string file_sha256(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) return {};
  SHA256_CTX context;
  SHA256_Init(&context);
  std::array<char, 8192> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = input.gcount();
    if (count > 0) SHA256_Update(&context, buffer.data(), static_cast<std::size_t>(count));
  }
  if (!input.eof()) return {};
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256_Final(digest, &context);
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const auto byte : digest) output << std::setw(2) << static_cast<unsigned int>(byte);
  return output.str();
}

}  // namespace

StaticKeyframeDatabase::StaticKeyframeDatabase(const KeyframeDatabaseConfig & config)
: config_(config)
{
  const bool thresholds_valid =
    config_.minimum_feature_repeatability >= 0.0 &&
    config_.minimum_feature_repeatability <= 1.0 &&
    config_.maximum_dynamic_ratio >= 0.0 && config_.maximum_dynamic_ratio <= 1.0 &&
    config_.maximum_lidar_degradation >= 0.0 &&
    config_.maximum_lidar_degradation <= 1.0;
  if (!thresholds_valid || config_.minimum_translation_spacing_m < 0.0 ||
    config_.minimum_rotation_spacing_rad < 0.0 || config_.maximum_keyframes == 0)
  {
    throw std::invalid_argument("invalid keyframe database configuration");
  }
}

AdmissionResult StaticKeyframeDatabase::quality_admission(
  const KeyframeQuality & quality) const
{
  if (!quality.scheduler_lidar_enabled) {
    return reject("scheduler_lidar_disabled");
  }
  if (!finite_quality(quality)) {
    return reject("non_finite_quality");
  }
  // map_quality is a diagnostic derived from adjacent-frame overlap,
  // repeatability, and dynamic ratio. Reusing it as a hard threshold would
  // count the same evidence twice and reject healthy moving keyframes.
  if (quality.feature_repeatability < config_.minimum_feature_repeatability) {
    return reject("low_feature_repeatability");
  }
  if (quality.dynamic_ratio > config_.maximum_dynamic_ratio) {
    return reject("high_dynamic_ratio");
  }
  if (quality.lidar_degradation > config_.maximum_lidar_degradation) {
    return reject("high_lidar_degradation");
  }
  return AdmissionResult{true, kInvalidKeyframeId, "quality_accepted"};
}

AdmissionResult StaticKeyframeDatabase::try_insert(
  const double stamp_s,
  const Eigen::Isometry3d & world_from_sensor,
  const Cloud::ConstPtr & cloud,
  const std::vector<float> & descriptor,
  const KeyframeQuality & quality)
{
  const auto quality_result = quality_admission(quality);
  if (!quality_result.accepted) {
    return quality_result;
  }
  if (!std::isfinite(stamp_s) || !world_from_sensor.matrix().allFinite()) {
    return reject("invalid_pose_or_stamp");
  }
  if (!cloud || cloud->empty()) {
    return reject("empty_cloud");
  }
  std::vector<float> normalized_descriptor;
  if (!normalize_descriptor(descriptor, normalized_descriptor)) {
    return reject("invalid_descriptor");
  }
  if (descriptor_dimension_ != 0 && descriptor.size() != descriptor_dimension_) {
    return reject("descriptor_dimension_mismatch");
  }

  if (!keyframes_.empty()) {
    const auto & previous = keyframes_.back();
    const double translation =
      (world_from_sensor.translation() - previous.world_from_sensor.translation()).norm();
    const Eigen::Matrix3d rotation_delta =
      previous.world_from_sensor.rotation().transpose() * world_from_sensor.rotation();
    const double rotation = Eigen::AngleAxisd(rotation_delta).angle();
    if (translation < config_.minimum_translation_spacing_m &&
      rotation < config_.minimum_rotation_spacing_rad)
    {
      return reject("insufficient_pose_spacing");
    }
  }

  if (descriptor_dimension_ == 0) {
    descriptor_dimension_ = descriptor.size();
  }
  if (keyframes_.size() >= config_.maximum_keyframes) {
    keyframes_.pop_front();
  }
  StaticKeyframe keyframe;
  keyframe.id = next_keyframe_id_++;
  keyframe.stamp_s = stamp_s;
  keyframe.world_from_sensor = world_from_sensor;
  keyframe.cloud = std::make_shared<Cloud>(*cloud);
  keyframe.normalized_descriptor = std::move(normalized_descriptor);
  keyframe.quality = quality;
  const std::size_t id = keyframe.id;
  keyframes_.push_back(std::move(keyframe));
  return AdmissionResult{true, id, "accepted"};
}

std::vector<CandidateMatch> StaticKeyframeDatabase::query(
  const std::vector<float> & descriptor,
  const std::size_t maximum_candidates,
  const std::size_t exclude_recent_keyframes) const
{
  if (maximum_candidates == 0 || keyframes_.empty()) {
    return {};
  }
  std::vector<float> normalized;
  if (!normalize_descriptor(descriptor, normalized)) {
    throw std::invalid_argument("query descriptor must be finite and non-zero");
  }
  if (normalized.size() != descriptor_dimension_) {
    throw std::invalid_argument("query descriptor dimension mismatch");
  }
  const std::size_t eligible = keyframes_.size() -
    std::min(exclude_recent_keyframes, keyframes_.size());
  std::vector<CandidateMatch> matches;
  matches.reserve(eligible);
  for (std::size_t index = 0; index < eligible; ++index) {
    const auto & keyframe = keyframes_[index];
    double similarity = 0.0;
    for (std::size_t element = 0; element < normalized.size(); ++element) {
      similarity += static_cast<double>(normalized[element]) *
        static_cast<double>(keyframe.normalized_descriptor[element]);
    }
    matches.push_back(CandidateMatch{
      keyframe.id,
      std::max(0.0, 1.0 - std::clamp(similarity, -1.0, 1.0))});
  }
  std::sort(
    matches.begin(), matches.end(),
    [](const CandidateMatch & left, const CandidateMatch & right) {
      if (left.descriptor_distance == right.descriptor_distance) {
        return left.keyframe_id < right.keyframe_id;
      }
      return left.descriptor_distance < right.descriptor_distance;
    });
  if (matches.size() > maximum_candidates) {
    matches.resize(maximum_candidates);
  }
  return matches;
}

const StaticKeyframe * StaticKeyframeDatabase::find(const std::size_t keyframe_id) const
{
  const auto iterator = std::find_if(
    keyframes_.begin(), keyframes_.end(),
    [keyframe_id](const StaticKeyframe & keyframe) {
      return keyframe.id == keyframe_id;
    });
  return iterator == keyframes_.end() ? nullptr : &(*iterator);
}

const std::deque<StaticKeyframe> & StaticKeyframeDatabase::keyframes() const
{
  return keyframes_;
}

std::size_t StaticKeyframeDatabase::descriptor_dimension() const
{
  return descriptor_dimension_;
}

bool StaticKeyframeDatabase::save_archive(
  const std::string & directory, const std::string & frame_id) const
{
  if (directory.empty() || frame_id.empty() || keyframes_.empty()) return false;
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::create_directories(directory, ec);
  if (ec) return false;
  const auto temporary = fs::path(directory) / "manifest.tmp";
  std::ofstream manifest(temporary);
  if (!manifest) return false;
  manifest << "UF_KEYFRAME_ARCHIVE 1\nframe " << frame_id << "\ncount "
           << keyframes_.size() << "\ndimension " << descriptor_dimension_ << "\n";
  manifest << std::setprecision(17);
  for (const auto & keyframe : keyframes_) {
    const auto filename = "keyframe_" + std::to_string(keyframe.id) + ".pcd";
    if (!keyframe.cloud || pcl::io::savePCDFileBinary(
        (fs::path(directory) / filename).string(), *keyframe.cloud) != 0) return false;
    const auto digest = file_sha256(fs::path(directory) / filename);
    if (digest.empty()) return false;
    manifest << keyframe.id << ' ' << filename << ' ' << digest << ' ' << keyframe.stamp_s;
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c)
      manifest << ' ' << keyframe.world_from_sensor.matrix()(r, c);
    manifest << ' ' << keyframe.quality.map_quality << ' '
             << keyframe.quality.feature_repeatability << ' '
             << keyframe.quality.dynamic_ratio << ' '
             << keyframe.quality.lidar_degradation << ' '
             << (keyframe.quality.scheduler_lidar_enabled ? 1 : 0) << ' '
             << keyframe.normalized_descriptor.size();
    for (const auto value : keyframe.normalized_descriptor) manifest << ' ' << value;
    manifest << '\n';
  }
  manifest.close();
  fs::rename(temporary, fs::path(directory) / "manifest", ec);
  return !ec;
}

bool StaticKeyframeDatabase::load_archive(
  const std::string & directory, const std::string & frame_id)
{
  namespace fs = std::filesystem;
  std::ifstream manifest(fs::path(directory) / "manifest");
  std::string magic, key; int version = 0; std::string stored_frame;
  std::size_t count = 0, dimension = 0;
  if (!(manifest >> magic >> version) || magic != "UF_KEYFRAME_ARCHIVE" || version != 1 ||
    !(manifest >> key >> stored_frame) || key != "frame" || stored_frame != frame_id ||
    !(manifest >> key >> count) || key != "count" || !(manifest >> key >> dimension) ||
    key != "dimension" || count == 0 || dimension == 0) return false;
  std::string line; std::getline(manifest, line);
  std::deque<StaticKeyframe> loaded;
  for (std::size_t i = 0; i < count; ++i) {
    if (!std::getline(manifest, line)) return false;
    std::istringstream stream(line); StaticKeyframe k; std::string filename; std::size_t d = 0;
    std::string digest;
    if (!(stream >> k.id >> filename >> digest >> k.stamp_s)) return false;
    if (digest != file_sha256(fs::path(directory) / filename)) return false;
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) if (!(stream >> k.world_from_sensor.matrix()(r,c))) return false;
    int enabled = 0;
    if (!(stream >> k.quality.map_quality >> k.quality.feature_repeatability >>
      k.quality.dynamic_ratio >> k.quality.lidar_degradation >> enabled >> d) || d != dimension) return false;
    k.quality.scheduler_lidar_enabled = enabled != 0;
    k.normalized_descriptor.resize(dimension);
    for (auto & value : k.normalized_descriptor) if (!(stream >> value) || !std::isfinite(value)) return false;
    auto cloud = std::make_shared<Cloud>();
    if (pcl::io::loadPCDFile((fs::path(directory) / filename).string(), *cloud) != 0 || cloud->empty()) return false;
    k.cloud = cloud;
    loaded.push_back(std::move(k));
  }
  keyframes_ = std::move(loaded); descriptor_dimension_ = dimension;
  next_keyframe_id_ = keyframes_.empty() ? 0 : keyframes_.back().id + 1;
  return true;
}

}  // namespace uf_relocalization
