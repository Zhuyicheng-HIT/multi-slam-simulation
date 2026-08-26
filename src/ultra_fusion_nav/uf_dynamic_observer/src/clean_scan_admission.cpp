#include "uf_dynamic_observer/clean_scan_admission.hpp"

#include <algorithm>

namespace uf_dynamic_observer
{

CleanScanAdmissionResult CleanScanAdmission::apply(
  std::size_t raw_point_count, const std::vector<std::size_t> & classified_source_indices,
  const std::vector<LabeledPoint> & classified_points) const
{
  CleanScanAdmissionResult result;
  result.keep.assign(raw_point_count, true);
  if (classified_source_indices.size() != classified_points.size()) {
    result.reason = "classification_size_mismatch";
    return result;
  }

  std::vector<bool> visited(raw_point_count, false);
  for (std::size_t index = 0U; index < classified_points.size(); ++index) {
    const auto source_index = classified_source_indices[index];
    if (source_index >= raw_point_count || visited[source_index]) {
      result.reason = source_index >= raw_point_count ?
        "classification_index_out_of_range" : "classification_index_duplicate";
      std::fill(result.keep.begin(), result.keep.end(), true);
      return result;
    }
    visited[source_index] = true;
    switch (classified_points[index].label) {
      case PointLabel::kStatic:
        ++result.static_points;
        break;
      case PointLabel::kDynamic:
        result.keep[source_index] = false;
        ++result.dynamic_removed;
        break;
      case PointLabel::kUnknown:
        ++result.unknown_points;
        break;
    }
  }

  const auto retained = static_cast<std::size_t>(std::count(
    result.keep.begin(), result.keep.end(), true));
  if (raw_point_count > 0U && retained == 0U) {
    result.reason = "empty_clean_scan_guard";
    std::fill(result.keep.begin(), result.keep.end(), true);
    result.dynamic_removed = 0U;
    return result;
  }
  result.healthy = true;
  result.fail_open = false;
  result.reason = "ok";
  return result;
}

}  // namespace uf_dynamic_observer
