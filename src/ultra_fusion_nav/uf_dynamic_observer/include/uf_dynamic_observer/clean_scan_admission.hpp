#pragma once

#include "uf_dynamic_observer/conservative_free_space.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace uf_dynamic_observer
{

struct CleanScanAdmissionResult
{
  bool healthy{false};
  bool fail_open{true};
  std::string reason{"not_evaluated"};
  std::vector<bool> keep;
  std::size_t static_points{0U};
  std::size_t dynamic_removed{0U};
  std::size_t unknown_points{0U};
};

// Converts observer labels back to the original Livox point ordering. Points
// that were outside the observer's valid range remain UNKNOWN and are kept.
// Any malformed index/label contract fails open to the complete raw scan.
class CleanScanAdmission
{
public:
  CleanScanAdmissionResult apply(
    std::size_t raw_point_count, const std::vector<std::size_t> & classified_source_indices,
    const std::vector<LabeledPoint> & classified_points) const;
};

}  // namespace uf_dynamic_observer
