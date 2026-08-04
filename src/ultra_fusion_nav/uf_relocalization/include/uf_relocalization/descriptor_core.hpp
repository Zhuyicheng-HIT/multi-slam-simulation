#pragma once

#include "uf_relocalization/registration_core.hpp"

#include <vector>

namespace uf_relocalization
{

std::vector<float> compute_esf_descriptor(const Cloud::ConstPtr & cloud);

}  // namespace uf_relocalization
