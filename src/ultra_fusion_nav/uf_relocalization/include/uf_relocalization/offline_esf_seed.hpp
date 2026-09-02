#pragma once

#include <cstdint>

namespace uf_relocalization
{

// PCL 1.12 seeds ESF sampling from wall-clock seconds inside every compute.
// Only the offline frontend links the scoped time() wrapper using this seed.
void set_offline_esf_seed(std::uint32_t seed);
std::uint32_t offline_esf_seed();

}  // namespace uf_relocalization
