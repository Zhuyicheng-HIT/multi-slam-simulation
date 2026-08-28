#include "uf_relocalization/offline_esf_seed.hpp"

#include <atomic>
#include <ctime>

namespace
{

std::atomic<std::uint32_t> g_offline_esf_seed{1731U};

}  // namespace

namespace uf_relocalization
{

void set_offline_esf_seed(const std::uint32_t seed)
{
  g_offline_esf_seed.store(seed, std::memory_order_relaxed);
}

std::uint32_t offline_esf_seed()
{
  return g_offline_esf_seed.load(std::memory_order_relaxed);
}

}  // namespace uf_relocalization

// A symbol exported by an executable preempts the libc symbol used by PCL's
// shared feature library. Only the offline frontend and its focused test link
// this object; production nodes never do.
extern "C" std::time_t time(std::time_t * output) noexcept
{
  const auto value = static_cast<std::time_t>(uf_relocalization::offline_esf_seed());
  if (output != nullptr) {
    *output = value;
  }
  return value;
}
