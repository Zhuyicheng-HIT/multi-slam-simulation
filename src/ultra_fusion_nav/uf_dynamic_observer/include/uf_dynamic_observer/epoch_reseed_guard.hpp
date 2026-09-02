#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace uf_dynamic_observer
{

enum class DynamicEpochState : std::uint8_t
{
  kReady = 0,
  kReseeding = 1,
};

const char * to_string(DynamicEpochState state);

struct EpochDecision
{
  bool accepted{false};
  bool reset_local_history{false};
  bool fail_open{false};
  std::string reason{"ignored"};
};

// Dynamic evidence lives in the FAST-LIO-local frame. A unified-backend
// FusionEpoch changes map_from_lio but does not change that local frame, so it
// is diagnostic-only here. A PreviousFastLioState reset_counter change is the
// authoritative signal that local voxel/free-space history became invalid.
class EpochReseedGuard
{
public:
  explicit EpochReseedGuard(std::size_t required_healthy_scans = 6U);

  EpochDecision observe_lio_state(std::uint32_t reset_counter);
  EpochDecision observe_backend_epoch(
    bool applied, std::uint64_t session_id, std::uint64_t transaction_id,
    std::uint32_t reset_counter);

  // A scan used as causal reseed evidence is still passed through raw. Clean
  // filtering resumes on the following scan after the configured count.
  bool observe_reseed_scan(bool healthy);

  bool fail_open_required() const {return state_ == DynamicEpochState::kReseeding;}
  DynamicEpochState state() const {return state_;}
  std::size_t reseed_scans() const {return reseed_scans_;}
  std::size_t required_healthy_scans() const {return required_healthy_scans_;}
  std::uint32_t lio_reset_counter() const {return lio_reset_counter_;}
  std::uint64_t backend_epoch_count() const {return backend_epoch_count_;}
  std::uint64_t ignored_backend_epoch_count() const {return ignored_backend_epoch_count_;}

private:
  std::size_t required_healthy_scans_{6U};
  std::size_t reseed_scans_{0U};
  DynamicEpochState state_{DynamicEpochState::kReady};
  bool lio_epoch_initialized_{false};
  std::uint32_t lio_reset_counter_{0U};
  bool backend_epoch_initialized_{false};
  std::uint64_t backend_session_id_{0U};
  std::uint64_t backend_transaction_id_{0U};
  std::uint32_t backend_reset_counter_{0U};
  std::uint64_t backend_epoch_count_{0U};
  std::uint64_t ignored_backend_epoch_count_{0U};
};

}  // namespace uf_dynamic_observer
