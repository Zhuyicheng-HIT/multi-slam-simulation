#include "uf_dynamic_observer/epoch_reseed_guard.hpp"

#include <algorithm>
#include <stdexcept>

namespace uf_dynamic_observer
{

const char * to_string(const DynamicEpochState state)
{
  switch (state) {
    case DynamicEpochState::kReady:
      return "READY";
    case DynamicEpochState::kReseeding:
      return "RESEEDING";
  }
  return "UNKNOWN";
}

EpochReseedGuard::EpochReseedGuard(const std::size_t required_healthy_scans)
: required_healthy_scans_(required_healthy_scans)
{
  if (required_healthy_scans_ == 0U) {
    throw std::invalid_argument("required_healthy_scans must be positive");
  }
}

EpochDecision EpochReseedGuard::observe_lio_state(const std::uint32_t reset_counter)
{
  if (!lio_epoch_initialized_) {
    lio_epoch_initialized_ = true;
    lio_reset_counter_ = reset_counter;
    return {true, false, false, "initial_lio_epoch"};
  }
  if (reset_counter == lio_reset_counter_) {
    return {true, false, fail_open_required(), "same_lio_epoch"};
  }
  if (reset_counter < lio_reset_counter_) {
    return {false, false, fail_open_required(), "stale_lio_epoch"};
  }
  lio_reset_counter_ = reset_counter;
  reseed_scans_ = 0U;
  state_ = DynamicEpochState::kReseeding;
  return {true, true, true, "lio_local_epoch_changed"};
}

EpochDecision EpochReseedGuard::observe_backend_epoch(
  const bool applied, const std::uint64_t session_id,
  const std::uint64_t transaction_id, const std::uint32_t reset_counter)
{
  if (!applied) {
    ++ignored_backend_epoch_count_;
    return {false, false, fail_open_required(), "backend_epoch_not_applied"};
  }
  if (backend_epoch_initialized_) {
    const bool older_session = session_id < backend_session_id_;
    const bool same_or_older_transaction = session_id == backend_session_id_ &&
      transaction_id <= backend_transaction_id_;
    const bool regressed_counter = session_id == backend_session_id_ &&
      reset_counter < backend_reset_counter_;
    if (older_session || same_or_older_transaction || regressed_counter) {
      ++ignored_backend_epoch_count_;
      return {false, false, fail_open_required(), "stale_or_duplicate_backend_epoch"};
    }
  }
  backend_epoch_initialized_ = true;
  backend_session_id_ = session_id;
  backend_transaction_id_ = transaction_id;
  backend_reset_counter_ = reset_counter;
  ++backend_epoch_count_;
  return {true, false, fail_open_required(), "backend_alignment_epoch_retains_lio_history"};
}

bool EpochReseedGuard::observe_reseed_scan(const bool healthy)
{
  if (state_ != DynamicEpochState::kReseeding || !healthy) {
    return false;
  }
  reseed_scans_ = std::min(reseed_scans_ + 1U, required_healthy_scans_);
  if (reseed_scans_ >= required_healthy_scans_) {
    state_ = DynamicEpochState::kReady;
    return true;
  }
  return false;
}

}  // namespace uf_dynamic_observer
