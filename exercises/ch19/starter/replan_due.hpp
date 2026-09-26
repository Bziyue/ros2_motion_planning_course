#pragma once
#include <cstdint>
#include <optional>
/** @brief EXERCISE(ch19-2): trigger at simulated period, except while awaiting a handover. */
inline bool studentReplanDue(std::int64_t now,std::optional<std::int64_t> last,std::int64_t period,bool waiting) {
  (void)now;(void)last;(void)period;(void)waiting;
  return true; // TODO: first call, pause, threshold, pending, rollback.
}
