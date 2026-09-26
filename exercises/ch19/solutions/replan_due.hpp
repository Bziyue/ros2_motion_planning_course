#pragma once
#include <cstdint>
#include <optional>
/** @brief A backwards clock requires clearing last elsewhere, never a negative duration. */
inline bool studentReplanDue(std::int64_t now,std::optional<std::int64_t> last,std::int64_t period,bool waiting) {
  return !waiting && (!last || (now>=*last && now-*last>=period));
}
