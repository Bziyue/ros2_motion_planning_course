#pragma once
#include <array>
#include <cstdint>
#include <vector>
/** @brief Reference confusion counts with unknown/uncertain excluded. */
inline std::array<int, 4> studentConfusion(const std::vector<std::int8_t> & observed,
  const std::vector<std::uint8_t> & truth)
{
  std::array<int, 4> counts{};
  for (std::size_t i = 0; i < observed.size(); ++i) {
    const int value = observed[i];
    if (value == -1 || (value > 35 && value < 65)) {continue;}
    if (value >= 65) {++counts[truth[i] ? 0 : 1];}
    else {++counts[truth[i] ? 2 : 3];}
  }
  return counts;
}
