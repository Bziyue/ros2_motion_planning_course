#pragma once
#include <vector>
#include <cstdint>

namespace motion2d
{
/** @brief Exact squared Euclidean distance to nonzero seed cells, in cell^2.
 * @param seeds Row-major width*height binary mask; any nonzero is a seed.
 * @return +infinity everywhere when no seed exists.
 * @details Separable lower envelopes of parabolas, O(width*height). No padding,
 * no radius, no square-area correction. See chapter 12 and Felzenszwalb /
 * Huttenlocher, Theory of Computing 8 (2012), doi:10.4086/toc.2012.v008a019.
 */
std::vector<double> squaredDistanceTransform(int width, int height,
  const std::vector<std::uint8_t> & seeds);

/** @brief Classify raw occupancy (-1 or 0..100) WITHOUT spatial inflation.
 * @details Observed values <=free_threshold are free. Unknown is independently
 * controlled; optimistic unknown handling does not certify physical free space.
 */
std::vector<std::uint8_t> obstacleMask(const std::vector<std::int8_t> & occupancy,
  int free_threshold = 35, bool unknown_blocked = true);
}  // namespace motion2d
