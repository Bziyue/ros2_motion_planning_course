#pragma once
#include "motion2d/mapping/occupancy_grid.hpp"
#include "motion2d/sim/world.hpp"

namespace motion2d
{
/** @brief Independent filled-geometry reference: 1 if a closed cell touches solid.
 * @details Circle/box distance and convex polygon SAT, not simulated rays.
 * Includes room walls and the outside. This oracle is for evaluation only;
 * mapping_node must never use it. A reference cell occupies its full square.
 * @pre Valid world; grid is validated by OccupancyGrid2D.
 */
std::vector<std::uint8_t> rasterizeTruth(const World2D & world, const GridConfig & grid);

/** @brief Confusion counts on observed, confident cells only; thresholds 35/65. */
struct MapMetrics
{
  int observed = 0, uncertain = 0;
  int true_positive = 0, false_positive = 0, false_negative = 0, true_negative = 0;
  /** @brief TP/(TP+FP), NaN if no occupied prediction. */
  double precision() const;
  /** @brief TP/(TP+FN), conditional on evaluated cells; NaN for empty denominator. */
  double recall() const;
};

/** @brief Compare equal-size grids, preserving unknown and uncertain exclusions.
 * @param observed -1 unknown or [0,100] probability percentage.
 * @param truth Independent geometry occupancy, 0 or 1.
 * @throws std::invalid_argument on size/encoding mismatch.
 * @details This is not full-map recall: unseen and uncertain cells are excluded.
 */
MapMetrics evaluateObserved(const std::vector<std::int8_t> & observed,
  const std::vector<std::uint8_t> & truth);
}  // namespace motion2d
