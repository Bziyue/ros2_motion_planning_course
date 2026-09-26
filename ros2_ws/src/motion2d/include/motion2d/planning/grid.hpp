#pragma once
#include <cstdint>
#include <optional>
#include <vector>
#include "motion2d/mapping/occupancy_grid.hpp"

namespace motion2d
{
/** @brief Planning policy for a disk; only observed p<=free_threshold is free. */
struct InflationConfig
{
  double radius = .2, margin = .05; ///< Physical radius plus deliberate clearance, m.
  int free_threshold = 35;
  bool unknown_blocked = true; ///< Disabling this is an explicitly optimistic experiment.
};

/** @brief Configuration-space grid with WHOLE free cells certified conservatively.
 * @details Blocked original cells occupy their full square area; exterior is
 * blocked. A free output cell's entire closed area stays farther than radius+
 * margin from those sets. This sacrifices thin passages at coarse resolution.
 */
struct PlanningGrid
{
  GridConfig geometry;
  std::vector<std::uint8_t> blocked; ///< Row-major y*width+x, nonzero means blocked.
  double clearance_radius = 0;
  /** @brief Point cell under the same half-open convention as occupancy, or nullopt. */
  std::optional<int> cellIndex(const Eigen::Vector2d & point) const;
  Eigen::Vector2d cellCenter(int index) const;
  /** @brief Outside indices are always blocked. */
  bool freeCell(int x, int y) const;
};

/** @brief Inflate occupied/uncertain/unknown square cells for a disk robot.
 * @details Exact minimum square-to-square distance is used for the small stencil,
 * avoiding the unsafe approximation of treating occupied cells as zero-area points.
 * Source values must be -1 or 0..100. Input geometry is validated at this boundary.
 */
PlanningGrid inflateGrid(const GridConfig & geometry, const std::vector<std::int8_t> & occupancy,
  const InflationConfig & config = {});

/** @brief Closed segment vs blocked squares, including corner/edge contacts.
 * @details Checks the segment's grid bounding box analytically. Slower than a
 * specialized supercover DDA, but easy to audit; regular mapping DDA is NOT safe
 * for this purpose because it skips cells touched only at corners.
 */
bool segmentIsFree(const PlanningGrid & grid, const Eigen::Vector2d & from,
  const Eigen::Vector2d & to);
}  // namespace motion2d
