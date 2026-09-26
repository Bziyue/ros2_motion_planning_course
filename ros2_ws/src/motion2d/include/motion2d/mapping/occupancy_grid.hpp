#pragma once
#include <cstdint>
#include <optional>
#include "motion2d/mapping/scan_projection.hpp"

namespace motion2d
{
/** @brief Fixed map geometry and inverse sensor model; resolution/origin in m. */
struct GridConfig
{
  double resolution = .1;
  int width = 220, height = 220;  ///< Number of cells, not metres.
  Eigen::Vector2d origin{-11, -11};  ///< Lower-left corner of cell (0,0).
  double hit_probability = .7, miss_probability = .4;
  double logodds_limit = 4.0;  ///< Symmetric saturation, dimensionless.
};

/** @brief Observation-only occupancy grid, with unknown distinct from p=0.5.
 * @details Row-major index=y*width+x. Each scan contributes at most once per
 * cell; endpoint hits take precedence over traversals within that scan.
 * Grid bounds are fixed and half-open. See chapter 07.
 */
class OccupancyGrid2D
{
public:
  /** @brief Allocate an unknown map; reject invalid sizes/probabilities. */
  explicit OccupancyGrid2D(const GridConfig & config);
  const GridConfig & config() const {return config_;}
  /** @brief Return the cell containing p, or nullopt outside the half-open map. */
  std::optional<int> cellIndex(const Eigen::Vector2d & point) const;
  /** @brief Centre of a valid row-major cell, in map metres. */
  Eigen::Vector2d cellCenter(int index) const;
  /** @brief DDA cells along a segment, including start and clipped end cells.
   * @pre Finite endpoints. Start must lie inside the map; otherwise returns empty.
   * @details Clip an outside end to the map. At an exact grid corner, step both
   * axes: cells touched only at that corner are not visited. Not a collision test.
   */
  std::vector<int> rayCells(const Eigen::Vector2d & start, const Eigen::Vector2d & end) const;
  /** @brief Apply one scan; false means its origin was outside the fixed map.
   * @pre Valid scan and finite acquisition pose T_map_laser.
   * @details Finite hits clear preceding cells; +inf clears through range_max;
   * invalid values do nothing. True obstacles/world geometry are never inputs.
   */
  bool insertScan(const Scan2D & scan, const Pose2D & laser_pose);
  /** @brief -1 for unobserved, otherwise rounded 100*sigmoid(log_odds). */
  std::vector<std::int8_t> occupancy() const;
  /** @brief Discard all observations when starting a new trial. */
  void clear();
private:
  GridConfig config_;
  std::vector<float> log_odds_;
  std::vector<std::uint8_t> observed_;
  double hit_increment_, miss_increment_;
};
}  // namespace motion2d
