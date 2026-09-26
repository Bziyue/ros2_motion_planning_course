#pragma once
#include <cstdint>
#include "motion2d/geometry/obstacles.hpp"

namespace motion2d
{
/** @brief Static map-frame geometry; rectangle centred at zero. All lengths in m. */
struct World2D
{
  double width = 20.0;
  double height = 20.0;
  Eigen::Vector2d start{-8.0, -8.0};
  Eigen::Vector2d goal{8.0, 8.0};
  std::vector<Circle> circles;
  std::vector<Polygon> polygons;
};

/** @brief Chapter 03 generation settings. No hidden reseeding on failure. */
struct WorldConfig
{
  std::uint32_t seed = 42;
  double width = 20.0, height = 20.0;
  int circle_count = 8, polygon_count = 8;
  double size_min = 0.35, size_max = 1.2;
  int polygon_samples = 8;
  Eigen::Vector2d start{-8.0, -8.0}, goal{8.0, 8.0};
  double robot_radius = 0.2, margin = 0.05;
  bool require_connected = true;
  double check_resolution = 0.25;
};

/** @brief Smallest signed distance to obstacles or the rectangle boundary (m).
 *  @details Exact clearance in free space. Inside an obstacle union or outside
 *  the rectangle, use its sign for collision; it is not a full-space exact ESDF.
 */
double clearance(const World2D & world, const Eigen::Vector2d & center);

/** @brief True if a disk is strictly in free space; contact is a collision.
 *  @pre radius >= 0 and all inputs finite.
 */
bool isFree(const World2D & world, const Eigen::Vector2d & center, double radius);

/**
 * @brief Conservative 4-connected flood-fill check, not a shortest-path planner.
 * @param radius Disk radius including the desired extra safety margin (m).
 * @param resolution Maximum grid cell edge length (m), positive.
 * @details Obstacles are expanded by an additional cell half-diagonal, so a
 * free cell's whole area is clear. False can be a discretisation false negative.
 */
bool isReachable(const World2D & world, double radius, double resolution);

/** @brief Generate a deterministic world; throw if placement/connectivity fails.
 *  @details Reproducible with the same seed, config and C++ standard library.
 *  Overlap between obstacles is allowed. Start/goal disks stay clear.
 */
World2D generateWorld(const WorldConfig & config);
}  // namespace motion2d
