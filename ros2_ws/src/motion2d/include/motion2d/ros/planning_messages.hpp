#pragma once
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "motion2d/planning/corridor.hpp"

namespace motion2d
{
/** @brief Geometric map-frame path, no timing law or commanded yaw; identity orientations. */
nav_msgs::msg::Path toPath(const std::vector<Eigen::Vector2d> & points, const std_msgs::msg::Header & header);
/** @brief Closed corridor outlines in route order; starts with DELETEALL to clear older regions.
 * @details Markers are visualization, not the planning API; algorithms use ConvexRegion directly.
 */
visualization_msgs::msg::MarkerArray toCorridorMarkers(
  const std::vector<ConvexRegion> & regions, const std_msgs::msg::Header & header);
}  // namespace motion2d
