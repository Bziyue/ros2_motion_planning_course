#pragma once
#include <array>
#include <optional>
#include "motion2d/mapping/occupancy_grid.hpp"

namespace motion2d
{
/** @brief Bilinear value (m) and analytic gradient (m/m) within one cell-centre patch. */
struct BilinearSample
{
  double value;
  Eigen::Vector2d gradient;
};
/** @brief Interpolate values ordered [00,10,01,11], fraction u,v in [0,1].
 * @pre Finite values, positive resolution, valid fractions. Gradient is in metre coordinates.
 */
BilinearSample bilinear(const std::array<double, 4> & values, double u, double v, double resolution);

struct EsdfSample
{
  double distance; ///< Signed centre-distance interpolation (m), NOT exact square-boundary distance.
  Eigen::Vector2d gradient; ///< Gradient of distance, not of the clearance bound; generally not unit norm.
  double clearance_lower_bound; ///< Certified lower bound to blocked SQUARES/exterior, m; zero is inconclusive.
};

/** @brief Chapter 12 discrete signed distance field of an UNINFLATED occupancy map.
 * @details Free centres have +distance to blocked centres, blocked centres have
 * -distance to free centres. A blocked padding ring models the exterior. There
 * is no free source for an entirely blocked map, so negative values are -inf.
 * Bilinear values help optimization; clearance bounds, not raw values, certify
 * point-to-square clearance. A finite set of samples does not certify a curve.
 */
class Esdf2D
{
public:
  /** @brief Build two exact EDTs. Geometry and encoding are validated here. */
  Esdf2D(const GridConfig & geometry, const std::vector<std::int8_t> & occupancy,
    int free_threshold = 35, bool unknown_blocked = true);
  const GridConfig & geometry() const {return geometry_;}
  const std::vector<double> & distances() const {return distances_;}
  const std::vector<std::uint8_t> & blocked() const {return blocked_;}
  /** @brief Query between outermost cell centres; nullopt outside or if a corner is infinite.
   * @details Gradient may jump across interpolation patch boundaries and nearest-site ridges.
   * The lower bound accounts for square half-diagonal and interpolation error.
   */
  std::optional<EsdfSample> sample(const Eigen::Vector2d & point) const;
private:
  GridConfig geometry_;
  std::vector<std::uint8_t> blocked_;
  std::vector<double> distances_;
};
}  // namespace motion2d
