#pragma once
#include "motion2d/trajectory/trajectory_cost.hpp"
namespace motion2d {
/** @brief Ascending seconds coefficients -> Bézier controls of derivative order 0..2.
 * @return (6-order)x6 map. The curve is unchanged; output derivatives have SI units.
 */
Eigen::MatrixXd bezierMap(double duration,int order=0);
/** @brief Direct coefficient/time partials of cubic-hinge control-point penalties.
 * @details Uses corridor/speed/acceleration settings, no ESDF at controls (which
 * could not certify a nonconvex free space). No time quadrature: sums controls.
 */
CoefficientGradient bezierCost(const PolynomialTrajectory & curve,
  const std::vector<ConvexRegion> & regions,const TrajectoryCostConfig & config);
/** @brief Sufficient bounds over every continuous segment, not finite trajectory samples. */
struct BezierCertificate {
  bool certified=false;
  double corridor_residual=-std::numeric_limits<double>::infinity();
  double speed_bound=0,acceleration_bound=0;
};
/** @brief Bound the exact curve via convex hulls, optionally tightened by subdivision.
 * @pre Each region has been validated in the robot configuration space; one per piece.
 * @details Rejects missing regions. Uses <=1e-9 arithmetic tolerance, not a physical
 * safety margin. Only certifies the represented curve against this fixed grid;
 * localization/tracking error require additional margins. Depth 0..10.
 */
BezierCertificate certifyBezier(const PolynomialTrajectory & curve,
  const std::vector<ConvexRegion> & regions,const TrajectoryLimits & limits,int depth=5);
}
