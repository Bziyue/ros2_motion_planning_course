#pragma once
#include "motion2d/trajectory/polynomial.hpp"
namespace motion2d {
/** @brief Planar clamped minimum-jerk spline via internal velocity/acceleration.
 * @details Eliminates each segment with a 6x6 endpoint map; the remaining
 * symmetric system is block tridiagonal with 2x2 blocks. Same mathematical
 * problem as Minco2D, not a new curve family or an interpolating B-spline.
 */
class Spline2D {
public:
  /** @brief Fixed endpoint p/v/a, N-1 interior positions and N positive SI durations. */
  Spline2D(const TranslationState & start,const TranslationState & finish,
    const std::vector<Eigen::Vector2d> & interior,const std::vector<double> & durations);
  /** @brief Common seconds/ascending-coefficient trajectory representation. */
  PolynomialTrajectory trajectory() const;
  const Eigen::MatrixX2d & coefficients() const {return coefficients_;}
  /** @brief Exact jerk integral, m²/s⁵. */
  double energy() const;
  /** @brief Direct variational energy gradient; no separate adjoint solve. */
  TrajectoryGradient energyGradient() const;
  /** @brief General analytic adjoint for any coefficient/direct-time cost. */
  TrajectoryGradient propagate(const Eigen::MatrixX2d & gradient,const Eigen::VectorXd & direct) const;
private:
  using Matrix6=Eigen::Matrix<double,6,6>;
  std::vector<double> times_;
  std::vector<Matrix6> endpoint_maps_,energy_maps_;
  std::vector<Eigen::Matrix2d> inverse_diagonal_,lower_;
  Eigen::MatrixX2d states_,coefficients_; // states: p/v/a per knot
  Eigen::MatrixX2d solveBlocks(Eigen::MatrixX2d rhs) const;
};
}
