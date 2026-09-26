#pragma once
#include "motion2d/trajectory/polynomial.hpp"
#include "motion2d/trajectory/banded_lu.hpp"

namespace motion2d
{
/** @brief Exact Gram matrix Q(T) such that jerk energy is trace(C^T Q C).
 * @param duration Positive local duration in seconds.
 * @details C is 6x2 with ascending rows. Energy units are m²/s⁵.
 */
Eigen::Matrix<double,6,6> jerkGram(double duration);

/** @brief Derivatives of a cost before eliminating polynomial coefficients. */
struct CoefficientGradient
{
  double cost = 0;
  Eigen::MatrixX2d coefficients;  ///< 6N rows, x/y columns; partial cost / partial c.
  Eigen::VectorXd times;         ///< N partials holding c fixed, not the final dJ/dT.
};

/** @brief Derivatives after eliminating coefficients with a MINCO adjoint solve. */
struct MincoGradient
{
  Eigen::MatrixX2d waypoints;  ///< N-1 rows; start/finish boundaries are held fixed.
  Eigen::VectorXd times;
};

/** @brief Clamped planar minimum-jerk trajectory through fixed positions at fixed times.
 * @details Solves a 6N band system for ascending coefficients, using C4 interior
 * continuity and p/v/a endpoint conditions. Equivalent to an equality-constrained
 * minimum-jerk quadratic problem; contains no obstacle or actuator constraints.
 * The course implementation is derived from these equations, not an upstream wrapper.
 */
class Minco2D
{
public:
  /** @param interior N-1 internal positions; no endpoint duplication.
   * @param durations N positive durations in seconds. Finite SI inputs required.
   * @throws std::invalid_argument on invalid data; std::runtime_error on numerical failure.
   */
  Minco2D(const TranslationState & start,const TranslationState & finish,
    const std::vector<Eigen::Vector2d> & interior,const std::vector<double> & durations);
  /** @brief Convert coefficients to the chapter-14 common sampling API. */
  PolynomialTrajectory trajectory() const;
  /** @brief Ascending coefficients, rows grouped six per piece, columns x/y. */
  const Eigen::MatrixX2d & coefficients() const {return coefficients_;}
  /** @brief Durations used by the factorization. */
  const std::vector<double> & durations() const {return durations_;}
  /** @brief Exact integral of squared jerk and direct coefficient/time partials. */
  CoefficientGradient energyPartials() const;
  /** @brief Backpropagate any differentiable coefficient/time cost in O(N).
   * @details Solves A^T Lambda = dJ/dC, then dJ/dT = partial_T J - Lambda : (dA/dT) C.
   */
  MincoGradient propagate(const Eigen::MatrixX2d & coefficient_gradient,
    const Eigen::VectorXd & direct_time_gradient) const;
private:
  std::vector<double> durations_;
  Eigen::MatrixX2d coefficients_;
  BandedLu factor_;
};
}  // namespace motion2d
