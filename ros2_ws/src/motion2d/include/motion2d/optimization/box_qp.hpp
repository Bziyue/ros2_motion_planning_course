#pragma once
#include <Eigen/Core>
#include <string>
#include <limits>
namespace motion2d {
/** @brief Strongly convex dense QP: min .5*x'P*x+q'x, lower<=A*x<=upper.
 * @details P must be symmetric positive definite. Infinite one-sided bounds allowed.
 * This small teaching solver is independent of the OSQP library, not a full replacement.
 */
struct BoxQp {Eigen::MatrixXd P,A;Eigen::VectorXd q,lower,upper;};
/** @brief ADMM tolerances and iteration/wall-time budgets; zero wall budget disables it. */
struct QpSettings {
  int max_iterations=1500;
  double absolute_tolerance=1e-5,relative_tolerance=1e-5,rho=.1,sigma=1e-6,max_wall_seconds=0;
};
/** @brief Iterates are diagnostic only unless status is solved. */
struct QpResult {
  Eigen::VectorXd x;
  std::string status;
  int iterations=0;
  double primal_residual=std::numeric_limits<double>::infinity(),dual_residual=std::numeric_limits<double>::infinity(),seconds=0;
  bool solved() const {return status=="solved";}
};
/** @brief Projected ADMM with dense LDLT and residual-based rho adaptation.
 * @param warm_start Optional finite primal iterate of the correct dimension.
 * @details Invalid input throws. Contradictory bounds or a normalized primal
 * certificate produce primal_infeasible; max_iterations does NOT imply infeasibility.
 * Wall budget checked between iterations, not a hard real-time guarantee.
 */
QpResult solveBoxQp(const BoxQp & problem,const QpSettings & settings={},
  const Eigen::VectorXd & warm_start={});
/** @brief Largest positive bound violation, zero for a feasible point. */
double qpViolation(const BoxQp & problem,const Eigen::VectorXd & x);
}
