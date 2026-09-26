#pragma once
#include <Eigen/Core>
#include <functional>
#include <string>
#include <limits>

namespace motion2d
{
/** @brief Finite value/gradient, or +inf with a reason for a rejected trial point. */
struct ObjectiveValue
{
  double value=std::numeric_limits<double>::infinity();
  Eigen::VectorXd gradient;
  std::string error;
};
struct BfgsConfig
{
  int max_iterations=200;
  double gradient_tolerance=1e-5;
  double max_wall_seconds=0; ///< Zero disables wall-time budget; checked between evaluations.
};
struct BfgsResult
{
  Eigen::VectorXd x;
  ObjectiveValue objective;
  std::string status;
  int iterations=0,evaluations=0;
  double seconds=0;
  bool converged() const {return status=="converged";}
};
/** @brief Small dense inverse-BFGS with Armijo backtracking for teaching-sized problems.
 * @details O(d²) storage belongs to the outer optimizer, unlike the O(N) MINCO
 * elimination. Not a constrained/global solver. Invalid trials shrink the step;
 * a timeout/failed line search has an explicit status and retains diagnostic x.
 */
BfgsResult minimizeBfgs(const std::function<ObjectiveValue(const Eigen::VectorXd &)> & objective,
  const Eigen::VectorXd & initial,const BfgsConfig & config={});
}  // namespace motion2d
