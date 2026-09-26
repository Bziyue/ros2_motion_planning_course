#pragma once
#include <string>
#include <vector>
#include "motion2d/geometry/se2.hpp"

namespace motion2d
{
/** @brief Relative pose Z_ij=T_i^-1*T_j, from node j into node i; units m,m,rad. */
struct PoseGraphEdge
{
  std::size_t from = 0, to = 0;
  Pose2D relative;
  Eigen::Vector3d stddev{.03, .03, .02}; ///< Residual-coordinate scales, not ICP Hessian inverse.
  bool loop = false; ///< Apply robust loss to loop edges only.
};

/** @brief Additive world-position/yaw derivatives for one SE(2) residual. */
struct GraphLinearization
{
  Eigen::Vector3d residual;
  Eigen::Matrix3d from, to;
};

/** @brief Residual coordinates of Z_ij^-1*T_i^-1*T_j (not the SE(2) logarithm).
 * @details Translation in measured edge axes, wrapped yaw. Jacobians use additive
 * global x/y and yaw perturbations; finite-difference tested away from the pi cut.
 */
GraphLinearization linearizeGraphEdge(const Pose2D & from, const Pose2D & to,
  const Pose2D & measured);

/** @brief Small-scene optimization settings; Huber delta is in whitened residual units. */
struct PoseGraphConfig
{
  int max_iterations = 30;
  double loop_huber = 3.;
  double step_tolerance = 1e-7;
};

/** @brief Explicit solve outcome; an unaccepted proposal must not replace the map. */
struct PoseGraphResult
{
  std::vector<Pose2D> poses;
  std::string status = "iteration_limit";
  bool converged = false;
  int iterations = 0;
  double initial_cost = 0, final_cost = 0;
};

/** @brief Sum of half squared residuals, Huber on whitened loop residual norms. */
double poseGraphCost(const std::vector<Pose2D> & poses, const std::vector<PoseGraphEdge> & edges,
  double loop_huber = 3.);

/** @brief Sparse Gauss-Newton with backtracking; fixes pose 0 exactly to remove gauge.
 * @pre Finite poses, valid indices/scales, connected graph; disconnected inputs
 * return an explicit status. This estimates a local minimum, not place identity.
 */
PoseGraphResult optimizePoseGraph(const std::vector<Pose2D> & initial,
  const std::vector<PoseGraphEdge> & edges, const PoseGraphConfig & config = {});
}  // namespace motion2d
