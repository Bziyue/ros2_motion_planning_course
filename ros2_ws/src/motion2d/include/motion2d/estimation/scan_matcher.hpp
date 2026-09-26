#pragma once
#include <limits>
#include <string>
#include <vector>
#include "motion2d/geometry/se2.hpp"

namespace motion2d
{
using PointCloud2D = std::vector<Eigen::Vector2d>;
enum class MatchMetric {PointToPoint, PointToLine};
enum class MatchStatus {Converged, TooFewPairs, Degenerate, IterationLimit, HighResidual};

/** @brief ICP configuration. Distances in m, yaw increments in rad; ch08. */
struct MatchConfig
{
  MatchMetric metric = MatchMetric::PointToPoint;
  int max_iterations = 40;
  int min_pairs = 20;
  double association_distance = 0.5;
  double huber_delta = 0.08;
  double translation_tolerance = 1e-5;
  double yaw_tolerance = 1e-5;
  double min_eigen_ratio = 1e-6;
  double max_rmse = 0.15;
};

/** @brief A fixed target in the reference frame; zero normals mean no usable line. */
struct MatchTarget
{
  PointCloud2D points;
  PointCloud2D normals;
};

/** @brief Diagnostic result; only Converged is accepted by odometry.
 * @details Information is the final weighted normal matrix, NOT a calibrated
 * covariance. Nearest-neighbour associations and a reused map are correlated.
 */
struct MatchResult
{
  Pose2D pose;
  MatchStatus status = MatchStatus::TooFewPairs;
  int pairs = 0, iterations = 0;
  double rmse = std::numeric_limits<double>::infinity();
  Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
  bool accepted() const {return status == MatchStatus::Converged;}
};

/** @brief Estimate local line normals by radius-neighbour PCA, in metres.
 * @param points Finite target points.
 * @param normal_radius Neighbours farther than this distance are ignored.
 * @details Needs >=5 neighbours and lambda_min/lambda_max < .15. Normals do
 * not depend on beam order, so invalid beams and submaps are supported.
 */
MatchTarget makeMatchTarget(const PointCloud2D & points, double normal_radius = 0.4);

/** @brief Jacobian of R(yaw)*point+t w.r.t. additive [tx,ty,yaw]; ch08.
 * @param rotated_point R(yaw)*point; do not include translation.
 */
Eigen::Matrix<double, 2, 3> pointJacobian(const Eigen::Vector2d & rotated_point);

/** @brief Robust ICP aligning current-frame source points into target's frame.
 * @param initial T_target_source used as local initial guess.
 * @details Brute-force nearest neighbours, Huber IRLS, Gauss-Newton and a
 * rank check. This is a local method, not a global relocalizer. Throws for
 * malformed configurations/nonfinite clouds; empty clouds return TooFewPairs.
 */
MatchResult matchClouds(const PointCloud2D & source, const MatchTarget & target,
  const Pose2D & initial = {}, const MatchConfig & config = {});

/** @brief Stable diagnostic text for ROS status and experiment CSV files. */
std::string matchStatusName(MatchStatus status);
}  // namespace motion2d
