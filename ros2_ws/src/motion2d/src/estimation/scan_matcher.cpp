#include "motion2d/estimation/scan_matcher.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

namespace motion2d
{
namespace
{
void validatePoints(const PointCloud2D & points)
{
  for (const auto & p : points) {
    if (!p.allFinite()) {throw std::invalid_argument("ICP requires finite points");}
  }
}
struct NormalEquations
{
  Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
  Eigen::Vector3d g = Eigen::Vector3d::Zero();
  int count = 0;
  double squared_error = 0;
};

NormalEquations linearize(const PointCloud2D & source, const MatchTarget & target,
  const Pose2D & pose, const MatchConfig & config)
{
  NormalEquations system;
  const auto r = rotation(pose.yaw);
  for (const auto & p : source) {
    const Eigen::Vector2d rotated = r * p;
    const Eigen::Vector2d transformed = rotated + pose.position;
    double best = config.association_distance * config.association_distance;
    int nearest = -1;
    for (std::size_t j = 0; j < target.points.size(); ++j) {
      const double d2 = (transformed - target.points[j]).squaredNorm();
      if (d2 < best) {best = d2; nearest = static_cast<int>(j);}
    }
    if (nearest < 0) {continue;}
    const Eigen::Vector2d residual = transformed - target.points[nearest];
    const auto jacobian = pointJacobian(rotated);
    // icp_residual_begin
    if (config.metric == MatchMetric::PointToPoint) {
      const double norm = residual.norm();
      const double weight = norm <= config.huber_delta ? 1.0 : config.huber_delta / norm;
      system.h += weight * jacobian.transpose() * jacobian;
      system.g += weight * jacobian.transpose() * residual;
      system.squared_error += residual.squaredNorm();
    } else {
      const auto & normal = target.normals[nearest];
      if (normal.squaredNorm() < 0.5) {continue;}
      const double error = normal.dot(residual);
      const Eigen::RowVector3d j = normal.transpose() * jacobian;
      const double weight = std::abs(error) <= config.huber_delta ?
        1.0 : config.huber_delta / std::abs(error);
      system.h += weight * j.transpose() * j;
      system.g += weight * j.transpose() * error;
      system.squared_error += error * error;
    }
    ++system.count;
    // icp_residual_end
  }
  return system;
}
}  // namespace

MatchTarget makeMatchTarget(const PointCloud2D & points, double radius)
{
  validatePoints(points);
  if (!std::isfinite(radius) || radius <= 0) {
    throw std::invalid_argument("normal radius must be positive");
  }
  MatchTarget result{points, PointCloud2D(points.size(), Eigen::Vector2d::Zero())};
  for (std::size_t i = 0; i < points.size(); ++i) {
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    Eigen::Matrix2d second = Eigen::Matrix2d::Zero();
    int count = 0;
    for (const auto & p : points) {
      const Eigen::Vector2d d = p - points[i];
      if (d.squaredNorm() > radius * radius) {continue;}
      sum += d;
      second += d * d.transpose();
      ++count;
    }
    if (count < 5) {continue;}
    const Eigen::Vector2d mean = sum / count;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen(second / count - mean * mean.transpose());
    if (eigen.eigenvalues()[1] > 1e-8 &&
      eigen.eigenvalues()[0] < .15 * eigen.eigenvalues()[1])
    {
      result.normals[i] = eigen.eigenvectors().col(0);
    }
  }
  return result;
}

Eigen::Matrix<double, 2, 3> pointJacobian(const Eigen::Vector2d & rotated)
{
  // icp_jacobian_begin
  Eigen::Matrix<double, 2, 3> j;
  j << 1, 0, -rotated.y(),
       0, 1,  rotated.x();
  return j;
  // icp_jacobian_end
}

MatchResult matchClouds(const PointCloud2D & source, const MatchTarget & target,
  const Pose2D & initial, const MatchConfig & c)
{
  validatePoints(source);
  validatePoints(target.points);
  if (!initial.position.allFinite() || !std::isfinite(initial.yaw) ||
    c.max_iterations < 1 || c.min_pairs < 3 ||
    !std::isfinite(c.association_distance) || c.association_distance <= 0 ||
    !std::isfinite(c.huber_delta) || c.huber_delta <= 0 ||
    !std::isfinite(c.max_rmse) || c.max_rmse <= 0 ||
    !std::isfinite(c.translation_tolerance) || c.translation_tolerance <= 0 ||
    !std::isfinite(c.yaw_tolerance) || c.yaw_tolerance <= 0 ||
    !std::isfinite(c.min_eigen_ratio) || c.min_eigen_ratio <= 0 || c.min_eigen_ratio >= 1)
  {
    throw std::invalid_argument("invalid ICP initial pose/configuration");
  }
  if (c.metric == MatchMetric::PointToLine) {
    if (target.normals.size() != target.points.size()) {
      throw std::invalid_argument("point-to-line ICP requires target normals");
    }
    validatePoints(target.normals);
    for (const auto & n : target.normals) {
      if (n.squaredNorm() > 1e-12 && std::abs(n.norm() - 1) > 1e-6) {
        throw std::invalid_argument("target normals must be unit vectors or zero");
      }
    }
  }
  MatchResult result;
  result.pose = initial;
  bool small_step = false;
  for (int k = 0; k <= c.max_iterations; ++k) {
    const auto system = linearize(source, target, result.pose, c);
    result.pairs = system.count;
    result.information = system.h;
    if (system.count < c.min_pairs) {result.status = MatchStatus::TooFewPairs; return result;}
    result.rmse = std::sqrt(system.squared_error / system.count);
    // icp_solve_begin
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(system.h);
    if (eigen.eigenvalues()[0] <= c.min_eigen_ratio * eigen.eigenvalues()[2]) {
      result.status = MatchStatus::Degenerate;
      return result;
    }
    if (small_step) {
      result.status = result.rmse <= c.max_rmse ?
        MatchStatus::Converged : MatchStatus::HighResidual;
      return result;
    }
    if (k == c.max_iterations) {break;}
    Eigen::Vector3d delta = -system.h.ldlt().solve(system.g);
    // Keep each local update moderate; this does not rescue a bad initial guess.
    const double scale = std::max({1.0, delta.head<2>().norm() / .3, std::abs(delta.z()) / .2});
    delta /= scale;
    result.pose.position += delta.head<2>();
    result.pose.yaw = wrapAngle(result.pose.yaw + delta.z());
    ++result.iterations;
    small_step = delta.head<2>().norm() < c.translation_tolerance &&
      std::abs(delta.z()) < c.yaw_tolerance;
    // icp_solve_end
  }
  result.status = MatchStatus::IterationLimit;
  return result;
}

std::string matchStatusName(MatchStatus status)
{
  switch (status) {
    case MatchStatus::Converged: return "converged";
    case MatchStatus::TooFewPairs: return "too_few_pairs";
    case MatchStatus::Degenerate: return "degenerate";
    case MatchStatus::IterationLimit: return "iteration_limit";
    case MatchStatus::HighResidual: return "high_residual";
  }
  return "unknown";
}
}  // namespace motion2d
