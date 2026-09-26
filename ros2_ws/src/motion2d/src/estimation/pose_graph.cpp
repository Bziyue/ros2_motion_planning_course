#include "motion2d/estimation/pose_graph.hpp"
#include <Eigen/SparseCholesky>
#include <cmath>
#include <queue>
#include <stdexcept>

namespace motion2d
{
// graph_residual_begin
GraphLinearization linearizeGraphEdge(const Pose2D & from, const Pose2D & to,
  const Pose2D & measured)
{
  const Eigen::Matrix2d ri = rotation(from.yaw).transpose();
  const Eigen::Matrix2d rz = rotation(measured.yaw).transpose();
  const Eigen::Vector2d q = ri * (to.position - from.position);
  GraphLinearization result;
  result.residual.head<2>() = rz * (q - measured.position);
  result.residual.z() = wrapAngle(to.yaw - from.yaw - measured.yaw);
  result.from.setZero(); result.to.setZero();
  result.from.topLeftCorner<2, 2>() = -rz * ri;
  result.from.topRightCorner<2, 1>() = rz * Eigen::Vector2d(q.y(), -q.x());
  result.from(2, 2) = -1;
  result.to.topLeftCorner<2, 2>() = rz * ri;
  result.to(2, 2) = 1;
  return result;
}
// graph_residual_end

namespace
{
void validate(const std::vector<Pose2D> & poses, const std::vector<PoseGraphEdge> & edges,
  double delta)
{
  if (poses.empty() || !std::isfinite(delta) || delta <= 0) {
    throw std::invalid_argument("pose graph needs a pose and positive Huber scale");
  }
  for (const auto & p : poses) {
    if (!p.position.allFinite() || !std::isfinite(p.yaw)) {throw std::invalid_argument("nonfinite graph pose");}
  }
  for (const auto & e : edges) {
    if (e.from >= poses.size() || e.to >= poses.size() || e.from == e.to ||
      !e.relative.position.allFinite() || !std::isfinite(e.relative.yaw) ||
      !e.stddev.allFinite() || e.stddev.minCoeff() <= 0) {
      throw std::invalid_argument("invalid graph edge");
    }
  }
}

bool connected(std::size_t size, const std::vector<PoseGraphEdge> & edges)
{
  std::vector<std::vector<std::size_t>> adjacent(size);
  for (const auto & e : edges) {adjacent[e.from].push_back(e.to); adjacent[e.to].push_back(e.from);}
  std::vector<bool> seen(size, false); seen[0] = true;
  std::queue<std::size_t> queue; queue.push(0);
  while (!queue.empty()) {
    const auto i = queue.front(); queue.pop();
    for (auto j : adjacent[i]) {if (!seen[j]) {seen[j] = true; queue.push(j);}}
  }
  return std::all_of(seen.begin(), seen.end(), [](bool value) {return value;});
}

double cost(const std::vector<Pose2D> & poses, const std::vector<PoseGraphEdge> & edges, double delta)
{
  double total = 0;
  for (const auto & e : edges) {
    const double norm = linearizeGraphEdge(poses[e.from], poses[e.to], e.relative)
      .residual.cwiseQuotient(e.stddev).norm();
    total += e.loop && norm > delta ? delta * (norm - .5 * delta) : .5 * norm * norm;
  }
  return total;
}
}  // namespace

double poseGraphCost(const std::vector<Pose2D> & poses, const std::vector<PoseGraphEdge> & edges,
  double loop_huber)
{
  validate(poses, edges, loop_huber);
  return cost(poses, edges, loop_huber);
}

PoseGraphResult optimizePoseGraph(const std::vector<Pose2D> & initial,
  const std::vector<PoseGraphEdge> & edges, const PoseGraphConfig & config)
{
  validate(initial, edges, config.loop_huber);
  if (config.max_iterations < 1 || !std::isfinite(config.step_tolerance) || config.step_tolerance <= 0) {
    throw std::invalid_argument("invalid pose graph solve settings");
  }
  PoseGraphResult result; result.poses = initial;
  result.initial_cost = result.final_cost = cost(initial, edges, config.loop_huber);
  if (!connected(initial.size(), edges)) {result.status = "disconnected"; return result;}
  const int dimension = 3 * static_cast<int>(initial.size() - 1);
  if (dimension == 0) {result.converged = true; result.status = "converged"; return result;}
  for (int iteration = 0; iteration < config.max_iterations; ++iteration) {
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dimension);
    std::vector<Eigen::Triplet<double>> entries;
    for (const auto & e : edges) {
      const auto linear = linearizeGraphEdge(result.poses[e.from], result.poses[e.to], e.relative);
      const Eigen::Matrix3d whiten = e.stddev.cwiseInverse().asDiagonal();
      const Eigen::Vector3d residual = whiten * linear.residual;
      const double weight = e.loop && residual.norm() > config.loop_huber ?
        config.loop_huber / residual.norm() : 1.;
      const Eigen::Matrix3d jacobian[]{whiten * linear.from, whiten * linear.to};
      const std::size_t indices[]{e.from, e.to};
      // graph_assembly_begin
      for (int a = 0; a < 2; ++a) {
        if (indices[a] == 0) {continue;}  // Fixed anchor has no optimization variables.
        const int row = 3 * (indices[a] - 1);
        gradient.segment<3>(row) += weight * jacobian[a].transpose() * residual;
        for (int b = 0; b < 2; ++b) {
          if (indices[b] == 0) {continue;}
          const int column = 3 * (indices[b] - 1);
          const Eigen::Matrix3d block = weight * jacobian[a].transpose() * jacobian[b];
          for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {entries.emplace_back(row + i, column + j, block(i, j));}
          }
        }
      }
      // graph_assembly_end
    }
    Eigen::SparseMatrix<double> hessian(dimension, dimension);
    hessian.setFromTriplets(entries.begin(), entries.end());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(hessian);
    if (solver.info() != Eigen::Success || solver.vectorD().minCoeff() <= 0) {
      result.status = "singular"; return result;
    }
    const Eigen::VectorXd step = solver.solve(-gradient);
    if (solver.info() != Eigen::Success || !step.allFinite()) {result.status = "solve_failed"; return result;}
    result.iterations = iteration + 1;
    if (step.lpNorm<Eigen::Infinity>() < config.step_tolerance) {
      result.converged = true; result.status = "converged"; return result;
    }
    bool accepted = false;
    for (double alpha = 1.; alpha >= 1. / 1024; alpha *= .5) {
      auto candidate = result.poses;
      for (std::size_t k = 1; k < candidate.size(); ++k) {
        candidate[k].position += alpha * step.segment<2>(3 * (k - 1));
        candidate[k].yaw = wrapAngle(candidate[k].yaw + alpha * step[3 * (k - 1) + 2]);
      }
      const double candidate_cost = cost(candidate, edges, config.loop_huber);
      if (candidate_cost <= result.final_cost + 1e-4 * alpha * gradient.dot(step)) {
        result.poses = std::move(candidate); result.final_cost = candidate_cost; accepted = true; break;
      }
    }
    if (!accepted) {result.status = "line_search_failed"; return result;}
  }
  return result;
}
}  // namespace motion2d
