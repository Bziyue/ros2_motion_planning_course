#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include "motion2d/estimation/pose_graph.hpp"

/** @brief Synthetic relative-pose experiment, isolating the graph solver from SLAM. */
int main(int argc, char ** argv)
{
  const std::filesystem::path output = argc > 1 ? argv[1] : "tmp/ch10_graph";
  std::filesystem::create_directories(output);
  std::vector<motion2d::Pose2D> truth(1), odometry(1);
  std::vector<motion2d::PoseGraphEdge> edges;
  std::mt19937 random(1010);
  std::normal_distribution<double> normal(0., 1.);
  const Eigen::Vector2d step[]{{.2, 0}, {0, .15}, {-.2, 0}, {0, -.15}};
  for (int side = 0; side < 4; ++side) {
    for (int k = 0; k < 20; ++k) {
      auto next = truth.back(); next.position += step[side]; truth.push_back(next);
      motion2d::Pose2D observation{step[side] + .01 * Eigen::Vector2d(normal(random), normal(random)),
        .003 + .002 * normal(random)};
      edges.push_back({odometry.size()-1, odometry.size(), observation, {.03, .03, .02}, false});
      odometry.push_back(motion2d::compose(odometry.back(), observation));
    }
  }
  std::ofstream summary(output / "ch10_graph.csv");
  summary << "mode,position_rmse,closure_error,initial_cost,final_cost,iterations,status\n";
  std::vector<motion2d::Pose2D> optimized;
  for (bool loop : {false, true}) {
    auto graph = edges;
    if (loop) {graph.push_back({0, odometry.size()-1, {}, {.02, .02, .01}, true});}
    const auto result = motion2d::optimizePoseGraph(odometry, graph);
    if (!result.converged) {std::cerr << result.status << '\n'; return 1;}
    double squared = 0;
    for (std::size_t i = 0; i < truth.size(); ++i) {squared += (result.poses[i].position-truth[i].position).squaredNorm();}
    summary << (loop ? "loop" : "odometry") << ',' << std::sqrt(squared/truth.size()) << ','
      << (result.poses.back().position-result.poses.front().position).norm() << ','
      << result.initial_cost << ',' << result.final_cost << ',' << result.iterations << ',' << result.status << '\n';
    if (loop) {optimized = result.poses;}
  }
  std::ofstream trace(output / "ch10_graph_trace.csv");
  trace << "index,truth_x,truth_y,odom_x,odom_y,optimized_x,optimized_y,odom_error,optimized_error\n";
  for (std::size_t i = 0; i < truth.size(); ++i) {
    trace << i << ',' << truth[i].position.x() << ',' << truth[i].position.y() << ','
      << odometry[i].position.x() << ',' << odometry[i].position.y() << ','
      << optimized[i].position.x() << ',' << optimized[i].position.y() << ','
      << (odometry[i].position-truth[i].position).norm() << ','
      << (optimized[i].position-truth[i].position).norm() << '\n';
  }
  std::cout << "Wrote synthetic graph experiment to " << output << '\n';
}
