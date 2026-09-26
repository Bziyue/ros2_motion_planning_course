#include <gtest/gtest.h>
#include "motion2d/estimation/pose_graph.hpp"
using namespace motion2d;

TEST(PoseGraph, ResidualMatchesCompositionAndJacobianFiniteDifference)
{
  Pose2D a{{1.1, -.3}, .8}, b{{-2.3, 4.2}, -1.1}, z{{.2, .1}, -.4};
  const auto linear = linearizeGraphEdge(a, b, z);
  const auto composed = compose(inverse(z), compose(inverse(a), b));
  EXPECT_LT((linear.residual.head<2>() - composed.position).norm(), 1e-12);
  EXPECT_NEAR(linear.residual.z(), composed.yaw, 1e-12);
  for (int which = 0; which < 2; ++which) {
    for (int axis = 0; axis < 3; ++axis) {
      Pose2D plus[]{a, b}, minus[]{a, b};
      if (axis < 2) {plus[which].position[axis] += 1e-6; minus[which].position[axis] -= 1e-6;}
      else {plus[which].yaw += 1e-6; minus[which].yaw -= 1e-6;}
      const Eigen::Vector3d fd = (linearizeGraphEdge(plus[0], plus[1], z).residual -
        linearizeGraphEdge(minus[0], minus[1], z).residual) / 2e-6;
      EXPECT_LT((fd - (which == 0 ? linear.from : linear.to).col(axis)).norm(), 2e-9);
    }
  }
}

TEST(PoseGraph, KnownSquareRecoversWithFixedNonzeroAnchor)
{
  std::vector<Pose2D> truth{{{2, -1}, .3}, {{3, -1}, .3}, {{3, 0}, .5}, {{2, 0}, -.2}};
  std::vector<PoseGraphEdge> edges;
  for (std::size_t k = 0; k < 4; ++k) {
    const auto j = (k + 1) % 4;
    edges.push_back({k, j, compose(inverse(truth[k]), truth[j]), {.03, .03, .02}, k == 3});
  }
  auto initial = truth;
  for (std::size_t k = 1; k < 4; ++k) {initial[k].position += Eigen::Vector2d(.1*k, -.04*k); initial[k].yaw += .03*k;}
  const auto result = optimizePoseGraph(initial, edges);
  ASSERT_TRUE(result.converged) << result.status;
  EXPECT_EQ((result.poses[0].position - truth[0].position).norm(), 0.);
  EXPECT_EQ(result.poses[0].yaw, truth[0].yaw);
  for (std::size_t k = 0; k < 4; ++k) {
    EXPECT_LT((result.poses[k].position - truth[k].position).norm(), 1e-8);
    EXPECT_NEAR(result.poses[k].yaw, truth[k].yaw, 1e-8);
  }
  EXPECT_LT(result.final_cost, 1e-14);
}

TEST(PoseGraph, WeightedOneDimensionalProblemMatchesAnalyticSolution)
{
  std::vector<Pose2D> nodes{{}, {{4, 0}, 0}};
  std::vector<PoseGraphEdge> edges{{0, 1, {{1, 0}, 0}, {1, 1, 1}, false},
    {0, 1, {{3, 0}, 0}, {2, 1, 1}, false}};
  const auto result = optimizePoseGraph(nodes, edges);
  ASSERT_TRUE(result.converged);
  EXPECT_NEAR(result.poses[1].position.x(), 1.4, 1e-12);
  EXPECT_NEAR(result.final_cost, .4, 1e-12);
}

TEST(PoseGraph, RobustLossLimitsButDoesNotCertifyFalseLoop)
{
  std::vector<Pose2D> nodes{{}, {{1, 0}, 0}};
  std::vector<PoseGraphEdge> edges{{0, 1, {{1, 0}, 0}, {.1, .1, .1}, false},
    {0, 1, {{10, 0}, 0}, {.1, .1, .1}, true}};
  const auto result = optimizePoseGraph(nodes, edges);
  ASSERT_TRUE(result.converged) << result.status;
  // With Huber delta=3, even this rejected-looking loop pulls the estimate by .3 m.
  EXPECT_NEAR(result.poses[1].position.x(), 1.3, 1e-7);
  auto plain = optimizePoseGraph(nodes, edges, {30, 1000, 1e-7});
  ASSERT_TRUE(plain.converged);
  EXPECT_NEAR(plain.poses[1].position.x(), 5.5, 1e-12);
}

TEST(PoseGraph, DisconnectedAndInvalidEdgesAreExplicit)
{
  std::vector<Pose2D> nodes(3);
  auto result = optimizePoseGraph(nodes, {{0, 1, {}, {.1, .1, .1}, false}});
  EXPECT_FALSE(result.converged); EXPECT_EQ(result.status, "disconnected");
  EXPECT_THROW(optimizePoseGraph(nodes, {{0, 3, {}}}), std::invalid_argument);
  EXPECT_THROW(optimizePoseGraph(nodes, {{0, 1, {}, {0, 1, 1}}}), std::invalid_argument);
}
