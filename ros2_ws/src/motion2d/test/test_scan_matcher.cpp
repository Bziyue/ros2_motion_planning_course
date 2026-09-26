#include <gtest/gtest.h>
#include "motion2d/estimation/scan_matcher.hpp"
#include "motion2d/mapping/scan_projection.hpp"

using namespace motion2d;

namespace
{
PointCloud2D corner()
{
  PointCloud2D points;
  for (int i = 0; i < 81; ++i) {
    points.emplace_back(-2 + .05 * i, 2);
    points.emplace_back(2, -2 + .05 * i);
  }
  return points;
}
}

TEST(ScanMatcher, JacobianMatchesIndependentCentralDifference)
{
  const Pose2D pose{{.8, -1.3}, .7};
  const Eigen::Vector2d p{2.3, -.9};
  const auto analytic = pointJacobian(rotation(pose.yaw) * p);
  for (int j = 0; j < 3; ++j) {
    Pose2D a = pose, b = pose;
    if (j < 2) {a.position[j] += 1e-6; b.position[j] -= 1e-6;}
    else {a.yaw += 1e-6; b.yaw -= 1e-6;}
    EXPECT_LT((analytic.col(j) -
      (transformPoint(a, p) - transformPoint(b, p)) / 2e-6).norm(), 1e-8);
  }
}

TEST(ScanMatcher, RecoversRigidTransformFromNearbyInitialGuessWithBothResiduals)
{
  const auto target = makeMatchTarget(corner());
  const Pose2D truth{{.12, -.08}, .04};
  const auto source = registerPoints(target.points, inverse(truth));
  for (auto metric : {MatchMetric::PointToPoint, MatchMetric::PointToLine}) {
    MatchConfig config;
    config.metric = metric;
    const auto result = matchClouds(source, target, {{.11, -.075}, .038}, config);
    ASSERT_TRUE(result.accepted()) << matchStatusName(result.status);
    EXPECT_LT((result.pose.position - truth.position).norm(), 2e-4);
    EXPECT_NEAR(result.pose.yaw, truth.yaw, 2e-4);
  }
}

TEST(ScanMatcher, SmallIncrementWithLargeResidualIsNotAccepted)
{
  const auto target = makeMatchTarget(corner());
  const auto source = registerPoints(target.points, inverse({{.12, -.08}, .04}));
  MatchConfig config;
  config.max_rmse = 1e-4;
  EXPECT_EQ(matchClouds(source, target, {}, config).status, MatchStatus::HighResidual);
}

TEST(ScanMatcher, RejectsUnobservableParallelWallTranslation)
{
  PointCloud2D wall;
  for (int i = 0; i < 100; ++i) {wall.emplace_back(i * .03, 2);}
  MatchConfig config;
  config.metric = MatchMetric::PointToLine;
  EXPECT_EQ(matchClouds(wall, makeMatchTarget(wall), {}, config).status,
    MatchStatus::Degenerate);
}

TEST(ScanMatcher, RejectsEmptyNoOverlapAndIterationLimit)
{
  const auto target = makeMatchTarget(corner());
  EXPECT_EQ(matchClouds({}, target).status, MatchStatus::TooFewPairs);
  EXPECT_EQ(matchClouds(corner(), target, {{20, 20}, 0}).status, MatchStatus::TooFewPairs);
  MatchConfig config;
  config.max_iterations = 1;
  EXPECT_EQ(matchClouds(corner(), target, {{.1, 0}, .05}, config).status,
    MatchStatus::IterationLimit);
}

TEST(ScanMatcher, GatedOutliersDoNotMoveAnExactAlignment)
{
  auto source = corner();
  const auto target = makeMatchTarget(source);
  for (int i = 0; i < 50; ++i) {source.emplace_back(100 + i, -100);}
  const auto result = matchClouds(source, target);
  ASSERT_TRUE(result.accepted());
  EXPECT_EQ(result.pairs, static_cast<int>(target.points.size()));
  EXPECT_LT(result.pose.position.norm(), 1e-12);
}

TEST(ScanMatcher, RejectsMalformedInputs)
{
  MatchConfig config;
  config.huber_delta = 0;
  EXPECT_THROW(matchClouds({}, {}, {}, config), std::invalid_argument);
  EXPECT_THROW(makeMatchTarget({{NAN, 0}}), std::invalid_argument);
  config = MatchConfig{};
  config.metric = MatchMetric::PointToLine;
  EXPECT_THROW(matchClouds(corner(), {corner(), {}}, {}, config), std::invalid_argument);
}
