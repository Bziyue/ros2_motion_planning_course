#include <gtest/gtest.h>
#include <cmath>
#include "motion2d/evaluation/map_evaluation.hpp"

using namespace motion2d;

TEST(MapEvaluation, CircleUsesCellFootprintNotOnlyCentre)
{
  GridConfig cfg;
  cfg.resolution = 1; cfg.width = cfg.height = 5; cfg.origin = {-2.5, -2.5};
  World2D world;
  world.circles.push_back({{.6, .6}, .2});
  const auto truth = rasterizeTruth(world, cfg);
  EXPECT_EQ(truth[12], 1);  // Cell centre (0,0) is outside, but the square intersects.
  EXPECT_EQ(truth[13], 1); EXPECT_EQ(truth[17], 1); EXPECT_EQ(truth[18], 1);
  EXPECT_EQ(std::count(truth.begin(), truth.end(), 1), 4);
}

TEST(MapEvaluation, PolygonSeparatingAxisAndRoomBoundary)
{
  GridConfig cfg;
  cfg.resolution = 1; cfg.width = cfg.height = 4; cfg.origin = {0, 0};
  World2D world;
  world.polygons.push_back({{{0, 0}, {2, 0}, {0, 2}}});
  const auto truth = rasterizeTruth(world, cfg);
  EXPECT_EQ(truth[0], 1);
  EXPECT_EQ(truth[5], 1);  // Touches triangle at (1,1).
  EXPECT_EQ(truth[6], 0);  // AABBs touch, but the triangle's diagonal separates.
  world.polygons.clear();
  world.width = world.height = 4;
  const auto walls = rasterizeTruth(world, cfg);
  EXPECT_EQ(walls[0], 0);
  EXPECT_EQ(walls[1], 1);  // Cell's upper x touches room wall.
  EXPECT_EQ(walls[15], 1); // Outside world is solid in reference only.
}

TEST(MapEvaluation, UnknownAndUncertainAreExcludedFromConfusion)
{
  const auto metrics = evaluateObserved({-1, 50, 70, 70, 20, 20}, {1, 1, 1, 0, 1, 0});
  EXPECT_EQ(metrics.observed, 5); EXPECT_EQ(metrics.uncertain, 1);
  EXPECT_EQ(metrics.true_positive, 1); EXPECT_EQ(metrics.false_positive, 1);
  EXPECT_EQ(metrics.false_negative, 1); EXPECT_EQ(metrics.true_negative, 1);
  EXPECT_DOUBLE_EQ(metrics.precision(), .5); EXPECT_DOUBLE_EQ(metrics.recall(), .5);
  const auto empty = evaluateObserved({-1, 50}, {0, 1});
  EXPECT_TRUE(std::isnan(empty.precision())); EXPECT_TRUE(std::isnan(empty.recall()));
}

TEST(MapEvaluation, RejectsMismatchedOrInvalidEncoding)
{
  EXPECT_THROW(evaluateObserved({-1}, {}), std::invalid_argument);
  EXPECT_THROW(evaluateObserved({-2}, {0}), std::invalid_argument);
  EXPECT_THROW(evaluateObserved({70}, {2}), std::invalid_argument);
}
