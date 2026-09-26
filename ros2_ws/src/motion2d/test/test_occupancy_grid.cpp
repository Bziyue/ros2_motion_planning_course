#include <gtest/gtest.h>
#include <limits>
#include <random>
#include <set>
#include "motion2d/mapping/occupancy_grid.hpp"

using namespace motion2d;
namespace
{
GridConfig smallGrid()
{
  GridConfig config;
  config.resolution = 1;
  config.width = config.height = 5;
  config.origin = {0, 0};
  return config;
}
}

TEST(OccupancyGrid, HalfOpenCellsAndNegativeOrigin)
{
  auto config = smallGrid();
  config.origin = {-2, -3};
  OccupancyGrid2D grid(config);
  EXPECT_EQ(grid.cellIndex({-2, -3}), 0);
  EXPECT_EQ(grid.cellIndex({-1, -2}), 6);
  EXPECT_EQ(grid.cellIndex({2.99, 1.99}), 24);
  EXPECT_FALSE(grid.cellIndex({-2.01, -3}));
  EXPECT_FALSE(grid.cellIndex({3, 0}));
  EXPECT_FALSE(grid.cellIndex({0, 2}));
  EXPECT_EQ(grid.cellCenter(6), Eigen::Vector2d(-.5, -1.5));
  for (auto value : grid.occupancy()) {EXPECT_EQ(value, -1);}
}

TEST(OccupancyGrid, DdaAxesDiagonalsClippingAndCorners)
{
  OccupancyGrid2D grid(smallGrid());
  EXPECT_EQ(grid.rayCells({.5, .5}, {3.5, .5}), (std::vector<int>{0, 1, 2, 3}));
  EXPECT_EQ(grid.rayCells({.5, .5}, {.5, 3.5}), (std::vector<int>{0, 5, 10, 15}));
  EXPECT_EQ(grid.rayCells({.5, .5}, {3.5, 3.5}), (std::vector<int>{0, 6, 12, 18}));
  EXPECT_EQ(grid.rayCells({3.5, 3.5}, {.5, .5}), (std::vector<int>{18, 12, 6, 0}));
  EXPECT_EQ(grid.rayCells({.5, .5}, {20, .5}), (std::vector<int>{0, 1, 2, 3, 4}));
  EXPECT_EQ(grid.rayCells({.5, .5}, {5, .5}), (std::vector<int>{0, 1, 2, 3, 4}));
  EXPECT_EQ(grid.rayCells({.5, .5}, {-20, .5}), (std::vector<int>{0}));
  EXPECT_EQ(grid.rayCells({1, .5}, {.5, .5}), (std::vector<int>{1, 0}));
  EXPECT_EQ(grid.rayCells({.5, .5}, {.6, .5}), (std::vector<int>{0}));
  EXPECT_EQ(grid.rayCells({.5, 1.5}, {1, 1}), (std::vector<int>{5, 6}));
  EXPECT_EQ(grid.rayCells({1.5, .5}, {1, 1}), (std::vector<int>{1, 6}));
  EXPECT_TRUE(grid.rayCells({-1, 0}, {1, 0}).empty());
}

TEST(OccupancyGrid, HitClearsBeforeSurfaceAndOcclusionStaysUnknown)
{
  OccupancyGrid2D grid(smallGrid());
  EXPECT_TRUE(grid.insertScan({0, .1, .05, 10, {2.0F}}, {{.5, .5}, 0}));
  const auto cells = grid.occupancy();
  EXPECT_EQ(cells[0], 40); EXPECT_EQ(cells[1], 40); EXPECT_EQ(cells[2], 70);
  EXPECT_EQ(cells[3], -1); EXPECT_EQ(cells[4], -1);
  EXPECT_EQ(cells[5], -1);
}

TEST(OccupancyGrid, DdaAgreesWithIndependentSegmentBoxIntersections)
{
  OccupancyGrid2D grid(smallGrid());
  std::mt19937 random(707);
  std::uniform_real_distribution<double> inside(.001, 4.999), outside(-3, 8);
  for (int trial = 0; trial < 1000; ++trial) {
    const Eigen::Vector2d start{inside(random), inside(random)};
    const Eigen::Vector2d end{outside(random), outside(random)};
    const Eigen::Vector2d delta = end - start;
    std::set<int> expected;
    // Independent reference: intersect the segment with each closed cell box.
    // Random non-boundary rays avoid ambiguous measure-zero corner contacts.
    for (int y = 0; y < 5; ++y) {
      for (int x = 0; x < 5; ++x) {
        double enter = 0, leave = 1;
        const Eigen::Vector2d lower(x, y);
        for (int axis = 0; axis < 2; ++axis) {
          double a = (lower[axis] - start[axis]) / delta[axis];
          double b = (lower[axis] + 1 - start[axis]) / delta[axis];
          if (a > b) {std::swap(a, b);}
          enter = std::max(enter, a);
          leave = std::min(leave, b);
        }
        if (leave - enter > 1e-10) {expected.insert(y * 5 + x);}
      }
    }
    const auto cells = grid.rayCells(start, end);
    EXPECT_EQ(std::set<int>(cells.begin(), cells.end()), expected);
    EXPECT_EQ(cells.size(), expected.size());
  }
}

TEST(OccupancyGrid, NoReturnClearsOnlyRangeAndInvalidDoesNothing)
{
  OccupancyGrid2D grid(smallGrid());
  const float inf = std::numeric_limits<float>::infinity();
  grid.insertScan({0, .1, .05, 2, {inf}}, {{.5, .5}, 0});
  const auto cells = grid.occupancy();
  EXPECT_EQ(cells[0], 40); EXPECT_EQ(cells[1], 40); EXPECT_EQ(cells[2], 40);
  EXPECT_EQ(cells[3], -1);
  grid.insertScan({0, .1, .05, 2, {std::numeric_limits<float>::quiet_NaN(), -inf, .01F, 3}},
    {{.5, .5}, 0});
  EXPECT_EQ(grid.occupancy(), cells);
}

TEST(OccupancyGrid, OutsideHitDoesNotInventBoundaryObstacle)
{
  OccupancyGrid2D grid(smallGrid());
  grid.insertScan({0, .1, .05, 10, {8}}, {{.5, .5}, 0});
  EXPECT_EQ(grid.occupancy()[4], 40);
  const auto before = grid.occupancy();
  EXPECT_FALSE(grid.insertScan({0, .1, .05, 10, {2}}, {{-1, 0}, 0}));
  EXPECT_EQ(grid.occupancy(), before);
}

TEST(OccupancyGrid, OncePerScanAndHitHasPriority)
{
  OccupancyGrid2D grid(smallGrid());
  // These rays share cells: the short endpoint must survive the longer free ray.
  grid.insertScan({0, .001, .05, 10, {2, 3, 3, 3}}, {{.5, .5}, 0});
  const auto cells = grid.occupancy();
  EXPECT_EQ(cells[0], 40); EXPECT_EQ(cells[1], 40);
  EXPECT_EQ(cells[2], 70); EXPECT_EQ(cells[3], 70);
}

TEST(OccupancyGrid, SaturationAndClear)
{
  OccupancyGrid2D grid(smallGrid());
  for (int i = 0; i < 100; ++i) {grid.insertScan({0, .1, .05, 10, {2}}, {{.5, .5}, 0});}
  EXPECT_EQ(grid.occupancy()[0], 2);
  EXPECT_EQ(grid.occupancy()[2], 98);
  grid.clear();
  for (auto value : grid.occupancy()) {EXPECT_EQ(value, -1);}
}

TEST(OccupancyGrid, RejectsInvalidConfiguration)
{
  auto config = smallGrid();
  config.resolution = 0;
  EXPECT_THROW(OccupancyGrid2D{config}, std::invalid_argument);
  config = smallGrid(); config.hit_probability = .4;
  EXPECT_THROW(OccupancyGrid2D{config}, std::invalid_argument);
  config = smallGrid(); config.width = config.height = 100000;
  EXPECT_THROW(OccupancyGrid2D{config}, std::invalid_argument);
}
