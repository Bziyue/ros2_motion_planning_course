#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include "motion2d/sim/raycast.hpp"
#include "motion2d/geometry/se2.hpp"

using namespace motion2d;

TEST(Raycast, CircleChoosesNearestForwardRoot)
{
  EXPECT_DOUBLE_EQ(rayCircle({0, 0}, {1, 0}, {{3, 0}, 1}), 2);
  EXPECT_TRUE(std::isinf(rayCircle({0, 0}, {-1, 0}, {{3, 0}, 1})));
  EXPECT_DOUBLE_EQ(rayCircle({3, 0}, {1, 0}, {{3, 0}, 1}), 1);
  EXPECT_DOUBLE_EQ(rayCircle({2, 0}, {-1, 0}, {{3, 0}, 1}), 0);
}

TEST(Raycast, CircleTangentAndNearMiss)
{
  EXPECT_DOUBLE_EQ(rayCircle({0, 0}, {1, 0}, {{3, 1}, 1}), 3);
  EXPECT_TRUE(std::isinf(rayCircle({0, 0}, {1, 0}, {{3, 1.001}, 1})));
  EXPECT_NEAR(rayCircle({0, 0}, {1, 0}, {{3, .6}, 1}), 2.2, 1e-12);
}

TEST(Raycast, SegmentCrossingEndpointAndBackwardMiss)
{
  EXPECT_DOUBLE_EQ(raySegment({0, 0}, {1, 0}, {4, -1}, {4, 1}), 4);
  EXPECT_DOUBLE_EQ(raySegment({0, 0}, {1, 0}, {4, 0}, {4, 2}), 4);
  EXPECT_DOUBLE_EQ(raySegment({0, 0}, {1, 0}, {4, 2}, {4, 0}), 4);
  EXPECT_TRUE(std::isinf(raySegment({0, 0}, {-1, 0}, {4, -1}, {4, 1})));
  EXPECT_TRUE(std::isinf(raySegment({0, 0}, {1, 0}, {4, 1}, {4, 2})));
}

TEST(Raycast, ParallelCollinearAndDegenerateSegments)
{
  EXPECT_TRUE(std::isinf(raySegment({0, 0}, {1, 0}, {1, 1}, {4, 1})));
  EXPECT_TRUE(std::isinf(raySegment({0, 0}, {1, 0}, {-4, 0}, {-1, 0})));
  EXPECT_DOUBLE_EQ(raySegment({0, 0}, {1, 0}, {4, 0}, {1, 0}), 1);
  EXPECT_DOUBLE_EQ(raySegment({0, 0}, {1, 0}, {-1, 0}, {4, 0}), 0);
  EXPECT_DOUBLE_EQ(raySegment({0, 0}, {1, 0}, {2, 0}, {2, 0}), 2);
  EXPECT_TRUE(std::isinf(raySegment({0, 0}, {1, 0}, {2, 1}, {2, 1})));
}

TEST(Raycast, RectangularBoundaryAndCorner)
{
  World2D world;
  EXPECT_DOUBLE_EQ(raycastWorld(world, {0, 0}, {1, 0}), 10);
  EXPECT_DOUBLE_EQ(raycastWorld(world, {-8, -8}, {0, -1}), 2);
  const Eigen::Vector2d diagonal = Eigen::Vector2d{1, 1}.normalized();
  EXPECT_NEAR(raycastWorld(world, {0, 0}, diagonal), std::sqrt(200.0), 1e-12);
}

TEST(Raycast, NearestSurfaceOccludesFartherObstacles)
{
  World2D world;
  world.circles = {{{6, 0}, 1}, {{3, 0}, .5}};
  world.polygons = {{{{4, -1}, {5, -1}, {5, 1}, {4, 1}}}};
  EXPECT_DOUBLE_EQ(raycastWorld(world, {0, 0}, {1, 0}), 2.5);
  std::reverse(world.circles.begin(), world.circles.end());
  EXPECT_DOUBLE_EQ(raycastWorld(world, {0, 0}, {1, 0}), 2.5);
  world.circles.clear();
  EXPECT_DOUBLE_EQ(raycastWorld(world, {0, 0}, {1, 0}), 4);
}

TEST(Raycast, RigidTransformPreservesDistance)
{
  const Eigen::Vector2d shift{1.2, -3.4};
  const auto r = rotation(.73);
  EXPECT_NEAR(rayCircle(shift, r * Eigen::Vector2d{1, 0},
    {shift + r * Eigen::Vector2d{3, 0}, 1}), 2, 1e-12);
  EXPECT_NEAR(raySegment(shift, r * Eigen::Vector2d{1, 0},
    shift + r * Eigen::Vector2d{4, -1}, shift + r * Eigen::Vector2d{4, 1}), 4, 1e-12);
}
