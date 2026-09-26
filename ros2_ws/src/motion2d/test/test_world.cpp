#include <gtest/gtest.h>
#include <stdexcept>
#include "motion2d/sim/world.hpp"

using namespace motion2d;

TEST(Geometry, SegmentInteriorEndpointAndDegenerate)
{
  EXPECT_DOUBLE_EQ(pointSegmentDistance({1, 1}, {0, 0}, {2, 0}), 1.0);
  EXPECT_DOUBLE_EQ(pointSegmentDistance({3, 0}, {0, 0}, {2, 0}), 1.0);
  EXPECT_DOUBLE_EQ(pointSegmentDistance({0, 2}, {0, 0}, {0, 0}), 2.0);
}

TEST(Geometry, HullAndSignedDistance)
{
  const auto square = convexHull({{0, 0}, {1, 1}, {1, 0}, {0, 1}, {.5, .5}, {0, 0}});
  ASSERT_TRUE(isValid(square));
  ASSERT_EQ(square.vertices.size(), 4U);
  EXPECT_NEAR(signedDistance(square, {.5, .5}), -.5, 1e-12);
  EXPECT_NEAR(signedDistance(square, {2, .5}), 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(signedDistance(square, {0, .5}), 0.0);
  EXPECT_FALSE(isValid({{{0, 0}, {0, 1}, {1, 1}, {1, 0}}}));  // Clockwise.
  EXPECT_FALSE(isValid(convexHull({{0, 0}, {1, 0}, {2, 0}})));
  EXPECT_FALSE(isValid({{{0, 0}, {1, 1}, {0, 1}, {1, 0}}}));  // Crossing.
}

TEST(World, DiskRadiusAndContact)
{
  World2D world;
  world.circles.push_back({{0, 0}, 1.0});
  EXPECT_TRUE(isFree(world, {1.5, 0}, .4));
  EXPECT_FALSE(isFree(world, {1.5, 0}, .5));  // Tangent.
  EXPECT_FALSE(isFree(world, {0, 0}, 0));     // Point is inside.
  EXPECT_FALSE(isFree(world, {9.8, 0}, .3));  // World boundary.
}

TEST(World, ConservativeReachability)
{
  World2D world;
  world.width = world.height = 10;
  world.start = {-4, 0};
  world.goal = {4, 0};
  EXPECT_TRUE(isReachable(world, .2, .25));
  world.polygons.push_back({{{-.2, -5}, {.2, -5}, {.2, 5}, {-.2, 5}}});
  EXPECT_FALSE(isReachable(world, .2, .25));
}

TEST(World, ReproducibleSeedAndValidPolygons)
{
  WorldConfig config;
  const auto a = generateWorld(config), b = generateWorld(config);
  ASSERT_EQ(a.circles.size(), 8U);
  ASSERT_EQ(a.polygons.size(), 8U);
  for (std::size_t i = 0; i < a.circles.size(); ++i) {
    EXPECT_EQ(a.circles[i].center, b.circles[i].center);
    EXPECT_DOUBLE_EQ(a.circles[i].radius, b.circles[i].radius);
  }
  for (std::size_t i = 0; i < a.polygons.size(); ++i) {
    EXPECT_TRUE(isValid(a.polygons[i]));
    EXPECT_EQ(a.polygons[i].vertices, b.polygons[i].vertices);
  }
  EXPECT_TRUE(isFree(a, a.start, .25));
  EXPECT_TRUE(isFree(a, a.goal, .25));
  config.seed = 43;
  config.require_connected = false;
  EXPECT_GT((generateWorld(config).circles[0].center - a.circles[0].center).norm(), .01);
}

TEST(World, RejectBadConfiguration)
{
  WorldConfig config;
  config.start = {20, 0};
  EXPECT_THROW(generateWorld(config), std::invalid_argument);
  config = WorldConfig{};
  config.size_min = -1;
  EXPECT_THROW(generateWorld(config), std::invalid_argument);
  config = WorldConfig{};
  config.polygon_samples = 2;
  EXPECT_THROW(generateWorld(config), std::invalid_argument);
}
