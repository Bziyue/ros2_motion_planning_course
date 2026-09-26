#include <gtest/gtest.h>
#include "motion2d/sim/robot_model.hpp"
#include "motion2d/sim/world.hpp"

using namespace motion2d;

TEST(IdealModel, PoseAndUnusedDerivatives)
{
  const auto state = idealPose({{2.0, 3.0}, 3.0 * kPi});
  EXPECT_EQ(state.pose.position, Eigen::Vector2d(2.0, 3.0));
  EXPECT_NEAR(state.pose.yaw, -kPi, 1e-12);
  EXPECT_EQ(state.velocity, Eigen::Vector2d::Zero());
  EXPECT_EQ(state.acceleration, Eigen::Vector2d::Zero());
}

TEST(SweptDisk, CannotJumpOverCircle)
{
  World2D world;
  world.circles.push_back({{0, 0}, 0.5});
  EXPECT_TRUE(isFree(world, {-2, 0}, .2));
  EXPECT_TRUE(isFree(world, {2, 0}, .2));
  EXPECT_FALSE(sweptDiskIsFree(world, {-2, 0}, {2, 0}, .2));
  EXPECT_FALSE(sweptDiskIsFree(world, {-2, .75}, {2, .75}, .25));  // Contact.
  EXPECT_TRUE(sweptDiskIsFree(world, {-2, .76}, {2, .76}, .25));
}

TEST(SweptDisk, ThinPolygonAndPointRobot)
{
  World2D world;
  world.polygons.push_back({{{-.01, -1}, {.01, -1}, {.01, 1}, {-.01, 1}}});
  EXPECT_FALSE(sweptDiskIsFree(world, {-2, 0}, {2, 0}, 0));
  EXPECT_FALSE(sweptDiskIsFree(world, {-2, 1.125}, {2, 1.125}, .125));
  EXPECT_TRUE(sweptDiskIsFree(world, {-2, 1.2}, {2, 1.2}, .125));
  EXPECT_FALSE(sweptDiskIsFree(world, {0, 0}, {0, .5}, 0));
}

TEST(SweptDisk, StationaryAndRectangleBoundary)
{
  World2D world;
  EXPECT_TRUE(sweptDiskIsFree(world, {1, 2}, {1, 2}, .2));
  EXPECT_FALSE(sweptDiskIsFree(world, {0, 0}, {9.875, 0}, .125));
  world.polygons.push_back({{{0, 0}, {1, 0}, {1, 1}, {0, 1}}});
  EXPECT_TRUE(sweptDiskIsFree(world, {2, 0}, {3, 0}, 0));  // Disjoint collinear edges.
  EXPECT_FALSE(sweptDiskIsFree(world, {1, 0}, {2, 0}, 0));
}
