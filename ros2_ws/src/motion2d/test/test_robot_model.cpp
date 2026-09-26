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

TEST(VelocityModel, WorldVelocityAndYawWrap)
{
  State2D state;
  state.pose = {{1, 2}, kPi - .1};
  const auto next = stepVelocity(state, {{-1, 2}, 1, false}, .2);
  EXPECT_NEAR(next.pose.position.x(), .8, 1e-12);
  EXPECT_NEAR(next.pose.position.y(), 2.4, 1e-12);
  EXPECT_NEAR(next.pose.yaw, -kPi + .1, 1e-12);
}

TEST(VelocityModel, BodyArcAndStraightLimit)
{
  State2D state;
  const auto arc = stepVelocity(state, {{1, 0}, kPi / 2.0, true}, 1.0);
  EXPECT_NEAR(arc.pose.position.x(), 2.0 / kPi, 1e-12);
  EXPECT_NEAR(arc.pose.position.y(), 2.0 / kPi, 1e-12);
  EXPECT_NEAR(arc.velocity.x(), 0, 1e-12);
  EXPECT_NEAR(arc.velocity.y(), 1, 1e-12);
  state.pose.yaw = kPi / 2.0;
  const auto straight = stepVelocity(state, {{1, 0}, 1e-12, true}, .1);
  EXPECT_NEAR(straight.pose.position.x(), 0, 1e-12);
  EXPECT_NEAR(straight.pose.position.y(), .1, 1e-12);
}

TEST(VelocityModel, DirectionPreservingLimits)
{
  const auto command = limitVelocity({{3, 4}, -2, true}, 1.0, .5);
  EXPECT_NEAR(command.velocity.x(), .6, 1e-12);
  EXPECT_NEAR(command.velocity.y(), .8, 1e-12);
  EXPECT_DOUBLE_EQ(command.yaw_rate, -.5);
  EXPECT_TRUE(command.body_frame);
}

TEST(VelocityModel, PaddingCoversObstacleOnArcOffChord)
{
  State2D state;
  const VelocityCommand command{{1, 0}, kPi / 2.0, true};
  const auto next = stepVelocity(state, command, 1.0);
  World2D world;
  world.circles.push_back({stepVelocity(state, command, .5).pose.position, .01});
  EXPECT_TRUE(sweptDiskIsFree(world, state.pose.position, next.pose.position, .02));
  EXPECT_FALSE(sweptDiskIsFree(world, state.pose.position, next.pose.position,
    .02 + velocitySweepPadding(command, 1.0)));
}
