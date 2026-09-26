#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>
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

TEST(InertialModel, ConstantForceAndTorque)
{
  InertialParameters p;
  p.mass = 2;
  p.inertia_z = .05;
  State2D state;
  state.pose = {{1, 2}, .2};
  state.velocity = {.3, -.1};
  state.yaw_rate = .4;
  const auto next = stepInertial(state, {{1, -.5}, .02}, p, .2);
  EXPECT_NEAR(next.pose.position.x(), 1.07, 1e-12);
  EXPECT_NEAR(next.pose.position.y(), 1.975, 1e-12);
  EXPECT_NEAR(next.velocity.x(), .4, 1e-12);
  EXPECT_NEAR(next.velocity.y(), -.15, 1e-12);
  EXPECT_NEAR(next.pose.yaw, .288, 1e-12);
  EXPECT_NEAR(next.yaw_rate, .48, 1e-12);
  EXPECT_NEAR(next.acceleration.x(), .5, 1e-12);
}

TEST(InertialModel, ZeroForceCoastsAndDragDecays)
{
  State2D state;
  state.velocity = {1, 0};
  state.yaw_rate = 1;
  InertialParameters p;
  const auto coast = stepInertial(state, {}, p, .3);
  EXPECT_DOUBLE_EQ(coast.velocity.x(), 1.0);
  EXPECT_NEAR(coast.pose.position.x(), .3, 1e-12);
  p.mass = 2;
  p.linear_drag = 4;
  p.inertia_z = .05;
  p.angular_drag = .1;
  const auto damped = stepInertial(state, {}, p, .3);
  EXPECT_NEAR(damped.velocity.x(), std::exp(-.6), 1e-12);
  EXPECT_NEAR(damped.pose.position.x(), (1 - std::exp(-.6)) / 2, 1e-12);
  EXPECT_NEAR(damped.yaw_rate, std::exp(-.6), 1e-12);
  EXPECT_NEAR(damped.acceleration.x(), -2 * std::exp(-.6), 1e-12);
}

TEST(InertialModel, ForceAndTorqueSaturateBeforeIntegration)
{
  const auto next = stepInertial({}, {{99, -99}, 10}, {}, .1);
  EXPECT_NEAR(next.velocity.x(), .2, 1e-12);
  EXPECT_NEAR(next.velocity.y(), -.2, 1e-12);
  EXPECT_NEAR(next.yaw_rate, 1.0, 1e-12);
  EXPECT_NEAR(next.pose.position.x(), .01, 1e-12);
}

TEST(InertialModel, MassAndStepAndBrakingDistance)
{
  for (const double mass : {1.0, 2.0}) {
    for (const double dt : {.02, .005}) {
      InertialParameters p;
      p.mass = mass;
      State2D state;
      for (int i = 0; i < std::lround(1 / dt); ++i) {
        state = stepInertial(state, {{1, 0}, 0}, p, dt);
      }
      EXPECT_NEAR(state.velocity.x(), 1 / mass, 1e-12);
      EXPECT_NEAR(state.pose.position.x(), .5 / mass, 1e-12);
      State2D braking;
      braking.velocity.x() = 1;
      for (int i = 0; i < std::lround(mass / dt); ++i) {
        braking = stepInertial(braking, {{-1, 0}, 0}, p, dt);
      }
      EXPECT_NEAR(braking.velocity.x(), 0, 1e-12);
      EXPECT_NEAR(braking.pose.position.x(), mass / 2, 1e-12);
    }
  }
}

TEST(InertialModel, SmallDragLimitAndStepComposition)
{
  State2D state;
  state.velocity = {.5, -.2};
  const Wrench2D command{{1, .5}, .01};
  InertialParameters p;
  const auto undamped = stepInertial(state, command, p, .1);
  p.linear_drag = p.angular_drag = 1e-12;
  const auto tiny_drag = stepInertial(state, command, p, .1);
  EXPECT_NEAR((tiny_drag.pose.position - undamped.pose.position).norm(), 0, 1e-12);
  p.linear_drag = .3;
  p.angular_drag = .01;
  const auto whole = stepInertial(state, command, p, .2);
  const auto half = stepInertial(stepInertial(state, command, p, .1), command, p, .1);
  EXPECT_NEAR((whole.pose.position - half.pose.position).norm(), 0, 1e-12);
  EXPECT_NEAR((whole.velocity - half.velocity).norm(), 0, 1e-12);
  EXPECT_NEAR(whole.pose.yaw, half.pose.yaw, 1e-12);
}

TEST(InertialModel, CurvedStepCannotTunnelAcrossOffChordObstacle)
{
  State2D state;
  state.velocity = {1, 1};
  const Wrench2D command{{0, -2}, 0};
  const auto next = stepInertial(state, command, {}, 1);
  World2D world;
  world.circles.push_back({{.5, .25}, .01});
  EXPECT_TRUE(sweptDiskIsFree(world, state.pose.position, next.pose.position, .02));
  EXPECT_FALSE(sweptDiskIsFree(world, state.pose.position, next.pose.position,
    .02 + inertialSweepPadding(state, command, {}, 1)));
}

TEST(InertialModel, RejectInvalidPhysicalParameters)
{
  InertialParameters p;
  p.mass = 0;
  EXPECT_THROW(validateInertialParameters(p), std::invalid_argument);
  p = InertialParameters{};
  p.inertia_z = 0;
  EXPECT_THROW(validateInertialParameters(p), std::invalid_argument);
  p = InertialParameters{};
  p.linear_drag = -1;
  EXPECT_THROW(validateInertialParameters(p), std::invalid_argument);
}
