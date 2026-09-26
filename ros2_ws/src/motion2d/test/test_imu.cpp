#include <gtest/gtest.h>
#include "motion2d/sim/imu.hpp"

using namespace motion2d;

TEST(Imu, RestAndConstantVelocityHaveTheSameSpecificForce)
{
  State2D state;
  const auto rest = idealImu(state);
  state.velocity = {8, -3};
  const auto moving = idealImu(state);
  EXPECT_TRUE(rest.specific_force.isApprox(Eigen::Vector3d(0, 0, 9.81)));
  EXPECT_TRUE(moving.specific_force.isApprox(rest.specific_force));
  EXPECT_TRUE(moving.angular_velocity.isZero());
}

TEST(Imu, AccelerationIsRotatedIntoSensorAxes)
{
  State2D state;
  state.pose.yaw = kPi / 2;
  state.acceleration = {2, 0};
  EXPECT_TRUE(idealImu(state).specific_force.isApprox(Eigen::Vector3d(0, -2, 9.81)));
  state.pose.yaw = -kPi / 2;
  EXPECT_TRUE(idealImu(state, 1.62).specific_force.isApprox(Eigen::Vector3d(0, 2, 1.62)));
}

TEST(Imu, CentreMountHasNoLeverArmAcceleration)
{
  State2D state;
  state.pose.yaw = 1.2;
  state.yaw_rate = -.3;
  state.yaw_acceleration = 4;
  const auto sample = idealImu(state);
  EXPECT_TRUE(sample.angular_velocity.isApprox(Eigen::Vector3d(0, 0, -.3)));
  EXPECT_TRUE(sample.specific_force.isApprox(Eigen::Vector3d(0, 0, 9.81)));
}

TEST(Imu, AnalyticCircleHasConstantBodyCentripetalAcceleration)
{
  Pose2D initial;
  initial.yaw = 2.3;
  for (const double t : {0.0, .37, 8.0, 100.0}) {
    const auto sample = idealImu(sampleCircle(initial, 2, -.6, t));
    EXPECT_NEAR(sample.specific_force.x(), 0, 1e-12);
    EXPECT_NEAR(sample.specific_force.y(), .72, 1e-12);
    EXPECT_NEAR(sample.angular_velocity.z(), -.6, 1e-12);
  }
}

TEST(Imu, InertialAccelerationIncludesDragAndMass)
{
  State2D state;
  state.pose.yaw = kPi / 2;
  state.velocity = {1, 0};
  InertialParameters p;
  p.mass = 2;
  p.linear_drag = 1;
  const auto next = stepInertial(state, {{2, 0}, 0}, p, .2);
  const auto sample = idealImu(next);
  EXPECT_NEAR(sample.specific_force.x(), 0, 1e-12);
  EXPECT_NEAR(sample.specific_force.y(), -.5 * std::exp(-.1), 1e-12);
}
