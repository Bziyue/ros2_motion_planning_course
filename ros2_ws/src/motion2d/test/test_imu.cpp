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

TEST(Imu, NoiseValidationAtConfigurationBoundary)
{
  ImuNoise noise;
  EXPECT_NO_THROW(validateImuNoise(noise));
  noise.accel_stddev.x() = -.1;
  EXPECT_THROW(validateImuNoise(noise), std::invalid_argument);
  noise.accel_stddev.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validateImuNoise(noise), std::invalid_argument);
  noise.accel_stddev.x() = 1e200;
  EXPECT_THROW(validateImuNoise(noise), std::invalid_argument);
}

TEST(Imu, ZeroNoiseConsumesNoRandomnessAndKeepsSample)
{
  ImuNoise noise;
  noise.gyro_stddev.setZero();
  noise.accel_stddev.setZero();
  std::mt19937 random(6060), untouched(6060);
  const auto ideal = idealImu(sampleCircle({}, 1, .4, .3));
  const auto measured = addImuNoise(ideal, noise, random);
  EXPECT_TRUE(measured.angular_velocity.isApprox(ideal.angular_velocity));
  EXPECT_TRUE(measured.specific_force.isApprox(ideal.specific_force));
  EXPECT_EQ(random, untouched);
}

TEST(Imu, ReseedingReplaysWholeSequenceIncludingPartialZeroAxes)
{
  ImuNoise noise;
  noise.gyro_stddev.y() = 0;
  std::mt19937 a(6060), b(6060), lidar(4242);
  for (int i = 0; i < 100; ++i) {
    (void)lidar();  // Independent sensor activity must not affect the IMU stream.
    const auto first = addImuNoise({}, noise, a);
    const auto replay = addImuNoise({}, noise, b);
    EXPECT_TRUE(first.angular_velocity == replay.angular_velocity);
    EXPECT_TRUE(first.specific_force == replay.specific_force);
    EXPECT_EQ(first.angular_velocity.y(), 0);
  }
  a.seed(6060);
  b.seed(6060);
  EXPECT_TRUE(addImuNoise({}, noise, a).specific_force ==
    addImuNoise({}, noise, b).specific_force);
}

TEST(Imu, SixAxisNoiseHasExpectedMeanVarianceAndNoStrongCorrelation)
{
  const ImuNoise noise;
  std::mt19937 random(6060);
  constexpr int count = 60000;
  using Vector6 = Eigen::Matrix<double, 6, 1>;
  using Matrix6 = Eigen::Matrix<double, 6, 6>;
  Vector6 sum = Vector6::Zero(), previous = Vector6::Zero(), lag = Vector6::Zero();
  Matrix6 outer = Matrix6::Zero();
  for (int k = 0; k < count; ++k) {
    const auto error = addImuNoise({}, noise, random);
    Vector6 normalized;
    normalized << error.angular_velocity.cwiseQuotient(noise.gyro_stddev),
      error.specific_force.cwiseQuotient(noise.accel_stddev);
    sum += normalized;
    outer += normalized * normalized.transpose();
    lag += normalized.cwiseProduct(previous);
    previous = normalized;
  }
  const Vector6 mean = sum / count;
  const Matrix6 covariance = outer / count - mean * mean.transpose();
  for (int i = 0; i < 6; ++i) {
    EXPECT_LT(std::abs(mean[i]), 5.0 / std::sqrt(count));
    EXPECT_NEAR(std::sqrt(covariance(i, i)), 1, .02);
    EXPECT_LT(std::abs(lag[i] / (count - 1)), .025);
    for (int j = 0; j < i; ++j) {EXPECT_LT(std::abs(covariance(i, j)), .025);}
  }
}
