#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>
#include "motion2d/estimation/planar_ekf.hpp"

using namespace motion2d;

TEST(PlanarEkf, StateAndInputJacobiansMatchIndependentPerturbations)
{
  Vector8d state;
  state << 1, -2, .3, -.2, .7, .02, -.03, .01;
  PlanarImu imu;
  imu.acceleration = {.2, -.4}; imu.yaw_rate = .6;
  const auto step = predictImu(state, imu, .013);
  const double epsilon = 1e-6;
  for (int i = 0; i < 8; ++i) {
    auto plus = state, minus = state;
    plus[i] += epsilon; minus[i] -= epsilon;
    const Vector8d derivative = (predictImu(plus, imu, .013).state -
      predictImu(minus, imu, .013).state) / (2 * epsilon);
    EXPECT_LT((derivative - step.f.col(i)).norm(), 3e-10) << i;
  }
  for (int i = 0; i < 3; ++i) {
    auto plus = imu, minus = imu;
    if (i < 2) {plus.acceleration[i] += epsilon; minus.acceleration[i] -= epsilon;}
    else {plus.yaw_rate += epsilon; minus.yaw_rate -= epsilon;}
    const Vector8d derivative = (predictImu(state, plus, .013).state -
      predictImu(state, minus, .013).state) / (2 * epsilon);
    EXPECT_LT((derivative - step.g.col(i)).norm(), 3e-10) << i;
  }
}

TEST(PlanarEkf, ConstantAccelerationMatchesAnalyticPositionAndVelocity)
{
  PlanarEkf filter;
  PlanarImu imu;
  imu.acceleration = {2, -1};
  for (int i = 0; i < 200; ++i) {filter.predict(imu, .005);}
  EXPECT_LT((filter.pose().position - Eigen::Vector2d(1, -.5)).norm(), 1e-12);
  EXPECT_LT((filter.state().segment<2>(2) - Eigen::Vector2d(2, -1)).norm(), 1e-12);
  EXPECT_TRUE(filter.state().tail<3>().isZero());
  EXPECT_TRUE(filter.covariance().bottomRows<3>().isZero());
}

TEST(PlanarEkf, PerSampleNoiseCarriesPositionVelocityCrossCovariance)
{
  PlanarEkf quiet, noisy;
  PlanarImu imu;
  imu.variance.setZero(); quiet.predict(imu, .02);
  imu.variance = {4, 9, .01}; noisy.predict(imu, .02);
  const Matrix8d difference = noisy.covariance() - quiet.covariance();
  EXPECT_NEAR(difference(0, 0), .25 * std::pow(.02, 4) * 4, 1e-15);
  EXPECT_NEAR(difference(0, 2), .5 * std::pow(.02, 3) * 4, 1e-15);
  EXPECT_NEAR(difference(2, 2), .02 * .02 * 4, 1e-15);
  EXPECT_NEAR(difference(4, 4), .02 * .02 * .01, 1e-15);
}

TEST(PlanarEkf, PoseInnovationWrapsAndOutliersDoNotChangeState)
{
  PlanarEkf filter;
  filter.reset({{0, 0}, std::acos(-1.) - .01});
  EXPECT_TRUE(filter.correct({{0, 0}, -std::acos(-1.) + .01}));
  const auto state = filter.state();
  const auto covariance = filter.covariance();
  EXPECT_FALSE(filter.correct({{100, 0}, 0}));
  EXPECT_EQ(filter.state(), state);
  EXPECT_EQ(filter.covariance(), covariance);
}

TEST(PlanarEkf, RepeatedPoseObservationsIdentifyStationaryBias)
{
  EkfConfig config;
  config.estimate_bias = true;
  PlanarEkf filter(config);
  PlanarImu biased;
  biased.acceleration = {.08, -.04}; biased.yaw_rate = .01;
  for (int i = 1; i <= 4000; ++i) {
    filter.predict(biased, .005);
    if (i % 20 == 0) {ASSERT_TRUE(filter.correct({}));}
  }
  EXPECT_NEAR(filter.state()[5], .08, .004);
  EXPECT_NEAR(filter.state()[6], -.04, .004);
  EXPECT_NEAR(filter.state()[7], .01, .0005);
  EXPECT_LT(filter.pose().position.norm(), .005);
  EXPECT_LT((filter.covariance() - filter.covariance().transpose()).norm(), 1e-15);
  Eigen::SelfAdjointEigenSolver<Matrix8d> eigen(filter.covariance());
  EXPECT_GE(eigen.eigenvalues().minCoeff(), -1e-12);
}

TEST(PlanarEkf, InvalidNoiseAndIntervalsAreRejected)
{
  EkfConfig config; config.pose_stddev.x() = 0;
  EXPECT_THROW(PlanarEkf{config}, std::invalid_argument);
  PlanarEkf filter;
  EXPECT_THROW(filter.predict({}, 0), std::invalid_argument);
  EXPECT_THROW(filter.predict({}, .2), std::invalid_argument);
  PlanarImu imu; imu.variance.x() = -1;
  EXPECT_THROW(filter.predict(imu, .01), std::invalid_argument);
}
