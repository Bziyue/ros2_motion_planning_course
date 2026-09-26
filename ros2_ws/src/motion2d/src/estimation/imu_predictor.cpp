#include "motion2d/estimation/planar_ekf.hpp"
#include <cmath>
#include <stdexcept>

namespace motion2d
{
ImuPrediction predictImu(const Vector8d & state, const PlanarImu & imu, double dt)
{
  if (!state.allFinite() || !imu.acceleration.allFinite() || !std::isfinite(imu.yaw_rate) ||
      !imu.variance.allFinite() || (imu.variance.array() < 0).any() ||
      !std::isfinite(dt) || dt <= 0 || dt > .1) {
    throw std::invalid_argument("IMU prediction needs finite data and dt in (0,.1] s");
  }
  ImuPrediction result;
  const Eigen::Matrix2d r = rotation(state[4]);
  const Eigen::Vector2d body_acceleration = imu.acceleration - state.segment<2>(5);
  const Eigen::Vector2d a = r * body_acceleration;
  const Eigen::Vector2d da = r * Eigen::Vector2d(-body_acceleration.y(), body_acceleration.x());
  // imu_predict_begin
  result.state = state;
  result.state.head<2>() += dt * state.segment<2>(2) + .5 * dt * dt * a;
  result.state.segment<2>(2) += dt * a;
  result.state[4] = wrapAngle(state[4] + dt * (imu.yaw_rate - state[7]));
  result.f.setIdentity();
  result.f.block<2, 2>(0, 2) = dt * Eigen::Matrix2d::Identity();
  result.f.block<2, 1>(0, 4) = .5 * dt * dt * da;
  result.f.block<2, 1>(2, 4) = dt * da;
  result.f.block<2, 2>(0, 5) = -.5 * dt * dt * r;
  result.f.block<2, 2>(2, 5) = -dt * r;
  result.f(4, 7) = -dt;
  result.g.setZero();
  result.g.block<2, 2>(0, 0) = .5 * dt * dt * r;
  result.g.block<2, 2>(2, 0) = dt * r;
  result.g(4, 2) = dt;
  // imu_predict_end
  return result;
}
}  // namespace motion2d
