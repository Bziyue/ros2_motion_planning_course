#include "motion2d/estimation/planar_ekf.hpp"
#include <Eigen/Cholesky>
#include <cmath>
#include <stdexcept>

namespace motion2d
{
PlanarEkf::PlanarEkf(EkfConfig config) : config_(config)
{
  if (!config.bias_initial_stddev.allFinite() || !config.bias_random_walk.allFinite() ||
      !config.pose_stddev.allFinite() || (config.bias_initial_stddev.array() < 0).any() ||
      (config.bias_random_walk.array() < 0).any() || (config.pose_stddev.array() <= 0).any() ||
      !std::isfinite(config.innovation_gate) || config.innovation_gate <= 0) {
    throw std::invalid_argument("Invalid planar EKF noise or innovation gate");
  }
  reset();
}

void PlanarEkf::reset(const Pose2D & origin, const Eigen::Vector2d & velocity)
{
  if (!origin.position.allFinite() || !std::isfinite(origin.yaw) || !velocity.allFinite()) {
    throw std::invalid_argument("Nonfinite EKF initial state");
  }
  state_.setZero();
  state_.head<2>() = origin.position;
  state_.segment<2>(2) = velocity;
  state_[4] = wrapAngle(origin.yaw);
  covariance_.setZero();
  covariance_(0, 0) = config_.pose_stddev.x() * config_.pose_stddev.x();
  covariance_(1, 1) = config_.pose_stddev.y() * config_.pose_stddev.y();
  covariance_(4, 4) = config_.pose_stddev.z() * config_.pose_stddev.z();
  covariance_(2, 2) = covariance_(3, 3) = 1.0;
  if (config_.estimate_bias) {
    covariance_.diagonal().tail<3>() = config_.bias_initial_stddev.array().square().matrix();
  }
  last_innovation_ = 0;
}

void PlanarEkf::predict(const PlanarImu & imu, double dt)
{
  const auto step = predictImu(state_, imu, dt);
  state_ = step.state;
  // ekf_covariance_begin
  covariance_ = step.f * covariance_ * step.f.transpose() +
    step.g * imu.variance.asDiagonal() * step.g.transpose();
  if (config_.estimate_bias) {
    covariance_.diagonal().tail<3>() +=
      dt * config_.bias_random_walk.array().square().matrix();
  }
  // ekf_covariance_end
  covariance_ = (.5 * (covariance_ + covariance_.transpose())).eval();
}

bool PlanarEkf::correct(const Pose2D & measured)
{
  if (!measured.position.allFinite() || !std::isfinite(measured.yaw)) {
    throw std::invalid_argument("Nonfinite pose measurement");
  }
  Eigen::Matrix<double, 3, 8> h = Eigen::Matrix<double, 3, 8>::Zero();
  h(0, 0) = h(1, 1) = h(2, 4) = 1;
  Eigen::Vector3d residual;
  residual.head<2>() = measured.position - state_.head<2>();
  residual[2] = wrapAngle(measured.yaw - state_[4]);
  const Eigen::Matrix3d noise = config_.pose_stddev.array().square().matrix().asDiagonal();
  const Eigen::Matrix3d innovation = h * covariance_ * h.transpose() + noise;
  const auto solve = innovation.ldlt();
  last_innovation_ = residual.dot(solve.solve(residual));
  if (last_innovation_ > config_.innovation_gate) {return false;}
  // ekf_correct_begin
  const Eigen::Matrix<double, 8, 3> gain = solve.solve(h * covariance_).transpose();
  state_ += gain * residual;
  state_[4] = wrapAngle(state_[4]);
  const Matrix8d remainder = Matrix8d::Identity() - gain * h;
  covariance_ = remainder * covariance_ * remainder.transpose() + gain * noise * gain.transpose();
  // ekf_correct_end
  covariance_ = (.5 * (covariance_ + covariance_.transpose())).eval();
  return true;
}
}  // namespace motion2d
