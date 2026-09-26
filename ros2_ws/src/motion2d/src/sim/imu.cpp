#include "motion2d/sim/imu.hpp"
#include <cmath>
#include <stdexcept>

namespace motion2d
{
// imu_ideal_begin
ImuSample idealImu(const State2D & state, double gravity)
{
  ImuSample sample;
  sample.angular_velocity.z() = state.yaw_rate;
  sample.specific_force.head<2>() =
    rotation(state.pose.yaw).transpose() * state.acceleration;
  sample.specific_force.z() = gravity;
  return sample;
}
// imu_ideal_end

void validateImuNoise(const ImuNoise & noise)
{
  for (int axis = 0; axis < 3; ++axis) {
    for (const double sigma : {noise.gyro_stddev[axis], noise.accel_stddev[axis]}) {
      if (!std::isfinite(sigma) || sigma < 0 || !std::isfinite(sigma * sigma)) {
        throw std::invalid_argument("IMU sigmas must be nonnegative with finite variance");
      }
    }
  }
}

// imu_noise_begin
ImuSample addImuNoise(ImuSample sample, const ImuNoise & noise, std::mt19937 & random)
{
  std::normal_distribution<double> normal(0.0, 1.0);
  for (int axis = 0; axis < 3; ++axis) {
    if (noise.gyro_stddev[axis] > 0) {
      sample.angular_velocity[axis] += noise.gyro_stddev[axis] * normal(random);
    }
    if (noise.accel_stddev[axis] > 0) {
      sample.specific_force[axis] += noise.accel_stddev[axis] * normal(random);
    }
  }
  return sample;
}
// imu_noise_end
}  // namespace motion2d
