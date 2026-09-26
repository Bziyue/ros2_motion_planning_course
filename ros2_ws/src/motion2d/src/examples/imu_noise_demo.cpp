#include <iomanip>
#include <iostream>
#include <string>
#include "motion2d/sim/imu.hpp"

/** @brief Fixed-seed six-axis statistics, or Monte Carlo integration (--drift).
 * @details Drift uses known fixed attitude and independent x-accel/z-gyro; this
 * isolates white-noise accumulation and is not a localization algorithm.
 */
int main(int argc, char ** argv)
{
  using namespace motion2d;
  const ImuNoise noise;
  std::cout << std::fixed << std::setprecision(9);
  if (argc == 2 && std::string(argv[1]) == "--drift") {
    constexpr int trials = 1000, steps = 2000;
    constexpr double dt = .005;
    std::cout << "sigma_scale,yaw_rms,yaw_theory,position_rms,position_theory\n";
    for (const double scale : {1.0, 2.0}) {
      ImuNoise scaled = noise;
      scaled.gyro_stddev *= scale;
      scaled.accel_stddev *= scale;
      std::mt19937 random(6060);
      double yaw_squared = 0, position_squared = 0;
      for (int trial = 0; trial < trials; ++trial) {
        double yaw = 0, position = 0, velocity = 0;
        for (int k = 0; k < steps; ++k) {
          const auto sample = addImuNoise({}, scaled, random);
          // drift_integration_begin
          yaw += sample.angular_velocity.z() * dt;
          position += velocity * dt + .5 * sample.specific_force.x() * dt * dt;
          velocity += sample.specific_force.x() * dt;
          // drift_integration_end
        }
        yaw_squared += yaw * yaw;
        position_squared += position * position;
      }
      // Sum_{j=0}^{N-1}(j+1/2)^2 = N*(4*N^2-1)/12.
      const double n = steps;
      const double position_variance = std::pow(scaled.accel_stddev.x() * dt * dt, 2) *
        n * (4 * n * n - 1) / 12;
      std::cout << scale << ',' << std::sqrt(yaw_squared / trials) << ','
                << scaled.gyro_stddev.z() * dt * std::sqrt(n) << ','
                << std::sqrt(position_squared / trials) << ','
                << std::sqrt(position_variance) << '\n';
    }
    return 0;
  }
  if (argc != 1) {
    std::cerr << "Usage: imu_noise_demo [--drift]\n";
    return 1;
  }
  constexpr int count = 60000;
  std::mt19937 random(6060);
  Eigen::Matrix<double, 6, 1> sum = Eigen::Matrix<double, 6, 1>::Zero(), square = sum;
  for (int k = 0; k < count; ++k) {
    const auto error = addImuNoise({}, noise, random);
    Eigen::Matrix<double, 6, 1> values;
    values << error.angular_velocity, error.specific_force;
    sum += values;
    square += values.cwiseAbs2();
  }
  const char * names[] = {"gx", "gy", "gz", "ax", "ay", "az"};
  std::cout << "axis,count,sigma,mean,measured_stddev\n";
  for (int axis = 0; axis < 6; ++axis) {
    const double mean = sum[axis] / count;
    const double sigma = axis < 3 ? noise.gyro_stddev[axis] : noise.accel_stddev[axis - 3];
    std::cout << names[axis] << ',' << count << ',' << sigma << ',' << mean << ','
              << std::sqrt(square[axis] / count - mean * mean) << '\n';
  }
}
