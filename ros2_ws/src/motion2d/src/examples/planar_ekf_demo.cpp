#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include "motion2d/estimation/planar_ekf.hpp"

/** @brief Isolate filter mathematics with synthetic noisy pose measurements.
 * @details This is NOT a SLAM experiment: the pose sensor is generated from an
 * analytic path. The online chapter uses actual scan matching instead.
 */
int main(int argc, char ** argv)
{
  const std::filesystem::path output = argc > 1 ? argv[1] : "ch09_filter";
  std::filesystem::create_directories(output);
  motion2d::EkfConfig bias_config;
  bias_config.estimate_bias = true;
  std::array<motion2d::PlanarEkf, 3> filters{
    motion2d::PlanarEkf{}, motion2d::PlanarEkf{}, motion2d::PlanarEkf{bias_config}};
  const std::array<std::string, 3> names{"imu_only", "fixed_zero_bias", "estimate_bias"};
  std::array<double, 3> sum{}, dropout_sum{};
  std::array<int, 3> accepted{}, rejected{};
  int dropout_samples = 0;
  std::mt19937 random(9090);
  std::normal_distribution<double> gaussian(0, 1);
  std::ofstream trace(output / "ch09_filter_trace.csv");
  trace << "time,imu_error,zero_bias_error,estimated_bias_error,bax,bay,bg\n";
  for (int k = 1; k <= 4000; ++k) {
    const double previous_time = (k - 1) * .005, time = k * .005;
    const double yaw = .6 * previous_time;
    const Eigen::Vector2d acceleration(-.36 * std::sin(yaw), .18 * std::cos(yaw));
    motion2d::PlanarImu imu;
    imu.acceleration = motion2d::rotation(yaw).transpose() * acceleration;
    imu.acceleration += Eigen::Vector2d(.08 + .04 * gaussian(random), -.04 + .05 * gaussian(random));
    imu.yaw_rate = .6 + .01 + .004 * gaussian(random);
    const motion2d::Pose2D truth{{std::sin(.6 * time), .5 * (1 - std::cos(.6 * time))},
      motion2d::wrapAngle(.6 * time)};
    auto observation = truth;
    // Draw once and feed exactly the same measurements to both fusion variants.
    const bool observe = k % 20 == 0 && !(time >= 4 && time < 5);
    if (observe) {
      observation.position += Eigen::Vector2d(.03 * gaussian(random), .03 * gaussian(random));
      observation.yaw = motion2d::wrapAngle(observation.yaw + .01 * gaussian(random));
    }
    std::array<double, 3> error{};
    const bool dropout = time >= 4 && time < 5;
    if (dropout) {++dropout_samples;}
    for (int i = 0; i < 3; ++i) {
      filters[i].predict(imu, .005);
      if (i > 0 && observe) {
        if (filters[i].correct(observation)) {++accepted[i];} else {++rejected[i];}
      }
      error[i] = (filters[i].pose().position - truth.position).norm();
      sum[i] += error[i] * error[i];
      if (dropout) {dropout_sum[i] += error[i] * error[i];}
    }
    if (k % 20 == 0) {
      trace << time << ',' << error[0] << ',' << error[1] << ',' << error[2] << ','
            << filters[2].state()[5] << ',' << filters[2].state()[6] << ',' << filters[2].state()[7] << '\n';
    }
  }
  std::ofstream summary(output / "ch09_filter.csv");
  summary << "mode,rmse,dropout_rmse,accepted,rejected,bax,bay,bg\n";
  for (int i = 0; i < 3; ++i) {
    summary << names[i] << ',' << std::sqrt(sum[i] / 4000) << ','
      << std::sqrt(dropout_sum[i] / dropout_samples) << ',' << accepted[i] << ',' << rejected[i] << ','
      << filters[i].state()[5] << ',' << filters[i].state()[6] << ',' << filters[i].state()[7] << '\n';
  }
  std::cout << "Synthetic pose-sensor experiment written to " << output << '\n';
}
