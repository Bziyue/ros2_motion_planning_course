#include <cmath>
#include <iomanip>
#include <iostream>
#include "motion2d/sim/lidar_cpu.hpp"

/** @brief Reproduce a distance-noise experiment far from both range limits.
 * @details Outputs CSV for the textbook: 50000 independent returns at 5 m,
 * sigma=.01 m, seed=4242. No geometry, ROS or range censoring is involved.
 */
int main()
{
  motion2d::LidarConfig config;
  config.range_stddev = .01;
  constexpr int count = 50000;
  std::mt19937 random(4242);
  std::vector<float> ranges(count, 5.0F);
  motion2d::addRangeNoise(ranges, config, random);
  double sum = 0, square_sum = 0;
  for (float range : ranges) {
    const double error = static_cast<double>(range) - 5.0;
    sum += error;
    square_sum += error * error;
  }
  const double mean = sum / count;
  const double variance = square_sum / count - mean * mean;
  std::cout << "samples,seed,sigma_m,mean_m,stddev_m\n" << std::fixed << std::setprecision(8)
            << count << ",4242," << config.range_stddev << "," << mean << ","
            << std::sqrt(variance) << "\n";
}
