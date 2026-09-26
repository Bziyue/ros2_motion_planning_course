#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include "motion2d/sim/lidar_cpu.hpp"

/** @brief Chapter 05 beam-count experiment, reporting core-call latency only.
 * @details Fixed world seed 42, pose (-8,-8,0), no noise. Each beam count uses
 * 100 warmups and 1000 timed scanCpu calls. CSV excludes world generation,
 * noise, ROS conversion, DDS transport and visualization.
 */
int main(int argc, char ** argv)
{
  using namespace motion2d;
  using Clock = std::chrono::steady_clock;
  const auto world = generateWorld(WorldConfig{});
  const Pose2D pose{world.start, 0};
  if (argc == 2 && std::string(argv[1]) == "--points") {
    std::cout << "beams,x,y\n" << std::fixed << std::setprecision(6);
    for (int beams : {90, 360, 720}) {
      LidarConfig config;
      config.beams = beams;
      const auto scan = scanCpu(world, pose, config);
      for (int i = 0; i < beams; ++i) {
        if (!std::isfinite(scan[i])) {continue;}
        const double angle = config.angle_min + i * beamIncrement(config);
        const Eigen::Vector2d point =
          pose.position + scan[i] * Eigen::Vector2d{std::cos(angle), std::sin(angle)};
        std::cout << beams << "," << point.x() << "," << point.y() << "\n";
      }
    }
    return 0;
  }
  if (argc != 1) {
    std::cerr << "Usage: lidar_benchmark [--points]\n";
    return 1;
  }
  constexpr int repetitions = 1000;
  double checksum = 0;
  std::cout << "beams,angle_deg,median_us,p95_us,world_hits,thin_hits,scans,seed\n";
  for (int beams : {90, 360, 720}) {
    LidarConfig config;
    config.beams = beams;
    for (int warmup = 0; warmup < 100; ++warmup) {
      checksum += scanCpu(world, pose, config).front();
    }
    std::vector<double> timings;
    timings.reserve(repetitions);
    for (int sample = 0; sample < repetitions; ++sample) {
      const auto begin = Clock::now();
      const auto ranges = scanCpu(world, pose, config);
      const auto end = Clock::now();
      timings.push_back(std::chrono::duration<double, std::micro>(end - begin).count());
      checksum += ranges.front();  // Consume results outside the timed interval.
    }
    std::sort(timings.begin(), timings.end());
    const auto ranges = scanCpu(world, pose, config);
    const auto world_hits = std::count_if(ranges.begin(), ranges.end(),
      [](float r) {return std::isfinite(r);});

    // An independent aliasing scene: a 0.1 m diameter circle 5 m away at 2 deg.
    World2D thin_world;
    const double angle = 2 * kPi / 180;
    thin_world.circles.push_back({{5 * std::cos(angle), 5 * std::sin(angle)}, .05});
    config.range_max = 8;  // All room walls are beyond range from the origin.
    const auto thin_scan = scanCpu(thin_world, {}, config);
    const auto thin_hits = std::count_if(thin_scan.begin(), thin_scan.end(),
      [](float r) {return std::isfinite(r);});
    std::cout << std::fixed << std::setprecision(3)
              << beams << "," << 360.0 / beams << ","
              << (timings[499] + timings[500]) / 2 << "," << timings[949] << ","
              << world_hits << "," << thin_hits << "," << repetitions << ",42\n";
  }
  std::cerr << "Core scan only; 100 warmups, 1000 samples per N, sigma=0, checksum="
            << checksum << "\n";
}
