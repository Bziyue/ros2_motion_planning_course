#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include "motion2d/evaluation/map_evaluation.hpp"
#include "motion2d/sim/lidar_cpu.hpp"
#include "motion2d/sim/robot_model.hpp"

namespace
{
using namespace motion2d;
struct Sample {Pose2D pose; Scan2D scan;};

std::vector<Sample> measurements(const World2D & world, LidarConfig lidar, bool moving)
{
  std::mt19937 random(4242);
  std::vector<Sample> data;
  for (int i = 0; i < 160; ++i) {
    const Pose2D pose = moving ? sampleCircle({world.start, 0}, 1, .4, .1 * i).pose : Pose2D{};
    if (!isFree(world, pose.position, .2)) {throw std::runtime_error("unsafe evaluation pose");}
    auto ranges = scanCpu(world, pose, lidar);
    addRangeNoise(ranges, lidar, random);
    // Match LaserScan's float32 metadata, not a more accurate offline angle.
    Scan2D scan{static_cast<float>(lidar.angle_min), static_cast<float>(beamIncrement(lidar)),
      static_cast<float>(lidar.range_min), static_cast<float>(lidar.range_max), std::move(ranges)};
    data.push_back({pose, std::move(scan)});
  }
  return data;
}

void saveGrid(const std::filesystem::path & path, const GridConfig & config,
  const std::vector<std::int8_t> & values)
{
  // PGM as compact experiment data: 0..100 probabilities, 255 unknown.
  // Row zero is the lowest map y; plotting must use origin="lower".
  std::ofstream out(path, std::ios::binary);
  out.exceptions(std::ios::badbit | std::ios::failbit);
  out << "P5\n" << config.width << " " << config.height << "\n255\n";
  for (const auto value : values) {out.put(static_cast<char>(value < 0 ? 255 : value));}
}

std::pair<double, int> wallWidth(const OccupancyGrid2D & grid)
{
  const auto values = grid.occupancy();
  const auto & cfg = grid.config();
  double sum = 0;
  int rows = 0;
  for (int y = 0; y < cfg.height; ++y) {
    const double center_y = grid.cellCenter(y * cfg.width).y();
    if (center_y < -1 || center_y > 1) {continue;}
    int first = cfg.width, last = -1;
    for (int x = 0; x < cfg.width; ++x) {
      const int index = y * cfg.width + x;
      const double center_x = grid.cellCenter(index).x();
      if (center_x < 3.0 || center_x > 5.0 || values[index] < 65) {continue;}
      first = std::min(first, x);
      last = std::max(last, x);
    }
    if (last >= first) {sum += (last - first + 1) * cfg.resolution; ++rows;}
  }
  return {rows ? sum / rows : std::numeric_limits<double>::quiet_NaN(), rows};
}
}

/** @brief Fixed-seed chapter 07 resolution/noise experiment, separate from mapping.
 * @details Generate scans once per sigma and reuse them at all resolutions.
 * Truth rasterization is used only AFTER observation-only integration.
 * Outputs metrics CSV and PGM experiment data; no ROS transport/timing benchmark.
 */
int main(int argc, char ** argv)
{
  if (argc != 2) {std::cerr << "Usage: mapping_demo OUTPUT_DIRECTORY\n"; return 1;}
  try {
    const std::filesystem::path output(argv[1]);
    std::filesystem::create_directories(output);
    const auto world = generateWorld(WorldConfig{});
    std::ofstream metrics(output / "ch07_mapping.csv"), wall(output / "ch07_wall.csv");
    metrics.exceptions(std::ios::badbit | std::ios::failbit);
    wall.exceptions(std::ios::badbit | std::ios::failbit);
    metrics << "resolution,sigma,total,observed,uncertain,tp,fp,fn,tn,coverage,precision,recall\n";
    wall << "resolution,sigma,mean_width,rows\n";
    metrics << std::fixed << std::setprecision(6);
    wall << std::fixed << std::setprecision(6);
    for (int noise_mm : {0, 50}) {
      LidarConfig lidar;
      lidar.range_stddev = noise_mm / 1000.0;
      const auto samples = measurements(world, lidar, true);
      for (int resolution_mm : {50, 100, 200}) {
        GridConfig config;
        config.resolution = resolution_mm / 1000.0;
        config.width = config.height = static_cast<int>(std::lround(22 / config.resolution));
        OccupancyGrid2D grid(config);
        for (const auto & sample : samples) {grid.insertScan(sample.scan, sample.pose);}
        const auto truth = rasterizeTruth(world, config);
        if (resolution_mm == 100 && noise_mm == 0) {
          std::vector<std::int8_t> reference(truth.begin(), truth.end());
          for (auto & value : reference) {value *= 100;}
          saveGrid(output / "truth.pgm", config, reference);
        }
        const auto result = evaluateObserved(grid.occupancy(), truth);
        metrics << config.resolution << "," << lidar.range_stddev << "," << truth.size()
                << "," << result.observed << "," << result.uncertain << ","
                << result.true_positive << "," << result.false_positive << ","
                << result.false_negative << "," << result.true_negative << ","
                << static_cast<double>(result.observed) / truth.size() << ","
                << result.precision() << "," << result.recall() << "\n";
        saveGrid(output / ("map_" + std::to_string(resolution_mm) + "_" +
          std::to_string(noise_mm) + ".pgm"), config, grid.occupancy());
      }
    }
    World2D room;
    room.width = 8.06; room.height = 8;  // Right wall x=4.03 avoids grid-line coincidence.
    for (int noise_mm : {0, 50, 100}) {
      LidarConfig lidar;
      lidar.beams = 161; lidar.angle_min = -.4; lidar.fov = .8;
      lidar.range_max = 6; lidar.range_stddev = noise_mm / 1000.0;
      const auto samples = measurements(room, lidar, false);
      for (int resolution_mm : {50, 100, 200}) {
        GridConfig config;
        config.origin = {-5, -5};
        config.resolution = resolution_mm / 1000.0;
        config.width = config.height = static_cast<int>(std::lround(10 / config.resolution));
        OccupancyGrid2D grid(config);
        for (const auto & sample : samples) {grid.insertScan(sample.scan, sample.pose);}
        const auto [width, rows] = wallWidth(grid);
        wall << config.resolution << "," << lidar.range_stddev << "," << width << "," << rows << "\n";
        saveGrid(output / ("wall_" + std::to_string(resolution_mm) + "_" +
          std::to_string(noise_mm) + ".pgm"), config, grid.occupancy());
      }
    }
    std::cout << "Wrote fixed-seed observed-map and wall experiments to " << output << "\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << "\n"; return 1;
  }
  return 0;
}
