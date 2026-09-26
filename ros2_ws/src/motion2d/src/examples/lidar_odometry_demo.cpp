#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include "motion2d/estimation/lidar_odometry.hpp"
#include "motion2d/mapping/scan_projection.hpp"
#include "motion2d/sim/lidar_cpu.hpp"
#include "motion2d/sim/robot_model.hpp"
#include "motion2d/sim/world.hpp"

/** @brief Sensor replay and independent first-frame-aligned evaluation; ch08. */
int main(int argc, char ** argv)
{
  const std::filesystem::path output = argc > 1 ? argv[1] : "ch08_results";
  std::filesystem::create_directories(output);
  const auto world = motion2d::generateWorld({});
  const motion2d::Pose2D start{world.start, 0};
  std::ofstream summary(output / "ch08_odometry.csv");
  summary << "beams,sigma,accepted,rejected,ate_rmse,final_position_error,final_yaw_error\n";
  for (int beams : {90, 360, 720}) {
    for (double sigma : {0., .03}) {
      motion2d::LidarConfig lidar;
      lidar.beams = beams; lidar.range_stddev = sigma;
      motion2d::LidarOdometry odometry;
      std::mt19937 random(4242);
      std::ofstream path(output / ("path_" + std::to_string(beams) + "_" +
        (sigma == 0 ? "clean" : "noisy") + ".csv"));
      path << "time,truth_x,truth_y,estimated_x,estimated_y,error,status\n";
      int accepted = 0, rejected = 0;
      double sum = 0, error = 0, yaw_error = 0;
      for (int k = 0; k < 160; ++k) {
        const auto truth = motion2d::sampleCircle(start, 1., .4, k * .1).pose;
        auto ranges = motion2d::scanCpu(world, truth, lidar);
        motion2d::addRangeNoise(ranges, lidar, random);
        motion2d::Scan2D scan{lidar.angle_min, motion2d::beamIncrement(lidar),
          lidar.range_min, lidar.range_max, ranges};
        const auto estimate = odometry.update(motion2d::projectScan(scan), k * 100000000LL);
        const auto aligned_truth = motion2d::compose(motion2d::inverse(start), truth);
        error = (estimate.pose.position - aligned_truth.position).norm();
        yaw_error = motion2d::wrapAngle(estimate.pose.yaw - aligned_truth.yaw);
        if (estimate.accepted) {++accepted; sum += error * error;} else {++rejected;}
        path << k * .1 << ',' << aligned_truth.position.x() << ',' << aligned_truth.position.y()
             << ',' << estimate.pose.position.x() << ',' << estimate.pose.position.y()
             << ',' << error << ',' << estimate.status << '\n';
      }
      summary << beams << ',' << sigma << ',' << accepted << ',' << rejected << ','
              << (accepted ? std::sqrt(sum / accepted) : NAN) << ',' << error << ',' << yaw_error << '\n';
    }
  }
  std::cout << "Wrote scan-only odometry evaluation to " << output << '\n';
}
