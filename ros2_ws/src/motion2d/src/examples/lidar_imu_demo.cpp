#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include "motion2d/estimation/fusion_timeline.hpp"
#include "motion2d/estimation/lidar_odometry.hpp"
#include "motion2d/mapping/scan_projection.hpp"
#include "motion2d/sim/imu.hpp"
#include "motion2d/sim/lidar_cpu.hpp"
#include "motion2d/sim/world.hpp"

/** @brief Replay actual ray-cast scans and raw IMU; truth only generates/evaluates sensors. */
int main(int argc, char ** argv)
{
  const std::filesystem::path output = argc > 1 ? argv[1] : "tmp/ch09_fusion";
  std::filesystem::create_directories(output);
  const auto world = motion2d::generateWorld({});
  const motion2d::Pose2D origin{world.start, 0};
  std::ofstream summary(output / "ch09_fusion.csv");
  summary << "omega,mode,position_rmse,yaw_rmse,dropout_rmse,lidar_rejected,ekf_rejected\n";
  for (double omega : {.4, 1.2}) {
    motion2d::LidarConfig config; config.beams = 360; config.range_stddev = .01;
    motion2d::ImuNoise noise;
    std::mt19937 laser_random(4242), imu_random(6060);
    motion2d::LidarOdometry lidar;
    motion2d::EkfConfig ekf; ekf.estimate_bias = true;
    motion2d::FusionTimeline fusion(ekf);
    struct Delayed {std::int64_t stamp; motion2d::Pose2D pose;};
    std::deque<Delayed> pending;
    motion2d::Pose2D held_lidar;
    double sum_position[2]{}, sum_yaw[2]{}, sum_dropout[2]{};
    int count = 0, dropout_count = 0, lidar_rejected = 0, ekf_rejected = 0;
    std::ofstream path(output / (omega < 1 ? "slow.csv" : "fast.csv"));
    path << "time,truth_x,truth_y,laser_error,fusion_error,laser_yaw_error,fusion_yaw_error,laser_age\n";
    for (int k = 0; k <= 2000; ++k) {
      const auto ns = k * 5000000LL;
      const double time = k * .005;
      const auto truth = motion2d::sampleCircle(origin, 1., omega, time);
      const auto sample = motion2d::addImuNoise(motion2d::idealImu(truth), noise, imu_random);
      motion2d::PlanarImu planar;
      planar.acceleration = sample.specific_force.head<2>(); planar.yaw_rate = sample.angular_velocity.z();
      planar.variance = {std::pow(noise.accel_stddev.x(), 2), std::pow(noise.accel_stddev.y(), 2),
        std::pow(noise.gyro_stddev.z(), 2)};
      fusion.pushImu({ns, planar});
      // Physical IMU continues while scans disappear for half a second.
      const bool dropout = time >= 3. && time < 3.5;
      if (k % 20 == 0 && !dropout) {
        auto ranges = motion2d::scanCpu(world, truth.pose, config);
        motion2d::addRangeNoise(ranges, config, laser_random);
        const motion2d::Scan2D scan{config.angle_min, motion2d::beamIncrement(config),
          config.range_min, config.range_max, ranges};
        const auto result = lidar.update(motion2d::projectScan(scan), ns);
        if (result.accepted) {pending.push_back({ns, result.pose});} else {++lidar_rejected;}
      }
      // Both comparisons see the same 30 ms delivery delay, never future data.
      while (!pending.empty() && pending.front().stamp + 30000000 <= ns) {
        held_lidar = pending.front().pose;
        if (!fusion.correct(pending.front().stamp, pending.front().pose)) {++ekf_rejected;}
        pending.pop_front();
      }
      if (!fusion.ready()) {continue;}
      const auto aligned = motion2d::compose(motion2d::inverse(origin), truth.pose);
      const motion2d::Pose2D poses[]{held_lidar, fusion.latest().filter.pose()};
      double position[2], yaw[2];
      for (int mode = 0; mode < 2; ++mode) {
        position[mode] = (poses[mode].position - aligned.position).norm();
        yaw[mode] = motion2d::wrapAngle(poses[mode].yaw - aligned.yaw);
        sum_position[mode] += position[mode] * position[mode];
        sum_yaw[mode] += yaw[mode] * yaw[mode];
        if (dropout) {sum_dropout[mode] += position[mode] * position[mode];}
      }
      ++count; if (dropout) {++dropout_count;}
      path << time << ',' << aligned.position.x() << ',' << aligned.position.y() << ','
           << position[0] << ',' << position[1] << ',' << yaw[0] << ',' << yaw[1] << ',' << fusion.laserAge() << '\n';
    }
    for (int mode = 0; mode < 2; ++mode) {
      summary << omega << ',' << (mode ? "fusion" : "held_lidar") << ','
        << std::sqrt(sum_position[mode] / count) << ',' << std::sqrt(sum_yaw[mode] / count) << ','
        << std::sqrt(sum_dropout[mode] / dropout_count) << ',' << lidar_rejected << ',' << ekf_rejected << '\n';
    }
  }
  std::cout << "Wrote ray-cast scan / IMU replay to " << output << '\n';
}
