#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include "motion2d/estimation/fusion_timeline.hpp"
#include "motion2d/estimation/keyframes.hpp"
#include "motion2d/estimation/lidar_odometry.hpp"
#include "motion2d/sim/imu.hpp"
#include "motion2d/sim/lidar_cpu.hpp"
#include "motion2d/sim/world.hpp"

namespace
{
/** @brief Stop at rectangle corners using a quintic time law; body yaw stays zero. */
motion2d::State2D reference(double time)
{
  const Eigen::Vector2d corners[]{{-3,-2}, {3,-2}, {3,2}, {-3,2}, {-3,-2}};
  const int side = std::min(3, static_cast<int>(time/6.));
  const double u = std::clamp((time-side*6.)/6., 0., 1.);
  const double s = 10*std::pow(u,3)-15*std::pow(u,4)+6*std::pow(u,5);
  const double v = (30*u*u-60*u*u*u+30*std::pow(u,4))/6.;
  const double a = (60*u-180*u*u+120*u*u*u)/36.;
  const Eigen::Vector2d difference = corners[side+1]-corners[side];
  motion2d::State2D state;
  state.pose.position = corners[side]+s*difference;
  state.velocity = v*difference; state.acceleration = a*difference;
  return state;
}

void saveGrid(const std::filesystem::path & path, const motion2d::OccupancyGrid2D & grid)
{
  std::ofstream file(path);
  file << "P2\n" << grid.config().width << ' ' << grid.config().height << "\n255\n";
  for (int value : grid.occupancy()) {file << (value < 0 ? 255 : value) << ' ';}
  file << '\n';
}
}

/** @brief Full sensor replay on a physical collision-free rectangle, with/without loops. */
int main(int argc, char ** argv)
{
  const std::filesystem::path output = argc > 1 ? argv[1] : "tmp/ch10_slam";
  std::filesystem::create_directories(output);
  motion2d::World2D world; world.width = 14.; world.height = 12.;
  world.circles = {{{-.7,.5}, .45}};
  world.polygons = {{{{1.,-.5}, {1.8,-.4}, {1.4,.5}}}};
  const auto origin = reference(0).pose;
  for (int side = 0; side < 4; ++side) {
    if (!motion2d::sweptDiskIsFree(world, reference(side*6.).pose.position,
      reference((side+1)*6.).pose.position, .2)) {
      std::cerr << "Reference rectangle has no safe disk sweep\n"; return 1;
    }
  }
  motion2d::LidarConfig lidar_config; lidar_config.beams = 360; lidar_config.range_stddev = .03;
  lidar_config.range_max = 20.;
  motion2d::ImuNoise imu_noise;
  std::mt19937 laser_random(4242), imu_random(6060);
  motion2d::LidarOdometry lidar;
  motion2d::EkfConfig ekf; ekf.estimate_bias = true;
  motion2d::FusionTimeline fusion(ekf);
  motion2d::KeyframeConfig open_config; open_config.enable_loops = false;
  motion2d::KeyframeSlam open(open_config), closed;
  int lidar_rejected = 0, ekf_rejected = 0, candidates = 0;
  std::ofstream events(output / "ch10_loop_events.csv");
  events << "time,keyframes,candidates,loops,status\n";
  for (int k = 0; k <= 4800; ++k) {
    const auto stamp = k*5000000LL;
    const auto truth = reference(k*.005);
    const auto imu = motion2d::addImuNoise(motion2d::idealImu(truth), imu_noise, imu_random);
    motion2d::PlanarImu sample;
    sample.acceleration = imu.specific_force.head<2>(); sample.yaw_rate = imu.angular_velocity.z();
    sample.variance = {std::pow(imu_noise.accel_stddev.x(),2), std::pow(imu_noise.accel_stddev.y(),2),
      std::pow(imu_noise.gyro_stddev.z(),2)};
    fusion.pushImu({stamp, sample});
    if (k%20 != 0) {continue;}
    auto ranges = motion2d::scanCpu(world, truth.pose, lidar_config);
    motion2d::addRangeNoise(ranges, lidar_config, laser_random);
    motion2d::Scan2D scan{lidar_config.angle_min, motion2d::beamIncrement(lidar_config),
      lidar_config.range_min, lidar_config.range_max, ranges};
    const auto estimate = lidar.update(motion2d::projectScan(scan), stamp);
    if (!estimate.accepted) {++lidar_rejected; continue;}
    const auto corrected = fusion.correct(stamp, estimate.pose);
    if (!corrected) {++ekf_rejected; continue;}
    open.update(scan, corrected->filter.pose(), stamp);
    const auto result = closed.update(scan, corrected->filter.pose(), stamp);
    candidates += result.candidates;
    if (result.keyframe_added) {
      events << k*.005 << ',' << closed.frames().size() << ',' << result.candidates << ','
        << closed.loopCount() << ',' << result.status << '\n';
    }
  }
  std::ofstream summary(output / "ch10_slam.csv");
  summary << "mode,keyframes,loops,position_rmse,final_position_error,candidates,lidar_rejected,ekf_rejected\n";
  for (const auto & [name, slam] : std::vector<std::pair<std::string, const motion2d::KeyframeSlam *>>{
    {"no_loop", &open}, {"loop", &closed}}) {
    double squared = 0, final_error = 0;
    for (const auto & frame : slam->frames()) {
      const auto truth = motion2d::compose(motion2d::inverse(origin), reference(frame.stamp*1e-9).pose);
      final_error = (frame.map_pose.position-truth.position).norm(); squared += final_error*final_error;
    }
    summary << name << ',' << slam->frames().size() << ',' << slam->loopCount() << ','
      << std::sqrt(squared/slam->frames().size()) << ',' << final_error << ',' << candidates << ','
      << lidar_rejected << ',' << ekf_rejected << '\n';
    const auto map = motion2d::rebuildKeyframeMap(slam->frames());
    saveGrid(output / (name+".pgm"), map.grid);
    std::ofstream cloud(output / (name+"_cloud.csv")); cloud << "x,y\n";
    for (const auto & point : map.cloud) {cloud << point.x() << ',' << point.y() << '\n';}
  }
  std::ofstream path(output / "ch10_slam_trace.csv");
  path << "time,truth_x,truth_y,odom_x,odom_y,optimized_x,optimized_y,odom_error,optimized_error\n";
  for (const auto & frame : closed.frames()) {
    const auto truth = motion2d::compose(motion2d::inverse(origin), reference(frame.stamp*1e-9).pose);
    path << frame.stamp*1e-9 << ',' << truth.position.x() << ',' << truth.position.y() << ','
      << frame.odom_pose.position.x() << ',' << frame.odom_pose.position.y() << ','
      << frame.map_pose.position.x() << ',' << frame.map_pose.position.y() << ','
      << (frame.odom_pose.position-truth.position).norm() << ',' << (frame.map_pose.position-truth.position).norm() << '\n';
  }
  std::cout << "Wrote actual scan/IMU SLAM replay to " << output << '\n';
}
