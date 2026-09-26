#pragma once
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "motion2d/sim/lidar_cpu.hpp"

namespace motion2d
{
/** @brief Convert measured ranges to LaserScan, frame laser, snapshot timing.
 * @param stamp Acquisition time shared by all rays, not callback arrival time.
 * @param period Time between scans in s; time_increment is always zero.
 * @pre Valid config, ranges.size()==config.beams.
 */
sensor_msgs::msg::LaserScan toLaserScan(const LidarConfig & config,
  const std::vector<float> & ranges, const builtin_interfaces::msg::Time & stamp, double period);

/** @brief Measured beam visualization in odom, anchored at the acquisition pose.
 * @details Finite echoes are orange lines, +inf rays are faint blue lines to
 * range_max, NaNs are omitted. LaserScan itself supplies RViz's hit points.
 * Display markers are truth-aligned helpers, never inputs to mapping/SLAM.
 */
visualization_msgs::msg::MarkerArray lidarBeamMarkers(
  const sensor_msgs::msg::LaserScan & scan, const Pose2D & acquisition_pose);
}  // namespace motion2d
