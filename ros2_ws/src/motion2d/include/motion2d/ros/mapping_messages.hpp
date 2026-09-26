#pragma once
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "motion2d/mapping/scan_projection.hpp"
#include "motion2d/mapping/occupancy_grid.hpp"

namespace motion2d
{
/** @brief Validate snapshot LaserScan in laser frame and convert it to Scan2D.
 * @throws std::invalid_argument for unsupported frame, rolling scan or metadata.
 */
Scan2D fromLaserScan(const sensor_msgs::msg::LaserScan & message);

/** @brief Validate odom/base_link planar pose and extract T_odom_base.
 * @throws std::invalid_argument for nonfinite, nonplanar or nonunit orientation.
 * @details This chapter's fixed T_base_laser is identity, so this is T_odom_laser.
 */
Pose2D poseFromOdometry(const nav_msgs::msg::Odometry & message);

/** @brief Serialize finite planar points as FLOAT32 xyz (z=0), one row, 12 B/point.
 * @pre Points are finite and representable as float; header is the acquisition time/frame.
 */
sensor_msgs::msg::PointCloud2 toPointCloud(
  const std::vector<Eigen::Vector2d> & points, const std_msgs::msg::Header & header);

/** @brief Serialize observed probabilities (-1 unknown, 0..100 observed), in map.
 * @param stamp Last integrated scan's acquisition time.
 * @param loaded First integrated scan's acquisition time since clear/reset.
 */
nav_msgs::msg::OccupancyGrid toOccupancyGrid(const OccupancyGrid2D & grid,
  const builtin_interfaces::msg::Time & stamp, const builtin_interfaces::msg::Time & loaded);
}  // namespace motion2d
