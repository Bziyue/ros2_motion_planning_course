#pragma once
#include <vector>
#include "motion2d/geometry/se2.hpp"

namespace motion2d
{
/** @brief Geometry of one snapshot scan, independent of ROS and the true world.
 * @details Angles in rad, ranges in m. NaN is invalid; +inf is a measured
 * no-return ray, not a point. The ROS boundary validates metadata once.
 */
struct Scan2D
{
  double angle_min = 0.0, angle_increment = 0.0;
  double range_min = 0.05, range_max = 10.0;
  std::vector<float> ranges;
};

/** @brief Reject invalid metadata/empty scans; throws invalid_argument.
 * Individual invalid ranges are allowed and skipped by the algorithms.
 */
void validateScan(const Scan2D & scan);

/** @brief Convert finite in-range echoes to laser-frame points in beam order.
 * @pre Validated scan. Inclusive range limits; no synthetic points for +inf.
 * @details \f$p_L=r_i(\cos\alpha_i,\sin\alpha_i)\f$, ch07.
 */
std::vector<Eigen::Vector2d> projectScan(const Scan2D & scan);

/** @brief Transform a current scan's points to the acquisition odometry frame.
 * @param laser_pose T_odom_laser at the scan timestamp, not the latest pose.
 */
std::vector<Eigen::Vector2d> registerPoints(
  const std::vector<Eigen::Vector2d> & points, const Pose2D & laser_pose);
}  // namespace motion2d
