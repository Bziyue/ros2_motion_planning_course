#pragma once
#include <Eigen/Core>
#include <stdexcept>
/** @brief One finite echo in odom; angles in rad, translation/range in m. */
inline Eigen::Vector2d studentScanPoint(double range, double angle,
  const Eigen::Vector2d & translation, double yaw)
{
  // EXERCISE(ch07-2): polar point, rotate by yaw, then translate.
  (void)range; (void)angle; (void)translation; (void)yaw;
  throw std::logic_error("Complete EXERCISE(ch07-2)");
}
