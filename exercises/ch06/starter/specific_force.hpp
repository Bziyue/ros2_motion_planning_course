#pragma once
#include <Eigen/Core>
#include <stdexcept>

/** @brief Convert horizontal world acceleration to body specific force, in m/s^2.
 * @param yaw Body heading (rad); gravity is positive, body z points upward.
 */
inline Eigen::Vector3d studentSpecificForce(
  const Eigen::Vector2d & acceleration, double yaw, double gravity)
{
  // EXERCISE(ch06-1): use R(yaw)^T and include vertical specific force.
  (void)acceleration; (void)yaw; (void)gravity;
  throw std::logic_error("Complete EXERCISE(ch06-1)");
}
