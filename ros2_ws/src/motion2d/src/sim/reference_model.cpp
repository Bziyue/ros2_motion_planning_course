#include "motion2d/sim/robot_model.hpp"
#include <cmath>

namespace motion2d
{
// reference_begin
State2D sampleCircle(const Pose2D & initial, double radius, double omega, double time)
{
  const double theta = omega * time;
  const auto r0 = rotation(initial.yaw);
  State2D reference;
  reference.pose.position = initial.position +
    r0 * Eigen::Vector2d{radius * std::sin(theta), radius * (1.0 - std::cos(theta))};
  reference.velocity = r0 * Eigen::Vector2d{
    radius * omega * std::cos(theta), radius * omega * std::sin(theta)};
  reference.acceleration = r0 * Eigen::Vector2d{
    -radius * omega * omega * std::sin(theta), radius * omega * omega * std::cos(theta)};
  reference.pose.yaw = wrapAngle(initial.yaw + theta);
  reference.yaw_rate = omega;
  return reference;
}
// reference_end
}  // namespace motion2d
