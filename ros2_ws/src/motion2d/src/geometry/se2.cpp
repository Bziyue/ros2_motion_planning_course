#include "motion2d/geometry/se2.hpp"
#include <cmath>

namespace motion2d
{
double wrapAngle(double angle)
{
  const double wrapped = std::remainder(angle, 2.0 * kPi);
  return wrapped >= kPi ? wrapped - 2.0 * kPi : wrapped;
}

// rotation_begin
Eigen::Matrix2d rotation(double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Eigen::Matrix2d result;
  result << c, -s, s, c;
  return result;
}

Eigen::Vector2d transformPoint(const Pose2D & pose, const Eigen::Vector2d & point)
{
  return rotation(pose.yaw) * point + pose.position;
}
// rotation_end

Pose2D compose(const Pose2D & lhs, const Pose2D & rhs)
{
  return {transformPoint(lhs, rhs.position), wrapAngle(lhs.yaw + rhs.yaw)};
}

Pose2D inverse(const Pose2D & pose)
{
  return {-rotation(pose.yaw).transpose() * pose.position, wrapAngle(-pose.yaw)};
}
}  // namespace motion2d
