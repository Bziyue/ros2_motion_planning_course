#include "motion2d/sim/robot_model.hpp"

namespace motion2d
{
// ideal_begin
State2D idealPose(const Pose2D & target)
{
  State2D result;
  result.pose = target;
  result.pose.yaw = wrapAngle(target.yaw);
  return result;  // Zero derivatives mean "unused", not an IMU measurement.
}
// ideal_end
}  // namespace motion2d
