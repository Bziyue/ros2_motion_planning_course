#include "motion2d/sim/imu.hpp"

namespace motion2d
{
// imu_ideal_begin
ImuSample idealImu(const State2D & state, double gravity)
{
  ImuSample sample;
  sample.angular_velocity.z() = state.yaw_rate;
  sample.specific_force.head<2>() =
    rotation(state.pose.yaw).transpose() * state.acceleration;
  sample.specific_force.z() = gravity;
  return sample;
}
// imu_ideal_end
}  // namespace motion2d
