#include "motion2d/sim/robot_model.hpp"
#include <algorithm>
#include <cmath>

namespace motion2d
{
VelocityCommand limitVelocity(VelocityCommand command, double speed_max, double yaw_rate_max)
{
  const double speed = command.velocity.norm();
  if (speed > speed_max) {command.velocity *= speed_max / speed;}
  command.yaw_rate = std::clamp(command.yaw_rate, -yaw_rate_max, yaw_rate_max);
  return command;
}

// velocity_begin
State2D stepVelocity(const State2D & state, const VelocityCommand & command, double dt)
{
  State2D next = state;
  const double half_turn = 0.5 * command.yaw_rate * dt;
  next.pose.yaw = wrapAngle(state.pose.yaw + 2.0 * half_turn);
  next.yaw_rate = command.yaw_rate;
  next.yaw_acceleration = 0.0;  // Only within the held-command interval.
  next.acceleration.setZero();
  if (command.body_frame) {
    const double sinc = std::abs(half_turn) < 1e-6 ?
      1.0 - half_turn * half_turn / 6.0 : std::sin(half_turn) / half_turn;
    next.pose.position += dt * sinc * rotation(state.pose.yaw + half_turn) * command.velocity;
    next.velocity = rotation(next.pose.yaw) * command.velocity;
    next.acceleration = command.yaw_rate * Eigen::Vector2d{-next.velocity.y(), next.velocity.x()};
  } else {
    next.pose.position += dt * command.velocity;
    next.velocity = command.velocity;
  }
  return next;
}
// velocity_end

double velocitySweepPadding(const VelocityCommand & command, double dt)
{
  return command.body_frame ? std::abs(command.yaw_rate) * command.velocity.norm() * dt * dt / 8.0 : 0.0;
}
}  // namespace motion2d
