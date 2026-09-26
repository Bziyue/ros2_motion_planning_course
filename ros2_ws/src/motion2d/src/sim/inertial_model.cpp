#include "motion2d/sim/robot_model.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion2d
{
namespace
{
struct Gains
{
  double decay, velocity, position;
};

/** @brief ZOH gains for dv/dt=u-lambda*v, stable as lambda approaches zero. */
Gains gains(double lambda, double h)
{
  const double z = lambda * h;
  if (z < 1e-3) {
    // Taylor limits avoid subtracting almost equal numbers in (h-B)/lambda.
    const double b = 1.0 + z * (-.5 + z * (1.0 / 6 + z * (-1.0 / 24 + z / 120)));
    const double c = .5 + z * (-1.0 / 6 + z * (1.0 / 24 + z * (-1.0 / 120 + z / 720)));
    return {std::exp(-z), h * b, h * h * c};
  }
  const double b = -std::expm1(-z) / lambda;
  return {std::exp(-z), b, (h - b) / lambda};
}
}  // namespace

void validateInertialParameters(const InertialParameters & p)
{
  for (const double value : {p.mass, p.inertia_z, p.force_max, p.torque_max}) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("mass, inertia_z and force/torque limits must be finite and positive");
    }
  }
  if (!std::isfinite(p.linear_drag) || !std::isfinite(p.angular_drag) ||
    p.linear_drag < 0.0 || p.angular_drag < 0.0)
  {
    throw std::invalid_argument("Linear/angular drag must be finite and nonnegative");
  }
}

Wrench2D limitWrench(Wrench2D command, const InertialParameters & p)
{
  for (int i = 0; i < 2; ++i) {
    command.force[i] = std::clamp(command.force[i], -p.force_max, p.force_max);
  }
  command.torque = std::clamp(command.torque, -p.torque_max, p.torque_max);
  return command;
}

// inertia_begin
State2D stepInertial(const State2D & state, const Wrench2D & command,
  const InertialParameters & p, double dt)
{
  const auto applied = limitWrench(command, p);
  const auto xy = gains(p.linear_drag / p.mass, dt);
  const auto yaw = gains(p.angular_drag / p.inertia_z, dt);
  const Eigen::Vector2d u = applied.force / p.mass;
  const double alpha = applied.torque / p.inertia_z;
  State2D next;
  next.pose.position = state.pose.position + xy.velocity * state.velocity + xy.position * u;
  next.velocity = xy.decay * state.velocity + xy.velocity * u;
  next.acceleration = (applied.force - p.linear_drag * next.velocity) / p.mass;
  next.pose.yaw = wrapAngle(state.pose.yaw + yaw.velocity * state.yaw_rate + yaw.position * alpha);
  next.yaw_rate = yaw.decay * state.yaw_rate + yaw.velocity * alpha;
  next.yaw_acceleration = (applied.torque - p.angular_drag * next.yaw_rate) / p.inertia_z;
  return next;
}
// inertia_end

double inertialSweepPadding(const State2D & state, const Wrench2D & command,
  const InertialParameters & p, double dt)
{
  const auto applied = limitWrench(command, p);
  const Eigen::Vector2d a0 = (applied.force - p.linear_drag * state.velocity) / p.mass;
  return a0.norm() * dt * dt / 8.0;
}
}  // namespace motion2d
