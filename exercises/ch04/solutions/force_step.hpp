#pragma once
#include "motion2d/sim/robot_model.hpp"

/** @brief Exact zero-drag ZOH step: p+=v*h+a*h^2/2 before v+=a*h. */
inline motion2d::State2D studentForceStep(motion2d::State2D state,
  const motion2d::Wrench2D & applied, double mass, double inertia_z, double dt)
{
  state.acceleration = applied.force / mass;
  state.yaw_acceleration = applied.torque / inertia_z;
  state.pose.position += state.velocity * dt + .5 * state.acceleration * dt * dt;
  state.pose.yaw = motion2d::wrapAngle(state.pose.yaw + state.yaw_rate * dt +
    .5 * state.yaw_acceleration * dt * dt);
  state.velocity += state.acceleration * dt;
  state.yaw_rate += state.yaw_acceleration * dt;
  return state;
}
