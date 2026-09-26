#pragma once
#include "motion2d/sim/robot_model.hpp"

/** @brief Undamped constant-force step; command is already limited, mass/inertia positive. */
inline motion2d::State2D studentForceStep(motion2d::State2D state,
  const motion2d::Wrench2D & applied, double mass, double inertia_z, double dt)
{
  // EXERCISE(ch04-3): update translation and yaw using the OLD velocity/rate.
  (void)applied;
  (void)mass;
  (void)inertia_z;
  (void)dt;
  return state;
}
