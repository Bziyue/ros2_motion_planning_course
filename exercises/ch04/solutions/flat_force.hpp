#pragma once
#include <Eigen/Core>
/** @brief Chain rule for 0.5*|F|^2 with F=m*a+c*v. */
inline Eigen::Vector2d forceGradient(double mass,double drag,
  const Eigen::Vector2d & velocity,const Eigen::Vector2d & acceleration)
{return mass*(mass*acceleration+drag*velocity);}
