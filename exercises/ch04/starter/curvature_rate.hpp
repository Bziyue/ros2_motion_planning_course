#pragma once
#include <Eigen/Core>
/** @brief Time derivative of signed path curvature on a regular forward branch. */
inline double curvatureRate(const Eigen::Vector2d & v,const Eigen::Vector2d & a,const Eigen::Vector2d & j)
{
  // EXERCISE(ch04-6): differentiate cross(v,a)/|v|^3; v must be nonzero.
  (void)v;(void)a;(void)j;return 0;
}
