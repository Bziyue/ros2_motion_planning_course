#pragma once
#include <Eigen/Core>
#include <cmath>
/** @brief Quotient rule on the nonzero-tangent branch. */
inline double curvatureRate(const Eigen::Vector2d & v,const Eigen::Vector2d & a,const Eigen::Vector2d & j)
{
  const double n=v.norm(),h=v.x()*a.y()-v.y()*a.x(),hp=v.x()*j.y()-v.y()*j.x();
  return hp/std::pow(n,3)-3*h*v.dot(a)/std::pow(n,5);
}
