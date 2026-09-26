#pragma once
#include <cmath>
#include <limits>
/** @brief Choose the entry root, or exit when the origin lies inside the circle. */
inline double studentCircleHit(double ox,double oy,double dx,double dy,double cx,double cy,double r) {
  const double x=cx-ox,y=cy-oy,h=x*dx+y*dy,perpendicular=dx*y-dy*x;
  const double discriminant=r*r-perpendicular*perpendicular,miss=std::numeric_limits<double>::infinity();
  if(discriminant<0) return miss;
  const double root=std::sqrt(discriminant);
  return h-root>=0 ? h-root : h+root>=0 ? h+root : miss;
}
