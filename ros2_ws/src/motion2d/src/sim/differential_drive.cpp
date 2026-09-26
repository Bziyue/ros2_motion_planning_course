#include "motion2d/sim/differential_drive.hpp"
#include <cmath>
#include <stdexcept>
namespace motion2d {
namespace {
void validate(const WheelGeometry & g,double a,double b) {
  if(!std::isfinite(g.radius) || !std::isfinite(g.track) || g.radius<=0 || g.track<=0 || !std::isfinite(a) || !std::isfinite(b))
    throw std::invalid_argument("Need positive wheel geometry and finite rates");
}
}
// differential_kinematics_begin
VelocityCommand differentialVelocity(double left,double right,const WheelGeometry & g) {
  validate(g,left,right);
  return {{g.radius*(right+left)/2,0},g.radius*(right-left)/g.track,true};
}
std::array<double,2> differentialWheels(double speed,double yaw_rate,const WheelGeometry & g) {
  validate(g,speed,yaw_rate);
  return {(speed-g.track*yaw_rate/2)/g.radius,(speed+g.track*yaw_rate/2)/g.radius};
}
// differential_kinematics_end
State2D stepDifferential(const State2D & state,double left,double right,double dt,const WheelGeometry & g) {
  if(!std::isfinite(dt) || dt<0) throw std::invalid_argument("Need nonnegative finite dt");
  return stepVelocity(state,differentialVelocity(left,right,g),dt);
}
}
