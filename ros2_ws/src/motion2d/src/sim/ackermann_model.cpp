#include "motion2d/sim/ackermann_model.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace motion2d {
namespace {
double speedAt(double speed,double force,const AckermannParameters & p,double t) {
  if(p.linear_drag==0) return speed+force/p.mass*t;
  const double change=-std::expm1(-p.linear_drag/p.mass*t);
  return speed+(force/p.linear_drag-speed)*change;
}
}
void validateAckermannParameters(const AckermannParameters & p) {
  for(double x:{p.wheelbase,p.mass,p.force_max,p.steering_max,p.steering_rate_max})
    if(!std::isfinite(x) || x<=0) throw std::invalid_argument("Positive finite Ackermann parameters required");
  if(!std::isfinite(p.linear_drag) || p.linear_drag<0 || p.steering_max>=1.5)
    throw std::invalid_argument("Nonnegative drag and steering_max < 1.5 rad required");
}
AckermannInput limitAckermannInput(const AckermannState & x,AckermannInput u,const AckermannParameters & p,double dt) {
  u.force=std::clamp(u.force,-p.force_max,p.force_max);
  const double low=std::max(-p.steering_rate_max,(-p.steering_max-x.steering)/dt);
  const double high=std::min(p.steering_rate_max,(p.steering_max-x.steering)/dt);
  u.steering_rate=std::clamp(u.steering_rate,low,high);return u;
}
AckermannState stepAckermann(const AckermannState & x,AckermannInput u,const AckermannParameters & p,double dt) {
  u=limitAckermannInput(x,u,p,dt);
  auto rhs=[&](const Eigen::Vector3d & q,double t)->Eigen::Vector3d {
    const double s=speedAt(x.speed,u.force,p,t),delta=x.steering+t*u.steering_rate;
    return {s*std::cos(q.z()),s*std::sin(q.z()),s*std::tan(delta)/p.wheelbase};
  };
  const Eigen::Vector3d q(x.pose.position.x(),x.pose.position.y(),x.pose.yaw);
  const Eigen::Vector3d k1=rhs(q,0),k2=rhs(q+.5*dt*k1,.5*dt),
    k3=rhs(q+.5*dt*k2,.5*dt),k4=rhs(q+dt*k3,dt);
  const Eigen::Vector3d next=q+dt/6*(k1+2*k2+2*k3+k4);
  return {{{next.x(),next.y()},wrapAngle(next.z())},speedAt(x.speed,u.force,p,dt),
    std::clamp(x.steering+dt*u.steering_rate,-p.steering_max,p.steering_max)};
}
// ackermann_kinematics_begin
State2D ackermannKinematics(const AckermannState & x,const AckermannInput & u,const AckermannParameters & p) {
  const Eigen::Vector2d e(std::cos(x.pose.yaw),std::sin(x.pose.yaw)),normal(-e.y(),e.x());
  const double along=(u.force-p.linear_drag*x.speed)/p.mass;
  const double tangent=std::tan(x.steering),omega=x.speed*tangent/p.wheelbase;
  State2D out;out.pose=x.pose;out.velocity=x.speed*e;
  out.acceleration=along*e+x.speed*omega*normal;out.yaw_rate=omega;
  out.yaw_acceleration=(along*tangent+x.speed*(1+tangent*tangent)*u.steering_rate)/p.wheelbase;
  return out;
}
// ackermann_kinematics_end
double ackermannSweepPadding(const AckermannState & x,AckermannInput u,const AckermannParameters & p,double dt) {
  u=limitAckermannInput(x,u,p,dt);
  const double speed=std::max(std::abs(x.speed),std::abs(speedAt(x.speed,u.force,p,dt)));
  const double angle=std::max(std::abs(x.steering),std::abs(x.steering+u.steering_rate*dt));
  const double acceleration=(std::abs(u.force)+p.linear_drag*speed)/p.mass+speed*speed*std::tan(angle)/p.wheelbase;
  return acceleration*dt*dt/8;
}
}
