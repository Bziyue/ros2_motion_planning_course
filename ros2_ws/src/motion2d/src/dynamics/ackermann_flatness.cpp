#include "motion2d/dynamics/ackermann_flatness.hpp"
#include <cmath>
#include <stdexcept>
namespace motion2d {
namespace {double cross(const Eigen::Vector2d & a,const Eigen::Vector2d & b){return a.x()*b.y()-a.y()*b.x();}}
AckermannFlatOutput ackermannForward(const AckermannFlatInput & x,const AckermannParameters & p,double minimum_speed) {
  const double n=x.velocity.norm();
  if(!x.velocity.allFinite() || !x.acceleration.allFinite() || !x.jerk.allFinite() || n<=minimum_speed)
    throw std::domain_error("Ackermann flat map needs a nonzero forward tangent");
  // ackermann_forward_begin
  const double b=x.velocity.dot(x.acceleration),h=cross(x.velocity,x.acceleration);
  const double hp=cross(x.velocity,x.jerk),k=h/std::pow(n,3);
  const double kdot=hp/std::pow(n,3)-3*h*b/std::pow(n,5);
  AckermannFlatOutput y;
  y.speed=n;y.yaw=std::atan2(x.velocity.y(),x.velocity.x());y.yaw_rate=h/(n*n);
  y.curvature=k;y.steering=std::atan(p.wheelbase*k);
  y.steering_rate=p.wheelbase*kdot/(1+std::pow(p.wheelbase*k,2));
  y.longitudinal_acceleration=b/n;y.lateral_acceleration=h/n;
  y.force=p.mass*y.longitudinal_acceleration+p.linear_drag*n;
  // ackermann_forward_end
  return y;
}
AckermannFlatInput ackermannBackward(const AckermannFlatInput & x,const AckermannFlatOutput & g,
  const AckermannParameters & p,double minimum_speed) {
  const auto y=ackermannForward(x,p,minimum_speed);
  const double n=y.speed,b=x.velocity.dot(x.acceleration),h=cross(x.velocity,x.acceleration),hp=cross(x.velocity,x.jerk);
  const double L=p.wheelbase,k=y.curvature,D=1+L*L*k*k,kdot=hp/std::pow(n,3)-3*h*b/std::pow(n,5);
  // Reverse the scalar intermediates in reverse dependency order.
  double gn=g.speed+p.linear_drag*g.force,gb=0,gh=0,ghp=0;
  const double galong=g.longitudinal_acceleration+p.mass*g.force;
  gb+=galong/n;gn-=galong*b/(n*n);
  gh+=g.yaw_rate/(n*n)+g.lateral_acceleration/n;
  gn-=2*g.yaw_rate*h/std::pow(n,3)+g.lateral_acceleration*h/(n*n);
  const double gkdot=L/D*g.steering_rate;
  const double gk=g.curvature+L/D*g.steering-2*L*L*L*k*kdot/(D*D)*g.steering_rate;
  ghp+=gkdot/std::pow(n,3);gh-=3*b/std::pow(n,5)*gkdot;gb-=3*h/std::pow(n,5)*gkdot;
  gn+=(-3*hp/std::pow(n,4)+15*h*b/std::pow(n,6))*gkdot;
  gh+=gk/std::pow(n,3);gn-=3*h/std::pow(n,4)*gk;
  AckermannFlatInput out;
  out.velocity=gn*x.velocity/n+gb*x.acceleration+gh*Eigen::Vector2d(x.acceleration.y(),-x.acceleration.x())+
    ghp*Eigen::Vector2d(x.jerk.y(),-x.jerk.x())+g.yaw/(n*n)*Eigen::Vector2d(-x.velocity.y(),x.velocity.x());
  out.acceleration=gb*x.velocity+gh*Eigen::Vector2d(-x.velocity.y(),x.velocity.x());
  out.jerk=ghp*Eigen::Vector2d(-x.velocity.y(),x.velocity.x());return out;
}
}
