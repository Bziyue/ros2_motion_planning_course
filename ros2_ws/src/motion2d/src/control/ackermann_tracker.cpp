#include "motion2d/control/ackermann_tracker.hpp"
#include <cmath>
#include <stdexcept>
namespace motion2d {
void validateAckermannGains(const AckermannGains & g) {
  for(double x:{g.position,g.speed,g.yaw,g.lateral,g.steering,g.soft_speed})
    if(!std::isfinite(x) || x<=0)throw std::invalid_argument("Positive finite Ackermann gains required");
}
// ackermann_tracking_begin
AckermannInput trackAckermann(const AckermannState & x,const AckermannReference & ref,
  const AckermannParameters & p,const AckermannGains & g,double dt) {
  const Eigen::Vector2d e(std::cos(x.pose.yaw),std::sin(x.pose.yaw)),left(-e.y(),e.x());
  const Eigen::Vector2d error=ref.state.pose.position-x.pose.position;
  const double steering=ref.dynamics.steering+g.yaw*wrapAngle(ref.state.pose.yaw-x.pose.yaw)+
    std::atan2(g.lateral*left.dot(error),std::abs(x.speed)+g.soft_speed);
  AckermannInput u;
  u.force=p.mass*(ref.dynamics.longitudinal_acceleration+g.position*e.dot(error)+
    g.speed*(e.dot(ref.state.velocity)-x.speed))+p.linear_drag*x.speed;
  u.steering_rate=ref.dynamics.steering_rate+g.steering*(steering-x.steering);
  return limitAckermannInput(x,u,p,dt);
}
// ackermann_tracking_end
}
