#include "motion2d/control/tracking_reference.hpp"
#include <cmath>
#include <stdexcept>
namespace motion2d {
void validateTrackingReference(const TrackingReferenceConfig & c) {
  if(!c.origin.position.allFinite() || !std::isfinite(c.origin.yaw) || !std::isfinite(c.scale) || c.scale<=0 ||
    !std::isfinite(c.omega) || c.omega<=0 || !std::isfinite(c.ramp_seconds) || c.ramp_seconds<=0 ||
    (c.shape!=ReferenceShape::Circle && c.shape!=ReferenceShape::FigureEight))
    throw std::invalid_argument("Invalid tracking reference");
}
State2D sampleTrackingReference(const TrackingReferenceConfig & c,double time) {
  if(!std::isfinite(time) || time<0) throw std::invalid_argument("Finite nonnegative reference time required");
  double tau=time-c.ramp_seconds/2,rate=1,acceleration=0;
  if(time<c.ramp_seconds) {
    const double s=time/c.ramp_seconds;
    tau=c.ramp_seconds*(s*s*s-.5*s*s*s*s);rate=3*s*s-2*s*s*s;acceleration=6*s*(1-s)/c.ramp_seconds;
  }
  const double phase=c.omega*tau,w=c.omega*rate,alpha=c.omega*acceleration,r=c.scale;
  Eigen::Vector2d p,dp,ddp;
  if(c.shape==ReferenceShape::Circle) {
    p={r*std::sin(phase),r*(1-std::cos(phase))};dp={r*std::cos(phase),r*std::sin(phase)};ddp={-r*std::sin(phase),r*std::cos(phase)};
  } else {
    p={r*std::sin(phase),.5*r*std::sin(2*phase)};dp={r*std::cos(phase),r*std::cos(2*phase)};ddp={-r*std::sin(phase),-2*r*std::sin(2*phase)};
  }
  State2D result;const auto R=rotation(c.origin.yaw);
  result.pose.position=c.origin.position+R*p;result.velocity=R*(w*dp);result.acceleration=R*(w*w*ddp+alpha*dp);
  result.pose.yaw=wrapAngle(c.origin.yaw+.4*std::sin(phase/2));
  result.yaw_rate=.2*std::cos(phase/2)*w;
  result.yaw_acceleration=-.1*std::sin(phase/2)*w*w+.2*std::cos(phase/2)*alpha;
  return result;
}
}
