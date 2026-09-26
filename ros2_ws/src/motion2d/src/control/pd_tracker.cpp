#include "motion2d/control/pd_tracker.hpp"
#include <cmath>
#include <stdexcept>
namespace motion2d {
void validatePdGains(const PdGains & g) {
  if(!g.kp.allFinite() || !g.kd.allFinite() || (g.kp.array()<0).any() || (g.kd.array()<0).any() ||
    !std::isfinite(g.yaw_kp) || g.yaw_kp<0 || !std::isfinite(g.yaw_kd) || g.yaw_kd<0)
    throw std::invalid_argument("Finite nonnegative PD gains required");
}
PdCommand trackPd(const State2D & state,const State2D & reference,const PdGains & gains,const InertialParameters & model) {
  PdCommand result;
  const Eigen::Vector2d feedforward=gains.feedforward ? reference.acceleration : Eigen::Vector2d::Zero();
  const double yaw_feedforward=gains.feedforward ? reference.yaw_acceleration : 0;
  // pd_force_begin
  const Eigen::Vector2d acceleration=feedforward+
    gains.kp.cwiseProduct(reference.pose.position-state.pose.position)+
    gains.kd.cwiseProduct(reference.velocity-state.velocity);
  result.requested.force=model.mass*acceleration+model.linear_drag*state.velocity;
  const double alpha=yaw_feedforward+gains.yaw_kp*wrapAngle(reference.pose.yaw-state.pose.yaw)+
    gains.yaw_kd*(reference.yaw_rate-state.yaw_rate);
  result.requested.torque=model.inertia_z*alpha+model.angular_drag*state.yaw_rate;
  result.applied=limitWrench(result.requested,model);
  // pd_force_end
  result.saturated=(result.applied.force-result.requested.force).norm()>1e-12 ||
    std::abs(result.applied.torque-result.requested.torque)>1e-12;
  return result;
}
Wrench2D dampingBrake(const State2D & state,const InertialParameters & model,double rate) {
  if(!std::isfinite(rate) || rate<=0) throw std::invalid_argument("Positive braking rate required");
  return limitWrench({-model.mass*rate*state.velocity,-model.inertia_z*rate*state.yaw_rate},model);
}
}
