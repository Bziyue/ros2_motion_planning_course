#include "motion2d/trajectory/reference_schedule.hpp"
#include <stdexcept>
namespace motion2d {
NavigationReference transformReference(const NavigationReference & plan,const Pose2D & target) {
  NavigationReference out{{transformTrajectory(plan.motion.curve,target),plan.motion.start_ns,
    wrapAngle(plan.motion.yaw+target.yaw)},{},plan.snapshot_ns};
  for(const auto & region:plan.regions) {
    Polygon polygon;
    for(const auto & p:region.polygon.vertices) polygon.vertices.push_back(target.position+rotation(target.yaw)*p);
    out.regions.push_back(makeRegion(polygon));
  }
  return out;
}
void ReferenceSchedule::reset(const Pose2D & hold) {
  if(!hold.position.allFinite() || !std::isfinite(hold.yaw)) throw std::invalid_argument("Invalid reference anchor");
  hold_=hold;active_.reset();pending_.reset();
}
void ReferenceSchedule::advance(std::int64_t stamp) {
  if(pending_ && stamp>=pending_->motion.start_ns) {active_=std::move(pending_);pending_.reset();}
}
const NavigationReference * ReferenceSchedule::reference(std::int64_t stamp) const {
  if(pending_ && stamp>=pending_->motion.start_ns) return &*pending_;
  return active_ ? &*active_ : nullptr;
}
State2D ReferenceSchedule::sample(std::int64_t stamp) const {
  const auto * selected=reference(stamp);
  return selected ? sampleHeldTrajectory(selected->motion,stamp) : idealPose(hold_);
}
const ConvexRegion * ReferenceSchedule::region(std::int64_t stamp) const {
  const auto * selected=reference(stamp);
  if(!selected || selected->regions.empty()) return nullptr;
  double t=(stamp-selected->motion.start_ns)*1e-9;std::size_t index=0;
  const auto & pieces=selected->motion.curve.pieces();
  while(index+1<pieces.size() && t>=pieces[index].duration) {t-=pieces[index].duration;++index;}
  return &selected->regions[index];
}
ReferenceAcceptance ReferenceSchedule::accept(NavigationReference next,std::int64_t now) {
  advance(now);
  if(now<0 || next.motion.start_ns<now || next.snapshot_ns<0 || !std::isfinite(next.motion.yaw)) return {false,"invalid_or_past_time"};
  if(pending_) return {false,"handover_pending"};
  if(!next.regions.empty() && next.regions.size()!=next.motion.curve.pieces().size()) return {false,"region_count"};
  for(const auto & r:next.regions) if(!isValid(r.polygon)) return {false,"invalid_region"};
  // reference_handover_begin
  const auto old=sample(next.motion.start_ns);
  const auto first=next.motion.curve.sample(0);
  const auto last=next.motion.curve.sample(next.motion.curve.duration());
  if((first.position-old.pose.position).norm()>1e-7 ||
     (first.velocity-old.velocity).norm()>1e-7 ||
     (first.acceleration-old.acceleration).norm()>1e-7 ||
     std::abs(wrapAngle(next.motion.yaw-old.pose.yaw))>1e-7)
    return {false,"discontinuous_handover"};
  if(last.velocity.norm()>1e-7 || last.acceleration.norm()>1e-7)
    return {false,"nonstationary_end"};
  pending_=std::move(next);
  return {true,"accepted"};
  // reference_handover_end
}
}
