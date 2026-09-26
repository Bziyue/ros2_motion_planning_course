#include "motion2d/ros/navigation_messages.hpp"
#include "motion2d/ros/trajectory_messages.hpp"
#include <limits>
#include <stdexcept>
namespace motion2d {
NavigationReference fromNavigationMessage(const motion2d_interfaces::msg::NavigationReference & m) {
  NavigationReference out{fromTrajectoryMessage(m.motion),{},0};
  if(m.regions.size()!=out.motion.curve.pieces().size() || m.snapshot_time.sec<0 || m.snapshot_time.nanosec>=1000000000)
    throw std::invalid_argument("Invalid region count or snapshot time");
  out.snapshot_ns=std::int64_t(m.snapshot_time.sec)*1000000000+m.snapshot_time.nanosec;
  for(const auto & r:m.regions) {
    Polygon polygon;
    for(const auto & p:r.vertices) {
      if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || std::abs(p.z)>1e-9)
        throw std::invalid_argument("Invalid navigation polygon vertex");
      polygon.vertices.emplace_back(p.x,p.y);
    }
    out.regions.push_back(makeRegion(polygon));
  }
  return out;
}
motion2d_interfaces::msg::NavigationReference toNavigationMessage(const NavigationReference & p,const builtin_interfaces::msg::Time & stamp) {
  if(p.snapshot_ns<0 || p.snapshot_ns/1000000000>std::numeric_limits<std::int32_t>::max() || p.regions.size()!=p.motion.curve.pieces().size())
    throw std::invalid_argument("Invalid navigation snapshot or region count");
  motion2d_interfaces::msg::NavigationReference out;out.motion=toTrajectoryMessage(p.motion,stamp);
  out.snapshot_time.sec=p.snapshot_ns/1000000000;out.snapshot_time.nanosec=p.snapshot_ns%1000000000;
  for(const auto & r:p.regions) {
    motion2d_interfaces::msg::ConvexRegion2D region;
    for(const auto & v:r.polygon.vertices) {geometry_msgs::msg::Point point;point.x=v.x();point.y=v.y();region.vertices.push_back(point);}
    out.regions.push_back(region);
  }
  return out;
}
}
