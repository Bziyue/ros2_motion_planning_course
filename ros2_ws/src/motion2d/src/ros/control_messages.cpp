#include "motion2d/ros/control_messages.hpp"
#include "motion2d/ros/mapping_messages.hpp"
#include <cmath>
#include <stdexcept>
namespace motion2d {
State2D stateFromOdometry(const nav_msgs::msg::Odometry & m) {
  State2D state;state.pose=poseFromOdometry(m);const auto & t=m.twist.twist;
  if(!std::isfinite(t.linear.x) || !std::isfinite(t.linear.y) || !std::isfinite(t.angular.z) ||
    t.linear.z!=0 || t.angular.x!=0 || t.angular.y!=0) throw std::invalid_argument("Finite planar odometry twist required");
  state.velocity=rotation(state.pose.yaw)*Eigen::Vector2d(t.linear.x,t.linear.y);state.yaw_rate=t.angular.z;return state;
}
nav_msgs::msg::Odometry referenceOdometry(const State2D & s,const builtin_interfaces::msg::Time & stamp) {
  nav_msgs::msg::Odometry m;m.header.frame_id="odom";m.header.stamp=stamp;m.child_frame_id="reference_base";
  m.pose.pose.position.x=s.pose.position.x();m.pose.pose.position.y=s.pose.position.y();
  m.pose.pose.orientation.z=std::sin(s.pose.yaw/2);m.pose.pose.orientation.w=std::cos(s.pose.yaw/2);
  const Eigen::Vector2d v=rotation(s.pose.yaw).transpose()*s.velocity;
  m.twist.twist.linear.x=v.x();m.twist.twist.linear.y=v.y();m.twist.twist.angular.z=s.yaw_rate;return m;
}
}
