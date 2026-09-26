#include "motion2d/ros/odometry_messages.hpp"
#include <cmath>

namespace motion2d
{
// truth_odometry_begin
nav_msgs::msg::Odometry truthOdometry(
  const State2D & state, const builtin_interfaces::msg::Time & stamp)
{
  nav_msgs::msg::Odometry message;
  message.header.stamp = stamp;
  message.header.frame_id = "world";
  message.child_frame_id = "ground_truth_base";
  message.pose.pose.position.x = state.pose.position.x();
  message.pose.pose.position.y = state.pose.position.y();
  message.pose.pose.orientation.z = std::sin(state.pose.yaw / 2);
  message.pose.pose.orientation.w = std::cos(state.pose.yaw / 2);
  const Eigen::Vector2d body_velocity =
    rotation(state.pose.yaw).transpose() * state.velocity;
  message.twist.twist.linear.x = body_velocity.x();
  message.twist.twist.linear.y = body_velocity.y();
  message.twist.twist.angular.z = state.yaw_rate;
  return message;
}
// truth_odometry_end
}  // namespace motion2d
