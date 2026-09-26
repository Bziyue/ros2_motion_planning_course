#include "motion2d/ros/imu_messages.hpp"
#include <cmath>
#include <iomanip>
#include <sstream>

namespace motion2d
{
// imu_message_begin
sensor_msgs::msg::Imu toImuMessage(const ImuSample & sample, const ImuNoise & noise,
  const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::Imu message;
  message.header.stamp = stamp;
  message.header.frame_id = "imu_link";
  message.orientation.w = 1.0;  // Placeholder, never a heading measurement.
  message.orientation_covariance[0] = -1.0;
  message.angular_velocity.x = sample.angular_velocity.x();
  message.angular_velocity.y = sample.angular_velocity.y();
  message.angular_velocity.z = sample.angular_velocity.z();
  message.linear_acceleration.x = sample.specific_force.x();
  message.linear_acceleration.y = sample.specific_force.y();
  message.linear_acceleration.z = sample.specific_force.z();
  for (int axis = 0; axis < 3; ++axis) {
    message.angular_velocity_covariance[4 * axis] = std::pow(noise.gyro_stddev[axis], 2);
    message.linear_acceleration_covariance[4 * axis] = std::pow(noise.accel_stddev[axis], 2);
  }
  return message;
}
// imu_message_end

visualization_msgs::msg::MarkerArray imuMarkers(
  const sensor_msgs::msg::Imu & message, const Pose2D & pose)
{
  using Marker = visualization_msgs::msg::Marker;
  Marker arrow;
  arrow.header = message.header;
  arrow.header.frame_id = "odom";
  arrow.ns = "imu_specific_force_xy";
  arrow.type = Marker::ARROW;
  arrow.pose.orientation.w = 1;
  arrow.scale.x = .06;
  arrow.scale.y = .14;
  arrow.scale.z = .16;
  arrow.color.r = .65F;
  arrow.color.g = .1F;
  arrow.color.b = .85F;
  arrow.color.a = 1;
  geometry_msgs::msg::Point origin;
  origin.x = pose.position.x();
  origin.y = pose.position.y();
  origin.z = .22;
  auto endpoint = origin;
  const Eigen::Vector2d world_force = rotation(pose.yaw) *
    Eigen::Vector2d(message.linear_acceleration.x, message.linear_acceleration.y);
  endpoint.x += 3 * world_force.x();
  endpoint.y += 3 * world_force.y();
  arrow.points = {origin, endpoint};
  arrow.action = world_force.squaredNorm() > 1e-16 ? Marker::ADD : Marker::DELETE;
  Marker label = arrow;
  label.ns = "imu_raw_label";
  label.type = Marker::TEXT_VIEW_FACING;
  label.action = Marker::ADD;
  label.points.clear();
  label.pose.position = origin;
  label.pose.position.y += 1.1;
  label.scale.z = .22;
  std::ostringstream text;
  text << std::fixed << std::setprecision(2)
       << "raw IMU: fz=" << message.linear_acceleration.z << " m/s^2"
       << "\nwz=" << message.angular_velocity.z << " rad/s; XY arrow x3";
  label.text = text.str();
  visualization_msgs::msg::MarkerArray markers;
  markers.markers = {arrow, label};
  return markers;
}
}  // namespace motion2d
