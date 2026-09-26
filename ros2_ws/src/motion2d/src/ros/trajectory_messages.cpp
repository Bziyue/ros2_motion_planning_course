#include "motion2d/ros/trajectory_messages.hpp"
#include <cmath>
#include <stdexcept>

namespace motion2d
{
TimedTrajectory fromTrajectoryMessage(const motion2d_interfaces::msg::Trajectory2D & message)
{
  const auto valid_time=[](const builtin_interfaces::msg::Time & t) {return t.sec>=0 && t.nanosec<1000000000;};
  if(message.header.frame_id!="odom" || !valid_time(message.header.stamp) ||
    !valid_time(message.start_time) || !std::isfinite(message.yaw))
  {throw std::invalid_argument("Expected odom, nonnegative ROS time and finite yaw");}
  std::vector<QuinticPiece> pieces;
  for(const auto & source : message.pieces) {
    QuinticPiece p; p.duration=source.duration;
    for(int k=0;k<6;++k) {p.coefficients(0,k)=source.x[k]; p.coefficients(1,k)=source.y[k];}
    pieces.push_back(p);
  }
  return {PolynomialTrajectory(std::move(pieces)),
    std::int64_t(message.start_time.sec)*1000000000+message.start_time.nanosec,wrapAngle(message.yaw)};
}

motion2d_interfaces::msg::Trajectory2D toTrajectoryMessage(const TimedTrajectory & trajectory,
  const builtin_interfaces::msg::Time & published)
{
  motion2d_interfaces::msg::Trajectory2D message;
  message.header.frame_id="odom"; message.header.stamp=published;
  message.start_time.sec=trajectory.start_ns/1000000000;
  message.start_time.nanosec=trajectory.start_ns%1000000000;
  message.yaw=trajectory.yaw;
  for(const auto & piece : trajectory.curve.pieces()) {
    motion2d_interfaces::msg::QuinticPiece2D p; p.duration=piece.duration;
    for(int k=0;k<6;++k) {p.x[k]=piece.coefficients(0,k); p.y[k]=piece.coefficients(1,k);}
    message.pieces.push_back(p);
  }
  return message;
}
}  // namespace motion2d
