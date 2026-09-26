#include "motion2d/ros/ackermann_messages.hpp"
#include <stdexcept>
namespace motion2d {
AckermannTrajectory transformAckermann(const AckermannTrajectory & curve,const Pose2D & tf) {
  auto pieces=curve.pieces();
  for(auto & piece:pieces){piece.geometry.coefficients=rotation(tf.yaw)*piece.geometry.coefficients;piece.geometry.coefficients.col(0)+=tf.position;}
  return AckermannTrajectory(std::move(pieces));
}
motion2d_interfaces::msg::AckermannTrajectory2D toAckermannMessage(const TimedAckermannTrajectory & trajectory,
  const builtin_interfaces::msg::Time & stamp) {
  motion2d_interfaces::msg::AckermannTrajectory2D out;out.header.frame_id="odom";out.header.stamp=stamp;
  out.start_time.sec=trajectory.start_ns/1000000000;out.start_time.nanosec=trajectory.start_ns%1000000000;
  for(const auto & p:trajectory.curve.pieces()) {
    motion2d_interfaces::msg::AckermannPiece2D m;m.duration=p.duration;
    for(int k=0;k<6;++k){m.x[k]=p.geometry.coefficients(0,k);m.y[k]=p.geometry.coefficients(1,k);}
    out.pieces.push_back(m);
  }
  return out;
}
TimedAckermannTrajectory fromAckermannMessage(const motion2d_interfaces::msg::AckermannTrajectory2D & m) {
  if(m.header.frame_id!="odom" || m.start_time.sec<0 || m.start_time.nanosec>=1000000000 || m.pieces.empty() || m.pieces.size()>100)
    throw std::invalid_argument("Expected bounded nonempty Ackermann curve in odom with nonnegative start");
  std::vector<AckermannPiece> pieces;
  for(const auto & p:m.pieces) {
    AckermannPiece q;q.duration=p.duration;q.geometry.duration=1;
    for(int k=0;k<6;++k){q.geometry.coefficients(0,k)=p.x[k];q.geometry.coefficients(1,k)=p.y[k];}
    pieces.push_back(q);
  }
  return {AckermannTrajectory(std::move(pieces)),std::int64_t(m.start_time.sec)*1000000000+m.start_time.nanosec};
}
}
