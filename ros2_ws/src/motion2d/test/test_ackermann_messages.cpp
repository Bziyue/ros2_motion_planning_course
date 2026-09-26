#include "motion2d/ros/ackermann_messages.hpp"
#include "motion2d/trajectory/quintic.hpp"
#include <gtest/gtest.h>
using namespace motion2d;
TEST(AckermannMessages, RoundTripPreservesGeometryAndPhysicalTime) {
  TranslationState a,b;a.velocity={2,0};b.position={2,.3};b.velocity={2,0};
  AckermannTrajectory curve({{interpolateQuintic(a,b,1),5}});
  const auto message=toAckermannMessage({curve,1230000000},builtin_interfaces::msg::Time{});const auto recovered=fromAckermannMessage(message);
  EXPECT_EQ(recovered.start_ns,1230000000);EXPECT_EQ(recovered.curve.duration(),5);
  for(double t:{0.,.2,2.,5.})EXPECT_TRUE(recovered.curve.sample(t,{}).state.pose.position.isApprox(curve.sample(t,{}).state.pose.position));
  const Pose2D tf{{1,2},.5};const auto moved=transformAckermann(curve,tf);
  EXPECT_TRUE(moved.sample(2,{}).state.pose.position.isApprox(transformPoint(tf,curve.sample(2,{}).state.pose.position)));
  EXPECT_TRUE(moved.sample(2,{}).state.velocity.isApprox(rotation(tf.yaw)*curve.sample(2,{}).state.velocity));
  auto bad=message;bad.header.frame_id="map";EXPECT_THROW(fromAckermannMessage(bad),std::invalid_argument);
  bad=message;bad.pieces[0].duration=0;EXPECT_THROW(fromAckermannMessage(bad),std::invalid_argument);
  bad=message;bad.pieces[0].x[1]=0;bad.pieces[0].y[1]=0;EXPECT_THROW(fromAckermannMessage(bad),std::invalid_argument);
}
