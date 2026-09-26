#include <gtest/gtest.h>
#include "motion2d/sim/differential_drive.hpp"
using namespace motion2d;
TEST(DifferentialDrive,StraightReverseAndInPlace) {
  auto straight=stepDifferential({},4,4,2);EXPECT_NEAR(straight.pose.position.x(),.4,1e-12);EXPECT_NEAR(straight.pose.position.y(),0,1e-12);
  auto reverse=stepDifferential({},-4,-4,2);EXPECT_NEAR(reverse.pose.position.x(),-.4,1e-12);
  auto turn=stepDifferential({},-4,4,kPi/2);EXPECT_NEAR(turn.pose.position.norm(),0,1e-12);EXPECT_NEAR(turn.pose.yaw,kPi/2,1e-12);
}
TEST(DifferentialDrive,ExactArcAndNoLateralBodyVelocity) {
  // r=.05, track=.4, left2/right6 => v=.2, omega=.5, radius=.4.
  auto end=stepDifferential({},2,6,kPi);EXPECT_NEAR(end.pose.position.x(),.4,1e-12);EXPECT_NEAR(end.pose.position.y(),.4,1e-12);
  EXPECT_NEAR(end.pose.yaw,kPi/2,1e-12);EXPECT_NEAR((rotation(-end.pose.yaw)*end.velocity).y(),0,1e-12);
  EXPECT_NEAR(end.acceleration.norm(),.1,1e-12);
  State2D small;for(int i=0;i<100;++i) small=stepDifferential(small,2,6,kPi/100);
  EXPECT_LT((small.pose.position-end.pose.position).norm(),1e-12);
}
TEST(DifferentialDrive,InverseAndInvalidGeometry) {
  for(double v:{-.4,0.,.4}) for(double w:{-1.,0.,1.}) {
    const auto wheels=differentialWheels(v,w);const auto command=differentialVelocity(wheels[0],wheels[1]);
    EXPECT_NEAR(command.velocity.x(),v,1e-12);EXPECT_NEAR(command.yaw_rate,w,1e-12);EXPECT_TRUE(command.body_frame);
  }
  EXPECT_THROW(differentialVelocity(1,2,WheelGeometry{.05,0}),std::invalid_argument);
  EXPECT_THROW(stepDifferential({},1,2,-.1),std::invalid_argument);
}
