#include "motion2d/dynamics/ackermann_flatness.hpp"
#include <gtest/gtest.h>
using namespace motion2d;
TEST(Ackermann, CircleAndAnalyticSensorDerivatives) {
  AckermannParameters p;p.mass=2;p.linear_drag=.2;
  const double s=.6,R=1.2,omega=s/R;AckermannState state;state.speed=s;state.steering=std::atan(p.wheelbase/R);
  const AckermannInput input{p.linear_drag*s,0};
  for(int i=0;i<1000;++i)state=stepAckermann(state,input,p,.005);
  EXPECT_NEAR(state.pose.position.x(),R*std::sin(omega*5),1e-10);
  EXPECT_NEAR(state.pose.position.y(),R*(1-std::cos(omega*5)),1e-10);
  const auto sensed=ackermannKinematics(state,input,p);
  EXPECT_NEAR(sensed.acceleration.norm(),s*s/R,1e-12);EXPECT_NEAR(sensed.yaw_rate,omega,1e-12);
  AckermannFlatInput x{{s,0},{0,s*omega},{-s*omega*omega,0}};
  const auto flat=ackermannForward(x,p);
  EXPECT_NEAR(flat.steering,state.steering,1e-12);EXPECT_NEAR(flat.force,input.force,1e-12);
  EXPECT_NEAR(flat.steering_rate,0,1e-12);
}
TEST(Ackermann, ReverseAllFieldsMatchesFiniteDifferences) {
  AckermannParameters p;p.mass=1.7;p.linear_drag=.3;p.wheelbase=.4;
  AckermannFlatInput x{{.7,.3},{-.2,.6},{.4,-.1}};
  AckermannFlatOutput weights{.2,-.3,.4,.7,-.6,.8,.5,-.4,.3};
  auto cost=[&](const AckermannFlatInput & z) {
    const auto y=ackermannForward(z,p);
    return y.speed*weights.speed+y.yaw*weights.yaw+y.yaw_rate*weights.yaw_rate+y.curvature*weights.curvature+
      y.steering*weights.steering+y.steering_rate*weights.steering_rate+y.force*weights.force+
      y.longitudinal_acceleration*weights.longitudinal_acceleration+y.lateral_acceleration*weights.lateral_acceleration;
  };
  const auto g=ackermannBackward(x,weights,p);
  for(int k=0;k<6;++k) {
    auto plus=x,minus=x;
    const auto entry=[k](AckermannFlatInput & z)->double & {
      if(k<2)return z.velocity(k);if(k<4)return z.acceleration(k-2);return z.jerk(k-4);
    };
    entry(plus)+=1e-6;entry(minus)-=1e-6;auto gradient=g;
    EXPECT_NEAR(entry(gradient),(cost(plus)-cost(minus))/2e-6,2e-8);
  }
}
TEST(Ackermann, FlatMapReconstructsPlant) {
  AckermannParameters p;AckermannFlatInput x{{.8,-.4},{.2,.5},{.3,-.2}};
  const auto y=ackermannForward(x,p);
  const auto state=ackermannKinematics({{{0,0},y.yaw},y.speed,y.steering},{y.force,y.steering_rate},p);
  EXPECT_TRUE(state.velocity.isApprox(x.velocity,1e-12));EXPECT_TRUE(state.acceleration.isApprox(x.acceleration,1e-12));
  const double h=1e-6;auto plus=x,minus=x;
  plus.velocity+=h*x.acceleration;plus.acceleration+=h*x.jerk;
  minus.velocity-=h*x.acceleration;minus.acceleration-=h*x.jerk;
  EXPECT_NEAR(state.yaw_acceleration,(ackermannForward(plus,p).yaw_rate-ackermannForward(minus,p).yaw_rate)/(2*h),1e-8);
}
TEST(Ackermann, StopsAndBoundsHaveExplicitSemantics) {
  AckermannParameters p;AckermannState x;
  EXPECT_THROW(ackermannForward({},p),std::domain_error);
  auto next=stepAckermann(x,{0,100},p,.005);
  EXPECT_DOUBLE_EQ(next.pose.yaw,0);EXPECT_DOUBLE_EQ(next.speed,0);EXPECT_NEAR(next.steering,.005,1e-15);
  x.steering=p.steering_max-.001;
  next=stepAckermann(x,{0,10},p,.005);EXPECT_NEAR(next.steering,p.steering_max,1e-15);
  p.mass=2;p.linear_drag=0;x={};
  next=stepAckermann(x,{100,0},p,.01);EXPECT_NEAR(next.speed,.01,1e-15);EXPECT_NEAR(next.pose.position.x(),.00005,1e-15);
}
TEST(Ackermann, HeldInputRefinementAndPadding) {
  AckermannParameters p;AckermannState x{{{.2,.3},.4},.8,.2};AckermannInput u{.5,.7};
  auto whole=stepAckermann(x,u,p,.02),half=stepAckermann(stepAckermann(x,u,p,.01),u,p,.01);
  EXPECT_LT((whole.pose.position-half.pose.position).norm(),1e-9);
  const double pad=ackermannSweepPadding(x,u,p,.02);
  for(int k=1;k<20;++k) {
    const double s=k/20.;auto mid=stepAckermann(x,u,p,s*.02);
    EXPECT_LE((mid.pose.position-((1-s)*x.pose.position+s*whole.pose.position)).norm(),pad+1e-10);
  }
}
