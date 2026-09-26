#include <gtest/gtest.h>
#include "motion2d/control/pd_tracker.hpp"
#include "motion2d/control/tracking_reference.hpp"
using namespace motion2d;
TEST(Pd, FeedforwardUsesMassDragAndIndependentYaw) {
  State2D s;s.velocity={.4,-.3};s.acceleration={.2,.5};s.yaw_rate=.1;s.yaw_acceleration=.2;
  InertialParameters p;p.mass=1.5;p.linear_drag=.3;p.inertia_z=.04;p.angular_drag=.02;
  auto r=trackPd(s,s,{},p);EXPECT_FALSE(r.saturated);
  EXPECT_LT(((r.applied.force-p.linear_drag*s.velocity)/p.mass-s.acceleration).norm(),1e-12);
  EXPECT_NEAR((r.applied.torque-p.angular_drag*s.yaw_rate)/p.inertia_z,s.yaw_acceleration,1e-12);
  State2D target=s;s.pose.yaw=kPi-.01;target.pose.yaw=-kPi+.01;
  EXPECT_NEAR(trackPd(s,target,{},p).requested.torque-r.requested.torque,.04*4*.02,1e-12);
}
TEST(Pd, LimitsAndBrakeArePhysicalInputs) {
  State2D s,ref;ref.pose.position={100,-100};s.velocity={2,-1};s.yaw_rate=.4;
  InertialParameters p;p.force_max=.3;p.torque_max=.01;
  auto command=trackPd(s,ref,{},p);EXPECT_TRUE(command.saturated);
  EXPECT_DOUBLE_EQ(command.applied.force.x(),.3);EXPECT_DOUBLE_EQ(command.applied.force.y(),-.3);
  const auto brake=dampingBrake(s,p);EXPECT_LT(brake.force.dot(s.velocity),0);EXPECT_LT(brake.torque*s.yaw_rate,0);
  EXPECT_LE(brake.force.cwiseAbs().maxCoeff(),.3);
}
TEST(Reference, SmoothStartAndAnalyticDerivatives) {
  TrackingReferenceConfig c;c.origin={{.2,-.3},.4};
  for(auto shape:{ReferenceShape::Circle,ReferenceShape::FigureEight}) {
    c.shape=shape;const auto start=sampleTrackingReference(c,0);
    EXPECT_EQ(start.velocity.norm(),0);EXPECT_EQ(start.acceleration.norm(),0);EXPECT_EQ(start.yaw_rate,0);
    for(double t:{.2,1.4,2.,3.4}) {
      const double h=1e-5;auto p=sampleTrackingReference(c,t+h),m=sampleTrackingReference(c,t-h),v=sampleTrackingReference(c,t);
      EXPECT_LT(((p.pose.position-m.pose.position)/(2*h)-v.velocity).norm(),1e-8);
      EXPECT_LT(((p.velocity-m.velocity)/(2*h)-v.acceleration).norm(),3e-6);
      EXPECT_NEAR(wrapAngle(p.pose.yaw-m.pose.yaw)/(2*h),v.yaw_rate,1e-8);
      EXPECT_NEAR((p.yaw_rate-m.yaw_rate)/(2*h),v.yaw_acceleration,1e-6);
    }
  }
}
