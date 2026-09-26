#include "motion2d/dynamics/omni_flatness.hpp"
#include <gtest/gtest.h>
using namespace motion2d;
TEST(OmniFlatness, RecoversForceAndIndependentYawAtRest)
{
  InertialParameters p; p.mass=2; p.linear_drag=.3;p.inertia_z=.4;p.angular_drag=.2;
  OmniFlatInput x; x.acceleration={1,-2};x.jerk={-.5,.7};x.yaw_rate=.6;x.yaw_acceleration=-.8;
  const auto y=omniForward(x,p);
  EXPECT_TRUE(y.force.isApprox(Eigen::Vector2d(2,-4)));
  EXPECT_TRUE(y.force_rate.isApprox(Eigen::Vector2d(-.7,.8)));
  EXPECT_NEAR(y.torque,-.2,1e-14);
  // The map must not saturate, even though the simulator would clip this force.
  EXPECT_GT(std::abs(y.force.y()),p.force_max);
}
TEST(OmniFlatness, ReverseMatchesEveryInputPartial)
{
  InertialParameters p;p.mass=1.7;p.linear_drag=.4;p.inertia_z=.3;p.angular_drag=.13;
  OmniFlatInput x{{.7,-.3},{-.4,.9},{.2,-.6},.4,-.2};
  const OmniFlatOutput weights{{.6,-.8},{-.3,.5},.7};
  const auto g=omniBackward(weights,p);
  const auto cost=[&](const OmniFlatInput & z) {
    const auto y=omniForward(z,p);
    return y.force.dot(weights.force)+y.force_rate.dot(weights.force_rate)+y.torque*weights.torque;
  };
  for(int k=0;k<8;++k) {
    auto plus=x,minus=x;
    const auto entry=[k](OmniFlatInput & a)->double & {
      if(k<2)return a.velocity(k);if(k<4)return a.acceleration(k-2);
      if(k<6)return a.jerk(k-4);return k==6?a.yaw_rate:a.yaw_acceleration;
    };
    entry(plus)+=1e-6;entry(minus)-=1e-6;
    auto gradient=g;
    EXPECT_NEAR(entry(gradient),(cost(plus)-cost(minus))/2e-6,2e-9);
  }
}
TEST(OmniFlatness, ReconstructsPlantAcceleration)
{
  InertialParameters p;p.mass=2.3;p.linear_drag=.5;
  OmniFlatInput x{{.3,-.2},{.4,.1},{0,0},.2,.1};
  const auto y=omniForward(x,p);
  EXPECT_TRUE(((y.force-p.linear_drag*x.velocity)/p.mass).isApprox(x.acceleration));
  EXPECT_NEAR((y.torque-p.angular_drag*x.yaw_rate)/p.inertia_z,x.yaw_acceleration,1e-14);
}
