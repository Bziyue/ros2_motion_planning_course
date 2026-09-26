#include "motion2d/control/ackermann_tracker.hpp"
#include "motion2d/trajectory/quintic.hpp"
#include <gtest/gtest.h>
using namespace motion2d;
namespace {
AckermannPiece piece(double T=5) {
  TranslationState a,b;a.velocity={2,0};b.position={2,.3};b.velocity={2,0};
  return {interpolateQuintic(a,b,1),T};
}
ConvexRegion room(){return makeRegion({{{-1,-1},{3,-1},{3,2},{-1,2}}});}
double dot(const AckermannFlatOutput & a,const AckermannFlatOutput & b) {
  return a.speed*b.speed+a.yaw*b.yaw+a.yaw_rate*b.yaw_rate+a.curvature*b.curvature+a.steering*b.steering+
    a.steering_rate*b.steering_rate+a.force*b.force+a.longitudinal_acceleration*b.longitudinal_acceleration+a.lateral_acceleration*b.lateral_acceleration;
}
}
TEST(AckermannPlanning, WarpAndReverseIncludingStops) {
  AckermannParameters p;AckermannFlatInput x{{.8,.4},{-.2,.3},{.5,-.1}};const auto q=ackermannForward(x,p);
  ProgressSample h{.4,.7,-.3};AckermannFlatOutput g{.2,-.3,.4,.7,-.6,.8,.5,-.4,.3};
  const auto back=warpAckermannBackward(q,h,g,p);const auto flat=ackermannBackward(x,back.geometry,p);
  const auto cost=[&](AckermannFlatInput x,ProgressSample h){return dot(warpAckermann(ackermannForward(x,p),h,p),g);};
  for(int k=0;k<8;++k) {
    auto xp=x,xm=x;auto hp=h,hm=h;double exact=0;
    if(k<2){xp.velocity(k)+=1e-6;xm.velocity(k)-=1e-6;exact=flat.velocity(k);}
    else if(k<4){xp.acceleration(k-2)+=1e-6;xm.acceleration(k-2)-=1e-6;exact=flat.acceleration(k-2);}
    else if(k<6){xp.jerk(k-4)+=1e-6;xm.jerk(k-4)-=1e-6;exact=flat.jerk(k-4);}
    else if(k==6){hp.rate+=1e-6;hm.rate-=1e-6;exact=back.rate;}
    else {hp.acceleration+=1e-6;hm.acceleration-=1e-6;exact=back.acceleration;}
    EXPECT_NEAR(exact,(cost(xp,hp)-cost(xm,hm))/2e-6,2e-8);
  }
  const auto stop=warpAckermann(q,{},p);EXPECT_EQ(stop.speed,0);EXPECT_EQ(stop.force,0);
  EXPECT_EQ(stop.steering,q.steering);EXPECT_EQ(stop.yaw,q.yaw);
}
TEST(AckermannPlanning, DirectCoefficientAndTimeGradient) {
  auto q=piece(2);AckermannLimits l;l.force=.4;l.steering=.15;l.steering_rate=.2;l.speed=.3;
  const auto r=room();const auto exact=ackermannPieceCost(q,l,&r);
  for(int k=0;k<6;++k)for(int d=0;d<2;++d) {
    auto plus=q,minus=q;plus.geometry.coefficients(d,k)+=1e-6;minus.geometry.coefficients(d,k)-=1e-6;
    const double fd=(ackermannPieceCost(plus,l,&r).cost-ackermannPieceCost(minus,l,&r).cost)/2e-6;
    EXPECT_NEAR(fd,exact.coefficients(k,d),1e-5*std::max(1.,std::abs(fd)));
  }
  auto plus=q,minus=q;plus.duration+=1e-6;minus.duration-=1e-6;
  const double fd=(ackermannPieceCost(plus,l,&r).cost-ackermannPieceCost(minus,l,&r).cost)/2e-6;
  EXPECT_NEAR(fd,exact.time,1e-5*std::max(1.,std::abs(fd)));
}
TEST(AckermannPlanning, ContinuousBoundsContainSamplesAndTimingKeepsSteering) {
  const AckermannTrajectory curve({piece()}),slower({piece(10)});AckermannLimits l;
  const auto cert=certifyAckermann(curve,{room()},l);EXPECT_TRUE(cert.geometry_valid);
  for(int k=0;k<=1000;++k) {
    const auto y=curve.sample(5.*k/1000,l.vehicle).dynamics,z=slower.sample(10.*k/1000,l.vehicle).dynamics;
    EXPECT_NEAR(y.steering,z.steering,1e-12);EXPECT_LE(y.speed,cert.speed+1e-9);
    EXPECT_LE(std::abs(y.force),cert.force+1e-9);EXPECT_LE(std::abs(y.steering),cert.steering+1e-9);
    EXPECT_LE(std::abs(y.steering_rate),cert.steering_rate+1e-9);EXPECT_LE(std::abs(y.lateral_acceleration),cert.lateral_acceleration+1e-9);
  }
  l.steering=.01;EXPECT_FALSE(certifyAckermann(slower,{room()},l).geometry_valid);
  EXPECT_EQ(curve.sample(0,l.vehicle).state.velocity.norm(),0);EXPECT_LT(curve.sample(5,l.vehicle).state.acceleration.norm(),1e-14);
}
TEST(AckermannPlanning, OptimizedLaneChangeTracksFromRest) {
  AckermannPlanningConfig c;c.solver.max_iterations=400;c.solver.gradient_tolerance=1e-5;
  auto r=planAckermann({{{0,0},0},{{2,.3},0}},{5},{room()},c);
  ASSERT_TRUE(r.curve)<<r.status;EXPECT_TRUE(r.certificate.certified);
  AckermannState x;double peak=0;
  for(double t=0;t<r.curve->duration()+3;t+=.005) {
    const auto ref=r.curve->sample(t,c.limits.vehicle);
    const auto u=trackAckermann(x,ref,c.limits.vehicle,{},.005);
    peak=std::max(peak,(ref.state.pose.position-x.pose.position).norm());
    x=stepAckermann(x,u,c.limits.vehicle,.005);
  }
  EXPECT_LT(peak,.03);EXPECT_LT((x.pose.position-Eigen::Vector2d(2,.3)).norm(),.015);EXPECT_LT(std::abs(x.speed),.005);
}
TEST(AckermannPlanning, IncompatibleCorridorReturnsNoExecutableCurve) {
  AckermannPlanningConfig c;c.solver.max_iterations=100;
  auto narrow=makeRegion({{{-.1,-.02},{2.1,-.02},{2.1,.02},{-.1,.02}}});
  const auto r=planAckermann({{{0,0},0},{{2,.3},0}},{5},{narrow},c);EXPECT_FALSE(r.curve);
}
