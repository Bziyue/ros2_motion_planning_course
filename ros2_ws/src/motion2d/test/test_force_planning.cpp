#include "motion2d/trajectory/trajectory_optimizer.hpp"
#include "motion2d/trajectory/spline2d.hpp"
#include "motion2d/trajectory/bezier_bounds.hpp"
#include <gtest/gtest.h>
using namespace motion2d;
namespace {
TrajectoryCostConfig costConfig() {
  TrajectoryCostConfig c;OmniDynamicLimits d;d.parameters.mass=1.7;d.parameters.linear_drag=.3;
  d.force=.7;d.force_rate=1.1;c.dynamics=d;c.force_weight=3;c.force_rate_weight=2;return c;
}
}
TEST(ForcePlanning, DirectCoefficientAndMovingTimePartials) {
  QuinticPiece p;p.duration=1.2;p.coefficients << .2,.1,.3,-.2,.1,-.02, -.1,.3,.2,.1,-.1,.04;
  const auto c=costConfig();
  const auto eval=[&](const QuinticPiece & q){return trajectoryCost(PolynomialTrajectory({q}),c);};
  const auto exact=eval(p);const double h=1e-6;
  for(int k=0;k<6;++k)for(int d=0;d<2;++d) {
    auto plus=p,minus=p;plus.coefficients(d,k)+=h;minus.coefficients(d,k)-=h;
    const double fd=(eval(plus).cost-eval(minus).cost)/(2*h);
    EXPECT_NEAR(fd,exact.coefficients(k,d),2e-6*std::max(1.,std::abs(fd)));
  }
  auto plus=p,minus=p;plus.duration+=h;minus.duration-=h;
  const double fd=(eval(plus).cost-eval(minus).cost)/(2*h);
  EXPECT_NEAR(fd,exact.times(0),2e-6*std::max(1.,std::abs(fd)));
}
template<class Curve> void checkAdjoint() {
  TranslationState a,b;b.position={2,.1};std::vector<Eigen::Vector2d> q{{.8,.4}};std::vector<double>T{1.4,1.7};
  const auto c=costConfig();const Curve curve(a,b,q,T);
  const auto partial=trajectoryCost(curve.trajectory(),c),g=curve.propagate(partial.coefficients,partial.times);
  for(int k=0;k<4;++k) {
    auto qp=q,qm=q;auto tp=T,tm=T;const double h=1e-6;
    if(k<2){qp[0](k)+=h;qm[0](k)-=h;}else{tp[k-2]+=h;tm[k-2]-=h;}
    const double fd=(trajectoryCost(Curve(a,b,qp,tp).trajectory(),c).cost-trajectoryCost(Curve(a,b,qm,tm).trajectory(),c).cost)/(2*h);
    EXPECT_NEAR(fd,k<2?g.waypoints(0,k):g.times(k-2),1e-5*std::max(1.,std::abs(fd)));
  }
}
TEST(ForcePlanning, MincoAndSplineAdjoints) {checkAdjoint<Minco2D>();checkAdjoint<Spline2D>();}
TEST(ForcePlanning, CertificateContainsDenseValuesAndRejectsOverload) {
  TranslationState a,b;b.position={1,.3};const auto curve=Minco2D(a,b,{}, {2}).trajectory();
  auto d=*costConfig().dynamics;auto cert=certifyForce(curve,d,5);EXPECT_FALSE(cert.certified);
  for(int j=0;j<=1000;++j) {
    const double t=2.*j/1000;const auto s=curve.sample(t);
    const auto f=omniForward({s.velocity,s.acceleration,curve.evaluate(t,3),0,0},d.parameters);
    EXPECT_LE(f.force.norm(),cert.force_bound+1e-9);EXPECT_LE(f.force_rate.norm(),cert.force_rate_bound+1e-9);
  }
  d.force=20;d.force_rate=20;EXPECT_TRUE(certifyForce(curve,d).certified);
  TrajectoryLimits limits;limits.speed=5;limits.acceleration=5;limits.dynamics=*costConfig().dynamics;
  EXPECT_FALSE(checkTrajectorySamples(curve,limits).samples_feasible);
}
TEST(ForcePlanning, TimeOptimizationCanRespectMassAndForce) {
  TranslationState a,b;b.position={2,.3};TrajectoryOptimizationConfig c;c.optimize_waypoints=false;
  c.cost.energy_weight=.01;c.solver.max_iterations=300;c.solver.gradient_tolerance=1e-5;
  auto d=*costConfig().dynamics;d.force=1;d.force_rate=2;c.limits.dynamics=d;
  d.force*=.85;d.force_rate*=.85;c.cost.dynamics=d;
  for(bool spline:{false,true}) {
    auto r=spline?optimizeSpline(a,b,{}, {2},c):optimizeMinco(a,b,{}, {2},c);
    ASSERT_TRUE(r.curve);EXPECT_TRUE(r.solver.converged())<<r.solver.status;
    EXPECT_TRUE(certifyForce(*r.curve,*c.limits.dynamics).certified);EXPECT_GT(r.durations[0],2);
  }
}
