#include <gtest/gtest.h>
#include "motion2d/trajectory/spline2d.hpp"
#include "motion2d/trajectory/minco2d.hpp"
#include "motion2d/trajectory/trajectory_cost.hpp"
#include <random>
using namespace motion2d;
TEST(Spline, SameClampedProblemAndGeneralAdjoint) {
  std::mt19937 rng(1616);std::uniform_real_distribution<double> u(-1,1);
  for(int n=1;n<=12;++n) {
    TranslationState a,b;a.velocity={.2,-.1};a.acceleration={.1,.3};b.position={4,-.4};b.velocity={-.1,.3};b.acceleration={.2,.1};
    std::vector<Eigen::Vector2d> q;std::vector<double> T;
    for(int i=0;i<n;++i) {T.push_back(1.2+.5*u(rng));if(i<n-1) q.push_back({double(i+1)*4/n,u(rng)});}
    Minco2D minco(a,b,q,T);Spline2D spline(a,b,q,T);
    EXPECT_LT((minco.coefficients()-spline.coefficients()).norm()/std::max(1.,minco.coefficients().norm()),1e-9);
    const auto p=minco.energyPartials();const auto ref=minco.propagate(p.coefficients,p.times),analytic=spline.energyGradient();
    EXPECT_LT((ref.times-analytic.times).norm()/std::max(1.,ref.times.norm()),1e-9);
    EXPECT_LT((ref.waypoints-analytic.waypoints).norm()/std::max(1.,ref.waypoints.norm()),1e-9);
    TrajectoryCostConfig c;c.speed_max=.8;c.acceleration_max=1.;
    const auto partial=trajectoryCost(spline.trajectory(),c);const auto g=spline.propagate(partial.coefficients,partial.times);
    const double h=1e-5;
    auto cost=[&](const auto & points,const auto & times){return trajectoryCost(Spline2D(a,b,points,times).trajectory(),c).cost;};
    for(int k=0;k<n;++k) {auto plus=T,minus=T;plus[k]+=h;minus[k]-=h;double fd=(cost(q,plus)-cost(q,minus))/(2*h);EXPECT_NEAR(fd,g.times(k),2e-5*std::max(1.,std::abs(fd)));}
    for(int k=0;k<n-1;++k) for(int d=0;d<2;++d) {auto plus=q,minus=q;plus[k](d)+=h;minus[k](d)-=h;double fd=(cost(plus,T)-cost(minus,T))/(2*h);EXPECT_NEAR(fd,g.waypoints(k,d),2e-5*std::max(1.,std::abs(fd)));}
  }
}
TEST(Spline, InvalidData) {
  TranslationState a;EXPECT_THROW(Spline2D(a,a,{},{}),std::invalid_argument);
  EXPECT_THROW(Spline2D(a,a,{}, {-1}),std::invalid_argument);
  EXPECT_THROW(Spline2D(a,a,{{0,0}}, {1}),std::invalid_argument);
}
