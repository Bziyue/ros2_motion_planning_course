#include <gtest/gtest.h>
#include "motion2d/trajectory/minco_optimizer.hpp"
#include "motion2d/trajectory/quintic.hpp"
using namespace motion2d;
TEST(TrajectoryCost, DirectCoefficientAndMovingTimeGradient) {
  QuinticPiece p;p.duration=.85;
  p.coefficients << .313,.1,.2,.1,-.04,.01, .517,.08,.1,-.04,.02,-.01;
  TrajectoryCostConfig c;c.speed_max=.25;c.acceleration_max=.3;c.esdf_distance=.8;
  GridConfig g;g.width=40;g.height=40;g.resolution=.1;g.origin={-2,-2};
  std::vector<int8_t> cells(1600,0);for(int y=0;y<40;++y) cells[y*40+29]=100;
  Esdf2D field(g,cells);
  auto region=makeRegion({{{-.5,-.5},{.4,-.5},{.4,1.5},{-.5,1.5}}});
  const auto eval=[&](const QuinticPiece & piece) {return trajectoryCost(PolynomialTrajectory({piece}),c,{region},&field);};
  const auto exact=eval(p);const double h=1e-6;
  for(int k=0;k<6;++k) for(int d=0;d<2;++d) {
    auto plus=p,minus=p;plus.coefficients(d,k)+=h;minus.coefficients(d,k)-=h;
    const double fd=(eval(plus).cost-eval(minus).cost)/(2*h);
    EXPECT_NEAR(fd,exact.coefficients(k,d),1e-5*std::max(1.,std::abs(fd)));
  }
  auto plus=p,minus=p;plus.duration+=h;minus.duration-=h;
  const double fd=(eval(plus).cost-eval(minus).cost)/(2*h);
  EXPECT_NEAR(fd,exact.times(0),1e-5*std::max(1.,std::abs(fd)));
}
TEST(Optimization, AnalyticSingleSegmentTimeOptimum) {
  TranslationState a,b;b.position={1,0};MincoOptimizationConfig c;
  c.cost.speed_weight=0;c.cost.acceleration_weight=0;c.optimize_waypoints=false;
  c.solver.gradient_tolerance=1e-6;
  auto r=optimizeMinco(a,b,{}, {2},c);
  ASSERT_TRUE(r.solver.converged()) << r.solver.status;
  EXPECT_NEAR(r.durations[0],std::pow(3600*c.cost.energy_weight/c.cost.time_weight,1./6),1e-7);
  EXPECT_GT(r.durations[0],c.min_duration);
}
TEST(Optimization, QuadraticAndExplicitFailure) {
  auto f=[](const Eigen::VectorXd & x) {ObjectiveValue o;o.value=x.squaredNorm();o.gradient=2*x;return o;};
  const auto r=minimizeBfgs(f,Eigen::VectorXd::Ones(3),{});
  ASSERT_TRUE(r.converged());EXPECT_LT(r.x.norm(),1e-9);
  auto bad=[](const Eigen::VectorXd &) {ObjectiveValue o;o.error="outside";return o;};
  EXPECT_EQ(minimizeBfgs(bad,Eigen::VectorXd::Ones(2),{}).status,"invalid_initial: outside");
  BfgsConfig c;c.max_wall_seconds=1e-12;
  EXPECT_EQ(minimizeBfgs(f,Eigen::VectorXd::Ones(2),c).status,"time_limit");
}
TEST(TrajectoryCost, ViolationsRemainVisible) {
  TranslationState a,b;b.position={1,0};const auto curve=Minco2D(a,b,{}, {1}).trajectory();
  auto r=checkTrajectorySamples(curve,{});
  EXPECT_FALSE(r.samples_feasible);EXPECT_EQ(r.status,"sampled_violation");
  EXPECT_NEAR(r.peak_speed,1.875,1e-10);
}
