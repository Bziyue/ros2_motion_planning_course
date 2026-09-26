#include <gtest/gtest.h>
#include "motion2d/trajectory/bezier_bounds.hpp"
#include "motion2d/trajectory/quintic.hpp"
using namespace motion2d;
TEST(Bezier, ExactBernsteinAndContinuousBounds) {
  TranslationState a,b;b.position={1,0};auto piece=interpolateQuintic(a,b,2);PolynomialTrajectory curve({piece});
  auto box=makeRegion({{{-.1,-.2},{1.1,-.2},{1.1,.2},{-.1,.2}}});
  for(int order=0;order<=2;++order) {
    Eigen::MatrixX2d P=bezierMap(2,order)*piece.coefficients.transpose();const int degree=5-order;
    for(double s:{0.,.17,.4,.91,1.}) {
      Eigen::MatrixX2d work=P;for(int k=degree;k>0;--k) for(int j=0;j<k;++j) work.row(j)=((1-s)*work.row(j)+s*work.row(j+1)).eval();
      EXPECT_LT((work.row(0).transpose()-derivative(piece,2*s,order)).norm(),1e-12);
    }
  }
  TrajectoryLimits limits;limits.speed=1;limits.acceleration=1.5;
  EXPECT_FALSE(certifyBezier(curve,{box},limits,0).certified);
  const auto fine=certifyBezier(curve,{box},limits,5);EXPECT_TRUE(fine.certified);
  EXPECT_GE(fine.speed_bound,.9375-1e-12);EXPECT_GE(fine.acceleration_bound,10/std::sqrt(3.)/4-1e-12);
  EXPECT_THROW(certifyBezier(curve,{},limits),std::invalid_argument);
}
TEST(Bezier, EndpointsCanMissAnExcursion) {
  QuinticPiece p;p.coefficients(0,1)=4;p.coefficients(0,2)=-4;
  auto box=makeRegion({{{-.1,-.1},{.5,-.1},{.5,.1},{-.1,.1}}});
  TrajectoryLimits l;l.speed=10;l.acceleration=10;
  EXPECT_TRUE(box.contains(derivative(p,0,0)));EXPECT_TRUE(box.contains(derivative(p,1,0)));
  EXPECT_FALSE(certifyBezier(PolynomialTrajectory({p}),{box},l,6).certified);
}
TEST(Bezier, AnalyticControlCostGradient) {
  QuinticPiece p;p.duration=.85;p.coefficients<<.313,.1,.2,.1,-.04,.01,.517,.08,.1,-.04,.02,-.01;
  auto box=makeRegion({{{-.5,-.5},{.4,-.5},{.4,1.5},{-.5,1.5}}});TrajectoryCostConfig c;c.speed_max=.25;c.acceleration_max=.3;
  const auto eval=[&](const QuinticPiece & v){return bezierCost(PolynomialTrajectory({v}),{box},c);};
  const auto exact=eval(p);double h=1e-6;
  for(int d=0;d<2;++d) for(int k=0;k<6;++k) {auto plus=p,minus=p;plus.coefficients(d,k)+=h;minus.coefficients(d,k)-=h;double fd=(eval(plus).cost-eval(minus).cost)/(2*h);EXPECT_NEAR(fd,exact.coefficients(k,d),1e-5*std::max(1.,std::abs(fd)));}
  auto plus=p,minus=p;plus.duration+=h;minus.duration-=h;double fd=(eval(plus).cost-eval(minus).cost)/(2*h);EXPECT_NEAR(fd,exact.times(0),1e-5*std::max(1.,std::abs(fd)));
}
