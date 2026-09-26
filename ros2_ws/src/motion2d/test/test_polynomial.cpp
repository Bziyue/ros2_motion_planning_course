#include <gtest/gtest.h>
#include "motion2d/trajectory/quintic.hpp"
#include <random>
#include <limits>

using namespace motion2d;
TEST(Polynomial, RestToRestAnalyticScaling)
{
  TranslationState a,b; b.position={1,2};
  for (double T : {.2,1.,5.}) {
    const auto p=interpolateQuintic(a,b,T);
    const auto c=p.coefficients;
    EXPECT_NEAR(c(0,3)*T*T*T,10,1e-12);
    EXPECT_NEAR(c(0,4)*std::pow(T,4),-15,1e-12);
    EXPECT_NEAR(c(0,5)*std::pow(T,5),6,1e-12);
    EXPECT_LT((derivative(p,T*.5,0)-Eigen::Vector2d(.5,1)).norm(),1e-12);
    EXPECT_NEAR(derivative(p,T*.5,1).x(),1.875/T,1e-10);
    EXPECT_LT(derivative(p,T*.5,2).norm(),1e-10);
  }
}
TEST(Polynomial, RandomBoundaryAndFiniteDifferences)
{
  std::mt19937 gen(1414); std::uniform_real_distribution<double> random(-2,2);
  for (int n=0;n<100;++n) {
    TranslationState a,b;
    a.position={random(gen),random(gen)}; b.position={random(gen),random(gen)};
    a.velocity={random(gen),random(gen)}; b.velocity={random(gen),random(gen)};
    a.acceleration={random(gen),random(gen)}; b.acceleration={random(gen),random(gen)};
    const double T=1+.2*random(gen); const auto p=interpolateQuintic(a,b,T);
    EXPECT_LT((derivative(p,0,0)-a.position).norm(),1e-10);
    EXPECT_LT((derivative(p,0,1)-a.velocity).norm(),1e-10);
    EXPECT_LT((derivative(p,0,2)-a.acceleration).norm(),1e-10);
    EXPECT_LT((derivative(p,T,0)-b.position).norm(),1e-10);
    EXPECT_LT((derivative(p,T,1)-b.velocity).norm(),1e-10);
    EXPECT_LT((derivative(p,T,2)-b.acceleration).norm(),1e-9);
    for (int d=1;d<=3;++d) {
      const double t=.4*T,h=1e-5*T;
      const Eigen::Vector2d fd=(derivative(p,t+h,d-1)-derivative(p,t-h,d-1))/(2*h);
      EXPECT_LT((fd-derivative(p,t,d)).norm(),2e-6);
    }
  }
}
TEST(Polynomial, C2JoinRightKnotAndTimeBoundaries)
{
  TranslationState a,b,c; b.position={1,1}; b.velocity={.5,-.1}; b.acceleration={.2,.4}; c.position={2,0};
  const auto p=interpolateQuintic(a,b,1), q=interpolateQuintic(b,c,2);
  PolynomialTrajectory curve({p,q});
  EXPECT_DOUBLE_EQ(curve.duration(),3);
  EXPECT_LT((curve.sample(1).velocity-b.velocity).norm(),1e-12);
  EXPECT_LT((curve.evaluate(1,3)-derivative(q,0,3)).norm(),1e-12);
  for(int d=0;d<3;++d) EXPECT_LT((curve.evaluate(1-1e-8,d)-curve.evaluate(1+1e-8,d)).norm(),1e-5);
  EXPECT_LT(curve.sample(3).velocity.norm(),1e-12);
  EXPECT_THROW(curve.sample(-.01),std::invalid_argument);
  EXPECT_THROW(curve.sample(3.01),std::invalid_argument);
  EXPECT_THROW(curve.evaluate(1,6),std::invalid_argument);
  auto broken=q; broken.coefficients(0,1)+=.001;
  EXPECT_THROW(PolynomialTrajectory({p,broken}),std::invalid_argument);
}
TEST(Polynomial, ExplicitStopsRepeatedPointAndInvalidInput)
{
  auto curve=stopAtWaypoints({{0,0},{1,0},{1,0},{1,1}},.5,.2);
  EXPECT_NEAR(curve.duration(),4.2,1e-12);
  EXPECT_LT(curve.sample(2.1).velocity.norm(),1e-12);
  EXPECT_LT((curve.sample(2.1).position-Eigen::Vector2d(1,0)).norm(),1e-12);
  EXPECT_THROW(PolynomialTrajectory({}),std::invalid_argument);
  EXPECT_THROW(interpolateQuintic({}, {},0),std::invalid_argument);
  EXPECT_THROW(stopAtWaypoints({{0,0},{1,1}},0),std::invalid_argument);
  auto p=interpolateQuintic({}, {},1); p.coefficients(1,3)=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(PolynomialTrajectory({p}),std::invalid_argument);
}
