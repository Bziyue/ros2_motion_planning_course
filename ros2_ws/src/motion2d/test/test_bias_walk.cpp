#include <gtest/gtest.h>
#include <random>
#include "motion2d/sim/bias_walk.hpp"
using namespace motion2d;
TEST(BiasWalk,ScalingAndZeroTime) {
  EXPECT_DOUBLE_EQ(stepBiasWalk(.1,.02,0,3),.1);
  EXPECT_DOUBLE_EQ(stepBiasWalk(.1,0,4,3),.1);
  EXPECT_NEAR(stepBiasWalk(.1,.02,4,-2),.02,1e-14);
  EXPECT_THROW(stepBiasWalk(0,-1,.1,0),std::invalid_argument);
  EXPECT_THROW(stepBiasWalk(0,1,-.1,0),std::invalid_argument);
}
TEST(BiasWalk,EndpointVarianceIndependentOfSamplingRate) {
  // Independent trajectories, not correlated samples from one trajectory.
  for(int steps:{100,200}) {
    std::mt19937 rng(20260926+steps);std::normal_distribution<double> normal;
    double sum=0,square=0;constexpr int trials=5000;constexpr double density=.002,T=10;
    for(int j=0;j<trials;++j) {
      double b=0;for(int k=0;k<steps;++k) b=stepBiasWalk(b,density,T/steps,normal(rng));
      sum+=b;square+=b*b;
    }
    const double mean=sum/trials,variance=(square-trials*mean*mean)/(trials-1);
    EXPECT_NEAR(mean,0,4*std::sqrt(density*density*T/trials));
    EXPECT_NEAR(variance,density*density*T,.08*density*density*T);
  }
}
