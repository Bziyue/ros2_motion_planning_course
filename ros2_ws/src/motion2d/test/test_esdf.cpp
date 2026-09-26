#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include "motion2d/mapping/distance_transform.hpp"
#include "motion2d/mapping/esdf.hpp"

using namespace motion2d;

TEST(Edt, ExactAgainstIndependentNearestSeedSearch)
{
  std::mt19937 random(1212);
  for (int trial = 0; trial < 50; ++trial) {
    const int w = 3+random()%12, h = 1+random()%10;
    std::vector<std::uint8_t> seeds(w*h);
    for (auto & b : seeds) {b = random()%7 == 0;}
    const auto d = squaredDistanceTransform(w, h, seeds);
    for (int i = 0; i < w*h; ++i) {
      double expected = std::numeric_limits<double>::infinity();
      for (int j = 0; j < w*h; ++j) {
        if (!seeds[j]) {continue;}
        const int dx = i%w-j%w, dy = i/w-j/w;
        expected = std::min(expected, double(dx*dx+dy*dy));
      }
      EXPECT_EQ(d[i], expected);
    }
  }
  EXPECT_EQ(squaredDistanceTransform(5, 1, {1, 0, 0, 0, 0}).back(), 16);
  EXPECT_TRUE(std::isinf(squaredDistanceTransform(3, 2, std::vector<std::uint8_t>(6))[0]));
  EXPECT_THROW(squaredDistanceTransform(0, 2, {}), std::invalid_argument);
}

TEST(Esdf, SignedStraightWallAndUnits)
{
  GridConfig g; g.width = g.height = 40; g.resolution = .2; g.origin.setZero();
  std::vector<std::int8_t> raw(1600);
  for (int y = 0; y < 40; ++y) {for (int x = 0; x <= 10; ++x) {raw[y*40+x] = 100;}}
  Esdf2D field(g, raw);
  EXPECT_NEAR(field.distances()[12*40+12], .4, 1e-12);
  EXPECT_NEAR(field.distances()[12*40+9], -.4, 1e-12);
  const auto q = field.sample({2.63, 3.11}); ASSERT_TRUE(q);
  EXPECT_NEAR(q->distance, .53, 1e-12); // Wall centres lie at x=2.1, wall edge x=2.2.
  EXPECT_NEAR(q->gradient.x(), 1., 1e-12); EXPECT_NEAR(q->gradient.y(), 0., 1e-12);
  EXPECT_LE(q->clearance_lower_bound, .43);
  EXPECT_GT(q->clearance_lower_bound, .2);
  EXPECT_NEAR(field.sample({2.2, 3.11})->distance, 0., 1e-12);
  EXPECT_NEAR(field.sample({2.2, 3.11})->gradient.x(), 2., 1e-12);
}

TEST(Esdf, BilinearDerivativeMatchesFiniteDifferences)
{
  const std::array<double, 4> d{.2, .7, -.1, 1.4};
  const double u = .37, v = .63, resolution = .13, epsilon = 1e-6;
  const auto q = bilinear(d, u, v, resolution);
  EXPECT_NEAR(q.gradient.x(), (bilinear(d, u+epsilon, v, resolution).value-
    bilinear(d, u-epsilon, v, resolution).value)/(2*epsilon*resolution), 1e-9);
  EXPECT_NEAR(q.gradient.y(), (bilinear(d, u, v+epsilon, resolution).value-
    bilinear(d, u, v-epsilon, resolution).value)/(2*epsilon*resolution), 1e-9);
  GridConfig g; g.width = g.height = 20; g.resolution = .2; g.origin.setZero();
  std::vector<std::int8_t> raw(400, 0); raw[8*20+8] = 100;
  Esdf2D field(g, raw); const Eigen::Vector2d p(2.27, 2.39);
  const auto s = field.sample(p); ASSERT_TRUE(s);
  for (int axis = 0; axis < 2; ++axis) {
    Eigen::Vector2d delta = Eigen::Vector2d::Zero(); delta[axis] = epsilon;
    EXPECT_NEAR(s->gradient[axis], (field.sample(p+delta)->distance-field.sample(p-delta)->distance)/(2*epsilon), 1e-9);
  }
}

TEST(Esdf, ClearanceBoundNeverExceedsExactSquareAndExteriorDistance)
{
  GridConfig g; g.width = 17; g.height = 14; g.resolution = .13; g.origin = {-1.3, -.8};
  std::mt19937 rng(1213); std::uniform_real_distribution<double> uniform(0, 1);
  std::vector<std::int8_t> raw(g.width*g.height, 0);
  for (auto & v : raw) {if (rng()%9 == 0) {v = 100;}}
  Esdf2D field(g, raw);
  for (int trial = 0; trial < 1000; ++trial) {
    const Eigen::Vector2d local(.5+uniform(rng)*(g.width-1), .5+uniform(rng)*(g.height-1));
    const Eigen::Vector2d p = g.origin+g.resolution*local;
    double exact = g.resolution*std::min({local.x(), local.y(), g.width-local.x(), g.height-local.y()});
    for (int i = 0; i < int(raw.size()); ++i) {
      if (!raw[i]) {continue;}
      const Eigen::Vector2d c(i%g.width+.5, i/g.width+.5);
      const auto gap = ((local-c).cwiseAbs()-Eigen::Vector2d::Constant(.5)).cwiseMax(0.).eval();
      exact = std::min(exact, g.resolution*gap.norm());
    }
    const auto s = field.sample(p); ASSERT_TRUE(s);
    EXPECT_GE(s->clearance_lower_bound, 0);
    EXPECT_LE(s->clearance_lower_bound, exact+1e-12);
  }
}

TEST(Esdf, UnknownExteriorAndInfiniteInteriorHaveExplicitSemantics)
{
  GridConfig g; g.width = g.height = 8; g.resolution = .1; g.origin.setZero();
  std::vector<std::int8_t> raw(64, -1);
  Esdf2D conservative(g, raw); EXPECT_FALSE(conservative.sample({.3, .3}));
  EXPECT_EQ(conservative.distances()[0], -std::numeric_limits<double>::infinity());
  Esdf2D optimistic(g, raw, 35, false);
  EXPECT_NEAR(optimistic.distances()[3*8+3], .4, 1e-12); // Padded exterior centres.
  EXPECT_FALSE(optimistic.sample({.01, .4})); EXPECT_FALSE(optimistic.sample({10, 1}));
  EXPECT_FALSE(optimistic.sample({NAN, .3}));
  EXPECT_EQ(obstacleMask({-1, 0, 35, 36, 100}), (std::vector<std::uint8_t>{1,0,0,1,1}));
  EXPECT_THROW(Esdf2D(g, std::vector<std::int8_t>(1)), std::invalid_argument);
  raw[0] = -2; EXPECT_THROW(Esdf2D(g, raw), std::invalid_argument);
}
