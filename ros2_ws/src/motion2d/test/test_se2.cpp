#include <gtest/gtest.h>
#include <random>
#include "motion2d/geometry/se2.hpp"

using namespace motion2d;

TEST(Se2, HandComputedQuarterTurn)
{
  const auto p = transformPoint({{1.0, 2.0}, kPi / 2.0}, {2.0, 0.0});
  EXPECT_NEAR(p.x(), 1.0, 1e-12);
  EXPECT_NEAR(p.y(), 4.0, 1e-12);
}

TEST(Se2, OffsetSensorComposition)
{
  // Sensor is 0.5 m in front of a robot at (1,2), heading north.
  const auto map_from_laser = compose({{1.0, 2.0}, kPi / 2.0}, {{0.5, 0.0}, 0.0});
  EXPECT_NEAR(map_from_laser.position.x(), 1.0, 1e-12);
  EXPECT_NEAR(map_from_laser.position.y(), 2.5, 1e-12);
  EXPECT_NEAR(map_from_laser.yaw, kPi / 2.0, 1e-12);
}

TEST(Se2, RoundTripAcrossRandomFrames)
{
  std::mt19937 random(42);
  std::uniform_real_distribution<double> sample(-10.0, 10.0);
  for (int i = 0; i < 200; ++i) {
    const Pose2D pose{{sample(random), sample(random)}, sample(random)};
    const Eigen::Vector2d point{sample(random), sample(random)};
    const auto back = transformPoint(inverse(pose), transformPoint(pose, point));
    EXPECT_LT((back - point).norm(), 1e-12);
  }
}

TEST(Se2, AngleBoundary)
{
  EXPECT_DOUBLE_EQ(wrapAngle(kPi), -kPi);
  EXPECT_DOUBLE_EQ(wrapAngle(-kPi), -kPi);
  EXPECT_NEAR(wrapAngle(5.0 * kPi / 2.0), kPi / 2.0, 1e-12);
  EXPECT_NEAR(wrapAngle(-5.0 * kPi / 2.0), -kPi / 2.0, 1e-12);
}
