#include <algorithm>
#include <cmath>
#include <utility>
#include <limits>
#include <gtest/gtest.h>
#include "motion2d/sim/lidar_cpu.hpp"

using namespace motion2d;

TEST(Lidar, FullTurnDoesNotDuplicateEndpoint)
{
  LidarConfig config;
  config.beams = 4;
  config.angle_min = 0;
  config.range_max = 30;
  const auto scan = scanCpu(World2D{}, {{1, 2}, 0}, config);
  EXPECT_NEAR(beamIncrement(config), kPi / 2, 1e-12);
  ASSERT_EQ(scan.size(), 4U);
  EXPECT_FLOAT_EQ(scan[0], 9);
  EXPECT_FLOAT_EQ(scan[1], 8);
  EXPECT_FLOAT_EQ(scan[2], 11);
  EXPECT_FLOAT_EQ(scan[3], 12);
}

TEST(Lidar, PartialFovIncludesBothEndpointsAndRobotYaw)
{
  LidarConfig config;
  config.beams = 3;
  config.angle_min = -kPi / 4;
  config.fov = kPi / 2;
  config.range_max = 30;
  const auto scan = scanCpu(World2D{}, {{0, 0}, kPi / 2}, config);
  EXPECT_NEAR(beamIncrement(config), kPi / 4, 1e-12);
  EXPECT_NEAR(scan[0], std::sqrt(200.0), 1e-6);
  EXPECT_FLOAT_EQ(scan[1], 10);
  EXPECT_NEAR(scan[2], std::sqrt(200.0), 1e-6);
  const auto shifted = scanCpu(World2D{}, {{1, 2}, kPi / 2}, config);
  EXPECT_FLOAT_EQ(shifted[1], 8);
}

TEST(Lidar, BlindReturnOccludesFarWallAndNoReturnIsPositiveInfinity)
{
  World2D world;
  world.circles.push_back({{.06, 0}, .02});
  LidarConfig config;
  config.beams = 4;
  config.angle_min = 0;
  config.range_max = 5;
  const auto scan = scanCpu(world, {}, config);
  EXPECT_TRUE(std::isnan(scan[0]));  // .04 m hit, not the x=10 wall behind it.
  for (int i = 1; i < 4; ++i) {
    EXPECT_TRUE(std::isinf(scan[i]));
    EXPECT_GT(scan[i], 0);
  }
}

TEST(Lidar, ExactRangeEndpointsAreValidAndObjectBehindLimitIsNot)
{
  LidarConfig config;
  config.beams = 4;
  config.angle_min = 0;
  config.range_min = 1;
  config.range_max = 10;
  World2D world;
  world.circles.push_back({{2, 0}, 1});
  auto scan = scanCpu(world, {}, config);
  EXPECT_FLOAT_EQ(scan[0], 1);
  EXPECT_FLOAT_EQ(scan[1], 10);
  world.circles.clear();
  config.range_max = 9.99;
  EXPECT_TRUE(std::isinf(scanCpu(world, {}, config)[0]));
}

TEST(Lidar, InvalidConfigurationIsRejectedAtBoundary)
{
  LidarConfig config;
  EXPECT_NO_THROW(validateLidarConfig(config));
  config.beams = 1;
  EXPECT_THROW(validateLidarConfig(config), std::invalid_argument);
  config.beams = 4;
  config.fov = 7;
  EXPECT_THROW(validateLidarConfig(config), std::invalid_argument);
  config.fov = kPi;
  config.range_min = config.range_max;
  EXPECT_THROW(validateLidarConfig(config), std::invalid_argument);
  config.range_min = 0;
  config.angle_min = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validateLidarConfig(config), std::invalid_argument);
}

TEST(Lidar, ScanPeriodIsAlignedToSimulatedTicks)
{
  EXPECT_EQ(lidarPeriodTicks(10, .005), 20);
  EXPECT_EQ(lidarPeriodTicks(20, .005), 10);
  EXPECT_EQ(lidarPeriodTicks(200, .005), 1);
  EXPECT_THROW(lidarPeriodTicks(7, .005), std::invalid_argument);
  EXPECT_THROW(lidarPeriodTicks(201, .005), std::invalid_argument);
  EXPECT_THROW(lidarPeriodTicks(0, .005), std::invalid_argument);
}

TEST(LidarNoise, SigmaZeroPreservesValuesAndRandomState)
{
  LidarConfig config;
  std::mt19937 random(42), untouched(42);
  std::vector<float> ranges{1, 5, 10};
  const auto original = ranges;
  addRangeNoise(ranges, config, random);
  EXPECT_EQ(ranges, original);
  EXPECT_EQ(random, untouched);
  config.range_stddev = -1;
  EXPECT_THROW(validateLidarConfig(config), std::invalid_argument);
}

TEST(LidarNoise, InvalidAndAbsentReturnsDoNotConsumeNoise)
{
  LidarConfig config;
  config.range_stddev = .01;
  std::vector<float> ranges{std::numeric_limits<float>::infinity(),
    std::numeric_limits<float>::quiet_NaN()};
  std::mt19937 random(42), untouched(42);
  addRangeNoise(ranges, config, random);
  EXPECT_GT(ranges[0], 0);
  EXPECT_TRUE(std::isinf(ranges[0]));
  EXPECT_TRUE(std::isnan(ranges[1]));
  EXPECT_EQ(random, untouched);
}

TEST(LidarNoise, NoisyOutOfRangeReturnsAreInvalidNotFreeSpace)
{
  LidarConfig config;
  config.range_min = 1;
  config.range_max = 10;
  EXPECT_TRUE(std::isnan(perturbRange(1.01F, -.02, config)));
  EXPECT_TRUE(std::isnan(perturbRange(9.99F, .02, config)));
  EXPECT_FLOAT_EQ(perturbRange(2, -1, config), 1);
  EXPECT_FLOAT_EQ(perturbRange(9, 1, config), 10);
  EXPECT_FLOAT_EQ(perturbRange(5, .01, config), 5.01F);
}

TEST(LidarNoise, SeedAndResetReproduceTheSequence)
{
  LidarConfig config;
  config.range_stddev = .01;
  std::mt19937 first(4242), second(4242);
  std::vector<float> a(9, 5), b(9, 5), later(9, 5), replay(9, 5);
  addRangeNoise(a, config, first);
  addRangeNoise(b, config, second);
  EXPECT_EQ(a, b);
  addRangeNoise(later, config, first);
  EXPECT_NE(a, later);
  first.seed(4242);
  addRangeNoise(replay, config, first);
  EXPECT_EQ(a, replay);
}

TEST(LidarNoise, GaussianMeanAndVarianceAwayFromRangeLimits)
{
  LidarConfig config;
  config.range_stddev = .03;
  std::mt19937 random(4242);
  constexpr int n = 50000;
  std::vector<float> ranges(n, 5);
  addRangeNoise(ranges, config, random);
  double sum = 0, squared_sum = 0;
  for (float range : ranges) {
    const double error = range - 5.0;
    ASSERT_TRUE(std::isfinite(range));
    sum += error;
    squared_sum += error * error;
  }
  const double mean = sum / n;
  const double variance = squared_sum / n - mean * mean;
  EXPECT_LT(std::abs(mean), 5 * config.range_stddev / std::sqrt(n));
  EXPECT_NEAR(variance, config.range_stddev * config.range_stddev,
    .03 * config.range_stddev * config.range_stddev);
}

TEST(Lidar, SparseAngularSamplingCanMissASmallObstacle)
{
  World2D world;
  const double angle = 2 * kPi / 180;
  world.circles.push_back({{5 * std::cos(angle), 5 * std::sin(angle)}, .05});
  LidarConfig config;
  config.range_max = 8;
  for (const auto & [beams, expected_hits] :
    std::vector<std::pair<int, int>>{{90, 0}, {360, 1}, {720, 3}})
  {
    config.beams = beams;
    const auto ranges = scanCpu(world, {}, config);
    const auto hits = std::count_if(ranges.begin(), ranges.end(),
      [](float range) {return std::isfinite(range);});
    EXPECT_EQ(hits, expected_hits) << "beams=" << beams;
  }
}
