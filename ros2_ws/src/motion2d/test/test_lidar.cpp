#include <cmath>
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
