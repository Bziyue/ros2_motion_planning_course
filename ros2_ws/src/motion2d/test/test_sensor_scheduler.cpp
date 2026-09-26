#include <gtest/gtest.h>
#include <limits>
#include "motion2d/sim/sensor_scheduler.hpp"

using namespace motion2d;

TEST(SensorScheduler, AlignedRatesIncludeZeroAndReset)
{
  SensorScheduler imu(200), lidar(10);
  for (int k = 0; k < 100; ++k) {
    EXPECT_EQ(imu.nextStamp(), k * 5000000LL);
    EXPECT_EQ(lidar.nextStamp(), k * 100000000LL);
    imu.advance();
    lidar.advance();
  }
  imu.reset();
  EXPECT_EQ(imu.nextStamp(), 0);
  EXPECT_DOUBLE_EQ(lidar.period(), .1);
}

TEST(SensorScheduler, SevenHertzDoesNotRoundToTicksOrAccumulatePhaseError)
{
  SensorScheduler scheduler(7);
  for (std::int64_t k = 0; k <= 7 * 3600; ++k) {
    // Integer arithmetic supplies an independent nearest-ns oracle.
    EXPECT_EQ(scheduler.nextStamp(), (k * 1000000000LL + 3) / 7);
    scheduler.advance();
  }
}

TEST(SensorScheduler, NonintegerHertzAndFineNanosecondQuantization)
{
  SensorScheduler scheduler(137.5);
  for (std::int64_t k = 0; k < 10000; ++k) {
    EXPECT_EQ(scheduler.nextStamp(), (k * 2000000000LL + 137) / 275);
    scheduler.advance();
  }
}

TEST(SensorScheduler, PausedTickAndOneStepOnlyConsumeDueSamples)
{
  SensorScheduler scheduler(137);
  int samples = 0;
  for (std::int64_t tick = 0; tick <= 200; ++tick) {
    const auto now = tick * 5000000;
    for (int paused_callbacks = 0; paused_callbacks < 3; ++paused_callbacks) {
      while (scheduler.nextStamp() <= now) {
        const auto delay = now - scheduler.nextStamp();
        EXPECT_GE(delay, 0);
        EXPECT_LT(delay, 5000000);
        ++samples;
        scheduler.advance();
      }
    }
  }
  EXPECT_EQ(samples, 138);  // Includes t=0 and t=1.
}

TEST(SensorScheduler, InvalidRatesAreRejected)
{
  for (const double rate : {0., -.1, .09, 1e6 + 1,
      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
  {
    EXPECT_THROW(SensorScheduler{rate}, std::invalid_argument);
  }
}
