#include <gtest/gtest.h>
#include "motion2d/estimation/fusion_timeline.hpp"

using namespace motion2d;

TEST(FusionTimeline, LeftHeldSampleAndNonalignedCorrection)
{
  FusionTimeline timeline;
  PlanarImu first; first.acceleration = {2, 0}; first.variance.setZero();
  PlanarImu next = first; next.acceleration = {100, 0};
  ASSERT_TRUE(timeline.pushImu({0, first}));
  ASSERT_TRUE(timeline.correct(0, {}));
  timeline.pushImu({10000000, next});
  EXPECT_NEAR(timeline.latest().filter.pose().position.x(), .0001, 1e-14);
  // Correct halfway through the previous interval, after the newer IMU arrived.
  auto scan = timeline.correct(5000000, {{.000025, 0}, 0});
  ASSERT_TRUE(scan);
  EXPECT_EQ(scan->stamp, 5000000);
  EXPECT_NEAR(scan->filter.pose().position.x(), .000025, 1e-14);
  EXPECT_NEAR(timeline.latest().filter.pose().position.x(), .0001, 1e-14);
  EXPECT_NEAR(timeline.latest().filter.state()[2], .02, 1e-14);
}

TEST(FusionTimeline, DelayedMeasurementsEqualImmediateReplayedResult)
{
  EkfConfig config; config.estimate_bias = true;
  FusionTimeline early(config), delayed(config);
  PlanarImu imu; imu.acceleration = {.1, -.2}; imu.yaw_rate = .3;
  early.pushImu({0, imu}); delayed.pushImu({0, imu});
  ASSERT_TRUE(early.correct(0, {})); ASSERT_TRUE(delayed.correct(0, {}));
  for (int k = 1; k <= 20; ++k) {
    early.pushImu({k * 10000000LL, imu}); delayed.pushImu({k * 10000000LL, imu});
    if (k == 10) {ASSERT_TRUE(early.correct(97000000, {{.002, -.003}, .028}));}
  }
  ASSERT_TRUE(delayed.correct(97000000, {{.002, -.003}, .028}));
  EXPECT_LT((early.latest().filter.state() - delayed.latest().filter.state()).norm(), 1e-14);
  EXPECT_LT((early.latest().filter.covariance() - delayed.latest().filter.covariance()).norm(), 1e-14);
}

TEST(FusionTimeline, BoundedHistoryGapAndExplicitReset)
{
  FusionTimeline timeline({}, .2); PlanarImu imu;
  timeline.pushImu({0, imu}); ASSERT_TRUE(timeline.correct(0, {}));
  for (int k = 1; k <= 1000; ++k) {timeline.pushImu({k * 10000000LL, imu});}
  EXPECT_LE(timeline.historySize(), 22U);
  EXPECT_FALSE(timeline.correct(100000000, {})); EXPECT_EQ(timeline.status(), "laser_too_old");
  EXPECT_FALSE(timeline.correct(11000000000, {})); EXPECT_EQ(timeline.status(), "waiting_imu");
  EXPECT_DOUBLE_EQ(timeline.laserAge(), 10.);
  EXPECT_FALSE(timeline.pushImu({10000000000, imu}));
  timeline.pushImu({10200000000, imu}); EXPECT_FALSE(timeline.ready());
  EXPECT_EQ(timeline.status(), "imu_gap");
  timeline.reset(); timeline.pushImu({0, imu}); ASSERT_TRUE(timeline.correct(0, {}));
  EXPECT_EQ(timeline.latest().stamp, 0);
  EXPECT_EQ(timeline.latest().filter.state().norm(), 0.);
}

TEST(FusionTimeline, RejectedCorrectionDoesNotAlterLatestState)
{
  FusionTimeline timeline; PlanarImu imu; imu.yaw_rate = 1.;
  timeline.pushImu({0, imu}); ASSERT_TRUE(timeline.correct(0, {}));
  timeline.pushImu({10000000, imu});
  const auto before = timeline.latest();
  EXPECT_FALSE(timeline.correct(5000000, {{10, 10}, 0}));
  EXPECT_EQ(timeline.status(), "innovation_rejected");
  EXPECT_EQ((before.filter.state() - timeline.latest().filter.state()).norm(), 0.);
  EXPECT_EQ((before.filter.covariance() - timeline.latest().filter.covariance()).norm(), 0.);
  EXPECT_FALSE(timeline.correct(5000000, {}));
}
