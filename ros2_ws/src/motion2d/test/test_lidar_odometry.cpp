#include <gtest/gtest.h>
#include "motion2d/estimation/lidar_odometry.hpp"

using namespace motion2d;
namespace
{
PointCloud2D corner(const Pose2D & pose = {})
{
  PointCloud2D cloud;
  for (int i = -50; i <= 50; ++i) {
    cloud.push_back(transformPoint(inverse(pose), {3., i * .04}));
    cloud.push_back(transformPoint(inverse(pose), {i * .04, 3.}));
  }
  return cloud;
}
}

TEST(LidarOdometry, CentroidsRespectNegativeCoordinates)
{
  const auto cloud = voxelCloud({{-.01, 0}, {-.09, .04}, {.01, 0}}, .1);
  ASSERT_EQ(cloud.size(), 2u);
  EXPECT_NEAR(cloud[0].x(), -.05, 1e-12);
  EXPECT_NEAR(cloud[0].y(), .02, 1e-12);
  EXPECT_NEAR(cloud[1].x(), .01, 1e-12);
}

TEST(LidarOdometry, BoundedSubmapTracksSeveralTurns)
{
  LidarOdometryConfig config;
  config.max_keyframes = 3;
  LidarOdometry odom(config);
  ASSERT_TRUE(odom.update(corner(), 0).accepted);
  for (int k = 1; k <= 50; ++k) {
    Pose2D truth{{.02 * k, .1 * std::sin(.04 * k)}, .02 * k};
    const auto result = odom.update(corner(truth), k * 100000000LL);
    ASSERT_TRUE(result.accepted) << k << " " << result.status;
    EXPECT_LT((result.pose.position - truth.position).norm(), .02);
    EXPECT_NEAR(result.pose.yaw, truth.yaw, .003);
    EXPECT_LE(odom.keyframeCount(), 3u);
  }
  EXPECT_EQ(odom.keyframeCount(), 3u);
}

TEST(LidarOdometry, FailureIsNotInsertedAndNextFrameCanRecover)
{
  LidarOdometry odom;
  odom.update(corner(), 0);
  const auto points = odom.submap();
  auto rejected = odom.update({}, 100000000);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(odom.submap(), points);
  const Pose2D truth{{.04, .01}, .02};
  const auto result = odom.update(corner(truth), 200000000);
  ASSERT_TRUE(result.accepted);
  EXPECT_LT((result.pose.position - truth.position).norm(), .01);
  EXPECT_NEAR(result.velocity.x(), .2, .02); // dt spans the rejected frame.
  EXPECT_EQ(odom.update(corner(truth), 200000000).status, "duplicate");
  EXPECT_EQ(odom.update(corner(truth), 2000000000).status, "tracking_lost");
  EXPECT_TRUE(odom.update(corner(), 0).accepted);
  EXPECT_EQ(odom.keyframeCount(), 1u);
}
