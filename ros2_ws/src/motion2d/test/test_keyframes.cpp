#include <gtest/gtest.h>
#include "motion2d/estimation/keyframes.hpp"
#include "motion2d/sim/lidar_cpu.hpp"
#include "motion2d/sim/world.hpp"
using namespace motion2d;

namespace
{
PointCloud2D corner()
{
  PointCloud2D result;
  for (int i = -50; i <= 50; ++i) {result.emplace_back(3., i*.04); result.emplace_back(i*.04, 3.);}
  return result;
}
Scan2D roomScan(const Pose2D & pose)
{
  World2D world; world.width = 12.; world.height = 10.;
  world.circles.push_back({{.5, 1.}, .5});
  LidarConfig config; config.beams = 360; config.range_max = 20.;
  return {config.angle_min, beamIncrement(config), config.range_min, config.range_max,
    scanCpu(world, pose, config)};
}
}

TEST(LoopClosure, RecoversKnownTransformAndRejectsNoOverlap)
{
  const auto cloud = corner();
  const Pose2D truth{{.08, -.04}, .025};
  PointCloud2D current;
  for (const auto & p : cloud) {current.push_back(transformPoint(inverse(truth), p));}
  const auto result = verifyLoop(makeMatchTarget(current), makeMatchTarget(cloud), {{.07, -.03}, .02});
  ASSERT_TRUE(result.accepted) << result.status;
  EXPECT_LT((result.relative.position-truth.position).norm(), .001);
  EXPECT_NEAR(result.relative.yaw, truth.yaw, 1e-5);
  EXPECT_GT(result.overlap, .9);
  for (auto & p : current) {p.x() += 100;}
  EXPECT_FALSE(verifyLoop(makeMatchTarget(current), makeMatchTarget(cloud), {}).accepted);
}

TEST(LoopClosure, RejectsSingleWallDegeneracy)
{
  PointCloud2D line;
  for (int i = -50; i <= 50; ++i) {line.emplace_back(i*.04, 3.);}
  const auto target = makeMatchTarget(line);
  const auto result = verifyLoop(target, target, {});
  EXPECT_FALSE(result.accepted); EXPECT_EQ(result.status, "degenerate");
}

TEST(Keyframes, LoopChangesMapButPreservesAllOdomPoses)
{
  KeyframeConfig config; config.distance = .2; config.min_separation = 4;
  config.loop_spacing = 1; config.max_frames = 5;
  KeyframeSlam slam(config);
  const Eigen::Vector2d positions[]{{0, 0}, {.6, 0}, {.6, .6}, {0, .6}, {0, 0}};
  std::vector<Pose2D> odometry;
  for (int k = 0; k < 5; ++k) {
    Pose2D observed{positions[k] + Eigen::Vector2d(.04*k, .02*k), .01*k};
    odometry.push_back(observed);
    const auto scan = roomScan({positions[k]+Eigen::Vector2d(-2,-1), 0});
    const auto update = slam.update(scan, observed, k*1000000000LL);
    ASSERT_TRUE(update.keyframe_added);
    if (k < 4) {EXPECT_EQ(slam.loopCount(), 0U);}
    else {EXPECT_TRUE(update.loop_added) << update.status;}
  }
  ASSERT_EQ(slam.loopCount(), 1U);
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_EQ((slam.frames()[i].odom_pose.position-odometry[i].position).norm(), 0.);
    EXPECT_EQ(slam.frames()[i].odom_pose.yaw, odometry[i].yaw);
  }
  const auto map_pose = compose(slam.mapToOdom(), odometry.back());
  EXPECT_LT(map_pose.position.norm(), .02);
  EXPECT_LT((map_pose.position-slam.frames().back().map_pose.position).norm(), 1e-12);
  EXPECT_EQ(slam.update(roomScan({}), {{1,0},0}, 5000000000).status, "capacity_reached");
  EXPECT_TRUE(slam.update(roomScan({}), {}, 0).keyframe_added);
  EXPECT_EQ(slam.frames().size(), 1U); EXPECT_EQ(slam.loopCount(), 0U);
}

TEST(Keyframes, RebuildRemovesOldWallInsteadOfAccumulatingBoth)
{
  GridConfig config; config.resolution = 1.; config.width = 8; config.height = 4; config.origin = {-1,-1};
  Scan2D scan{0, .1, .05, 10., {3.f}};
  Keyframe frame{0, {}, {}, scan, {{Eigen::Vector2d(3,0)}, {}}};
  const auto before = rebuildKeyframeMap({frame}, config);
  ASSERT_GT(before.grid.occupancy()[*before.grid.cellIndex({3.5,.5})], 65);
  frame.map_pose.position.x() = 1.;
  const auto after = rebuildKeyframeMap({frame}, config);
  EXPECT_LT(after.grid.occupancy()[*after.grid.cellIndex({3.5,.5})], 50);
  EXPECT_GT(after.grid.occupancy()[*after.grid.cellIndex({4.5,.5})], 65);
  ASSERT_EQ(after.cloud.size(), 1U); EXPECT_NEAR(after.cloud[0].x(), 4., 1e-12);
  EXPECT_EQ(after.outside_scans, 0U);
}
