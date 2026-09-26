#include <gtest/gtest.h>
#include <limits>
#include "motion2d/mapping/scan_projection.hpp"

using namespace motion2d;

TEST(ScanProjection, PolarAnglesAndInclusiveLimits)
{
  Scan2D scan{0, kPi / 2, 1, 2, {1, 2, 1, 2}};
  validateScan(scan);
  const auto p = projectScan(scan);
  ASSERT_EQ(p.size(), 4u);
  EXPECT_NEAR((p[0] - Eigen::Vector2d(1, 0)).norm(), 0, 1e-12);
  EXPECT_NEAR((p[1] - Eigen::Vector2d(0, 2)).norm(), 0, 1e-12);
  EXPECT_NEAR((p[2] - Eigen::Vector2d(-1, 0)).norm(), 0, 1e-12);
  EXPECT_NEAR((p[3] - Eigen::Vector2d(0, -2)).norm(), 0, 1e-12);
}

TEST(ScanProjection, InvalidAndNoReturnAreNotPoints)
{
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  Scan2D scan{0, .1, 1, 2, {inf, nan, -inf, .9F, 2.1F}};
  EXPECT_TRUE(projectScan(scan).empty());
}

TEST(ScanProjection, AcquisitionPoseTransformsPoints)
{
  const auto points = registerPoints({{1, 0}, {0, 2}}, {{3, 4}, kPi / 2});
  EXPECT_NEAR((points[0] - Eigen::Vector2d(3, 5)).norm(), 0, 1e-12);
  EXPECT_NEAR((points[1] - Eigen::Vector2d(1, 4)).norm(), 0, 1e-12);
}

TEST(ScanProjection, RejectsBadMetadata)
{
  Scan2D scan{0, .1, 0, 2, {1}};
  EXPECT_NO_THROW(validateScan(scan));
  scan.angle_increment = 0;
  EXPECT_THROW(validateScan(scan), std::invalid_argument);
  scan.angle_increment = .1;
  scan.range_min = 3;
  EXPECT_THROW(validateScan(scan), std::invalid_argument);
  scan.range_min = 0;
  scan.ranges.clear();
  EXPECT_THROW(validateScan(scan), std::invalid_argument);
}
