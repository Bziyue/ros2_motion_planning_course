#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "motion2d/ros/mapping_messages.hpp"

TEST(MappingMessages, CloudLayoutAndEmptyCloud)
{
  std_msgs::msg::Header header;
  header.frame_id = "odom";
  header.stamp.sec = 7;
  const auto cloud = motion2d::toPointCloud({{1, 2}, {-3, 4}}, header);
  EXPECT_EQ(cloud.header, header);
  EXPECT_EQ(cloud.height, 1u);
  EXPECT_EQ(cloud.width, 2u);
  EXPECT_EQ(cloud.point_step, 12u);
  EXPECT_EQ(cloud.row_step, 24u);
  EXPECT_EQ(cloud.data.size(), 24u);
  EXPECT_TRUE(cloud.is_dense);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  EXPECT_FLOAT_EQ(*x, 1); EXPECT_FLOAT_EQ(*y, 2); EXPECT_FLOAT_EQ(*z, 0);
  ++x; ++y; ++z;
  EXPECT_FLOAT_EQ(*x, -3); EXPECT_FLOAT_EQ(*y, 4); EXPECT_FLOAT_EQ(*z, 0);
  const auto empty = motion2d::toPointCloud({}, header);
  EXPECT_EQ(empty.height, 1u);
  EXPECT_EQ(empty.width, 0u);
  EXPECT_TRUE(empty.data.empty());
}

TEST(MappingMessages, RejectsUnsupportedFramesAndNonplanarPose)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.range_max = 10;
  scan.angle_increment = .1F;
  scan.ranges = {1};
  EXPECT_NO_THROW(motion2d::fromLaserScan(scan));
  scan.time_increment = .001F;
  EXPECT_THROW(motion2d::fromLaserScan(scan), std::invalid_argument);
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.orientation.w = 1;
  EXPECT_NO_THROW(motion2d::poseFromOdometry(odom));
  odom.header.frame_id = "world";
  EXPECT_THROW(motion2d::poseFromOdometry(odom), std::invalid_argument);
  odom.header.frame_id = "odom";
  odom.pose.pose.orientation.x = .1;
  EXPECT_THROW(motion2d::poseFromOdometry(odom), std::invalid_argument);
}

TEST(MappingMessages, OccupancyMetadataOriginAndRowMajorData)
{
  motion2d::GridConfig config;
  config.resolution = 1;
  config.width = config.height = 5;
  config.origin = {-2, -3};
  motion2d::OccupancyGrid2D grid(config);
  grid.insertScan({0, .1, .05, 10, {2}}, {{-1.5, -2.5}, 0});
  builtin_interfaces::msg::Time stamp, loaded;
  stamp.sec = 7; loaded.sec = 2;
  const auto message = motion2d::toOccupancyGrid(grid, stamp, loaded);
  EXPECT_EQ(message.header.frame_id, "map");
  EXPECT_EQ(message.header.stamp, stamp);
  EXPECT_EQ(message.info.map_load_time, loaded);
  EXPECT_FLOAT_EQ(message.info.resolution, 1);
  EXPECT_EQ(message.info.width, 5u);
  EXPECT_EQ(message.info.height, 5u);
  EXPECT_EQ(message.info.origin.position.x, -2);
  EXPECT_EQ(message.info.origin.position.y, -3);
  EXPECT_EQ(message.info.origin.orientation.w, 1);
  EXPECT_EQ(message.data[2], 70);
  EXPECT_EQ(message.data[5], -1);
}
