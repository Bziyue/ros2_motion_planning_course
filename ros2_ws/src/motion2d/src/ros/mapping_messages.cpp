#include "motion2d/ros/mapping_messages.hpp"
#include <bit>
#include <cmath>
#include <stdexcept>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace motion2d
{
Scan2D fromLaserScan(const sensor_msgs::msg::LaserScan & message)
{
  if (message.header.frame_id != "laser" || message.time_increment != 0.0F) {
    throw std::invalid_argument("mapping requires a snapshot scan in laser frame");
  }
  Scan2D scan{message.angle_min, message.angle_increment,
    message.range_min, message.range_max, message.ranges};
  validateScan(scan);
  return scan;
}

Pose2D poseFromOdometry(const nav_msgs::msg::Odometry & message)
{
  const auto & p = message.pose.pose.position;
  const auto & q = message.pose.pose.orientation;
  if (message.header.frame_id != "odom" || message.child_frame_id != "base_link" ||
    !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) ||
    !std::isfinite(q.z) || !std::isfinite(q.w) ||
    std::abs(p.z) > 1e-6 || std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 ||
    std::abs(q.z * q.z + q.w * q.w - 1.0) > 1e-5)
  {
    throw std::invalid_argument("mapping requires a finite planar odom/base_link pose");
  }
  return {{p.x, p.y}, 2.0 * std::atan2(q.z, q.w)};
}

sensor_msgs::msg::PointCloud2 toPointCloud(
  const std::vector<Eigen::Vector2d> & points, const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.is_bigendian = std::endian::native == std::endian::big;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  const auto float32 = sensor_msgs::msg::PointField::FLOAT32;
  modifier.setPointCloud2Fields(3, "x", 1, float32, "y", 1, float32, "z", 1, float32);
  modifier.resize(points.size());
  if (points.empty()) {return cloud;}
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto & point : points) {
    *x = static_cast<float>(point.x());
    *y = static_cast<float>(point.y());
    *z = 0.0F;
    ++x; ++y; ++z;
  }
  return cloud;
}
}  // namespace motion2d
