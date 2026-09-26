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
GridConfig geometryFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid & m)
{
  const auto & o = m.info.origin;
  if (m.header.frame_id != "map" || !std::isfinite(o.position.z) ||
    std::abs(o.position.z) > 1e-9 || !std::isfinite(o.orientation.w) ||
    !std::isfinite(o.orientation.x) || !std::isfinite(o.orientation.y) ||
    !std::isfinite(o.orientation.z) || std::abs(o.orientation.x) > 1e-9 ||
    std::abs(o.orientation.y) > 1e-9 || std::abs(o.orientation.z) > 1e-9 ||
    std::abs(std::abs(o.orientation.w)-1) > 1e-9 ||
    m.info.width > 4000000 || m.info.height > 4000000) {
    throw std::invalid_argument("Only finite, axis-aligned map grids are supported");
  }
  GridConfig g; g.resolution = m.info.resolution;
  g.width = static_cast<int>(m.info.width); g.height = static_cast<int>(m.info.height);
  g.origin = {o.position.x, o.position.y};
  OccupancyGrid2D validate(g);
  if (m.data.size() != static_cast<std::size_t>(g.width)*g.height) {
    throw std::invalid_argument("Map data length does not match geometry");
  }
  return g;
}

nav_msgs::msg::OccupancyGrid toOccupancyGrid(const OccupancyGrid2D & grid,
  const builtin_interfaces::msg::Time & stamp, const builtin_interfaces::msg::Time & loaded)
{
  nav_msgs::msg::OccupancyGrid message;
  message.header.frame_id = "map";
  message.header.stamp = stamp;
  message.info.map_load_time = loaded;
  message.info.resolution = static_cast<float>(grid.config().resolution);
  message.info.width = grid.config().width;
  message.info.height = grid.config().height;
  message.info.origin.position.x = grid.config().origin.x();
  message.info.origin.position.y = grid.config().origin.y();
  message.info.origin.orientation.w = 1.0;
  message.data = grid.occupancy();
  return message;
}
}  // namespace motion2d
