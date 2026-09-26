#include "motion2d/ros/lidar_messages.hpp"
#include <cmath>

namespace motion2d
{
sensor_msgs::msg::LaserScan toLaserScan(const LidarConfig & config,
  const std::vector<float> & ranges, const builtin_interfaces::msg::Time & stamp, double period)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = "laser";
  scan.angle_min = static_cast<float>(config.angle_min);
  scan.angle_increment = static_cast<float>(beamIncrement(config));
  scan.angle_max = scan.angle_min + (config.beams - 1) * scan.angle_increment;
  scan.time_increment = 0.0F;
  scan.scan_time = static_cast<float>(period);
  scan.range_min = static_cast<float>(config.range_min);
  scan.range_max = static_cast<float>(config.range_max);
  scan.ranges = ranges;
  return scan;  // intensities stays empty: no reflection/intensity model.
}

visualization_msgs::msg::MarkerArray lidarBeamMarkers(
  const sensor_msgs::msg::LaserScan & scan, const Pose2D & pose)
{
  using Marker = visualization_msgs::msg::Marker;
  Marker hit;
  hit.header = scan.header;
  hit.header.frame_id = "odom";  // Retained markers must not move with the current robot.
  hit.ns = "lidar_hits";
  hit.type = Marker::LINE_LIST;
  hit.pose.orientation.w = 1.0;
  hit.scale.x = .009;
  hit.color.r = .95F;
  hit.color.g = .45F;
  hit.color.b = .08F;
  hit.color.a = .28F;
  Marker miss = hit;
  miss.ns = "lidar_no_return";
  miss.color.r = .2F;
  miss.color.g = .55F;
  miss.color.b = .85F;
  miss.color.a = .10F;
  geometry_msgs::msg::Point origin;
  origin.x = pose.position.x();
  origin.y = pose.position.y();
  origin.z = .10;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    const bool no_return = std::isinf(range) && range > 0;
    if (!no_return && !std::isfinite(range)) {continue;}
    const double angle = pose.yaw + scan.angle_min + i * static_cast<double>(scan.angle_increment);
    const double length = no_return ? scan.range_max : range;
    auto endpoint = origin;
    endpoint.x += length * std::cos(angle);
    endpoint.y += length * std::sin(angle);
    auto & points = no_return ? miss.points : hit.points;
    points.push_back(origin);
    points.push_back(endpoint);
  }
  hit.action = hit.points.empty() ? Marker::DELETE : Marker::ADD;
  miss.action = miss.points.empty() ? Marker::DELETE : Marker::ADD;
  visualization_msgs::msg::MarkerArray markers;
  markers.markers = {hit, miss};
  return markers;
}
}  // namespace motion2d
