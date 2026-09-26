#include "motion2d/ros/planning_messages.hpp"
#include <array>

namespace motion2d
{
nav_msgs::msg::Path toPath(const std::vector<Eigen::Vector2d> & points, const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path path; path.header = header;
  for (const auto & p : points) {
    geometry_msgs::msg::PoseStamped pose; pose.header = header;
    pose.pose.position.x = p.x(); pose.pose.position.y = p.y(); pose.pose.orientation.w = 1;
    path.poses.push_back(pose);
  }
  return path;
}

visualization_msgs::msg::MarkerArray toCorridorMarkers(
  const std::vector<ConvexRegion> & regions, const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker clear; clear.header = header; clear.action = clear.DELETEALL;
  array.markers.push_back(clear);
  const std::array<std::array<float,3>,5> colors{{{.05F,.5F,.65F},{.85F,.4F,.05F},
    {.15F,.6F,.3F},{.6F,.25F,.65F},{.65F,.25F,.2F}}};
  for (std::size_t i = 0; i < regions.size(); ++i) {
    visualization_msgs::msg::Marker marker; marker.header = header; marker.ns = "safe_corridor";
    marker.id = i; marker.type = marker.LINE_STRIP; marker.action = marker.ADD;
    marker.pose.orientation.w = 1; marker.scale.x = .025;
    const auto & color = colors[i%colors.size()];
    marker.color.r = color[0]; marker.color.g = color[1]; marker.color.b = color[2]; marker.color.a = .9;
    for (const auto & p : regions[i].polygon.vertices) {
      geometry_msgs::msg::Point point; point.x = p.x(); point.y = p.y(); point.z = .03;
      marker.points.push_back(point);
    }
    marker.points.push_back(marker.points.front());
    array.markers.push_back(std::move(marker));
  }
  return array;
}
}  // namespace motion2d
