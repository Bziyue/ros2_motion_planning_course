#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "motion2d/ros/world_parameters.hpp"

namespace motion2d
{
namespace
{
geometry_msgs::msg::Point point(const Eigen::Vector2d & p, double z = 0.1)
{
  geometry_msgs::msg::Point result;
  result.x = p.x();
  result.y = p.y();
  result.z = z;
  return result;
}

visualization_msgs::msg::Marker marker(const std::string & ns, int id, int type)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.pose.orientation.w = 1.0;
  m.scale.x = m.scale.y = m.scale.z = 1.0;
  m.color.r = 0.35F;
  m.color.g = 0.53F;
  m.color.b = 0.63F;
  m.color.a = 1.0F;
  return m;
}
}  // namespace

/** @brief Publish complete simulation geometry for viewing, not a sensed map. */
class WorldScene : public rclcpp::Node
{
public:
  WorldScene() : Node("world_scene")
  {
    const WorldConfig c = readWorldConfig(*this);
    const World2D world = generateWorld(c);
    markers_ = drawWorld(world);
    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/visualization/world", rclcpp::QoS(1).reliable().transient_local());
    publisher_->publish(markers_);
    // Geometry does not change. Repeating it lets RViz recover after time reset.
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publisher_->publish(markers_);});
    RCLCPP_INFO(get_logger(),
      "seed=%u; circles=%zu; polygons=%zu; start clearance=%.3f m; goal clearance=%.3f m; "
      "connectivity=%s",
      c.seed, world.circles.size(), world.polygons.size(),
      clearance(world, world.start), clearance(world, world.goal),
      c.require_connected ? "passed conservative check" : "not requested");
  }

private:
  visualization_msgs::msg::MarkerArray drawWorld(const World2D & world)
  {
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray result;
    Marker clear;
    clear.action = Marker::DELETEALL;
    result.markers.push_back(clear);
    auto boundary = marker("boundary", 0, Marker::LINE_STRIP);
    boundary.scale.x = 0.06;
    boundary.color.r = 0.1F;
    boundary.color.g = 0.2F;
    boundary.color.b = 0.3F;
    const double x = world.width / 2.0, y = world.height / 2.0;
    for (const auto & p : std::vector<Eigen::Vector2d>{
        {-x, -y}, {x, -y}, {x, y}, {-x, y}, {-x, -y}})
    {
      boundary.points.push_back(point(p));
    }
    result.markers.push_back(boundary);
    int id = 0;
    for (const auto & circle : world.circles) {
      auto m = marker("circles", id++, Marker::CYLINDER);
      m.pose.position = point(circle.center);
      m.scale.x = m.scale.y = 2.0 * circle.radius;
      m.scale.z = 0.2;
      result.markers.push_back(m);
    }
    id = 0;
    for (const auto & polygon : world.polygons) {
      auto m = marker("polygons", id++, Marker::TRIANGLE_LIST);
      m.color.g = 0.65F;
      // A convex CCW polygon is a fan of non-overlapping triangles.
      for (std::size_t i = 1; i + 1 < polygon.vertices.size(); ++i) {
        m.points.push_back(point(polygon.vertices[0]));
        m.points.push_back(point(polygon.vertices[i]));
        m.points.push_back(point(polygon.vertices[i + 1]));
      }
      result.markers.push_back(m);
    }
    id = 0;
    for (const auto & p : {world.start, world.goal}) {
      auto dot = marker("endpoints", id, Marker::SPHERE);
      dot.pose.position = point(p, 0.2);
      dot.scale.x = dot.scale.y = dot.scale.z = id == 0 ? 0.18 : 0.4;
      dot.color.r = id == 0 ? 0.1F : 0.9F;
      dot.color.g = id == 0 ? 0.65F : 0.4F;
      dot.color.b = 0.1F;
      result.markers.push_back(dot);
      auto text = dot;
      text.ns = "labels";
      text.type = Marker::TEXT_VIEW_FACING;
      text.pose.position.y += 0.6;
      text.scale.z = 0.45;
      text.text = id == 0 ? "Start" : "Goal";
      result.markers.push_back(text);
      ++id;
    }
    return result;
  }

  visualization_msgs::msg::MarkerArray markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace motion2d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<motion2d::WorldScene>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("world_scene"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
