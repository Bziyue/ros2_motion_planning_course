#include <cmath>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace motion2d
{
/**
 * @brief Publish one stationary disk in map coordinates (chapter 01).
 * @details A shallow CYLINDER represents the 2D footprint. Transient-local QoS
 * retains this sample so RViz can join after the publisher has started.
 */
class HelloScene : public rclcpp::Node
{
public:
  HelloScene() : Node("hello_scene")
  {
    const double radius = declare_parameter("radius", 0.2);
    const double x = declare_parameter("x", 0.0);
    const double y = declare_parameter("y", 0.0);
    if (!std::isfinite(radius) || radius <= 0.0 ||
      !std::isfinite(x) || !std::isfinite(y))
    {
      throw std::invalid_argument("radius must be finite and positive; x/y must be finite");
    }

    publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "/visualization/robot", rclcpp::QoS(1).reliable().transient_local());

    // marker_begin
    visualization_msgs::msg::Marker disk;
    disk.header.frame_id = "map";
    disk.header.stamp = now();
    disk.ns = "robot";
    disk.id = 0;
    disk.type = visualization_msgs::msg::Marker::CYLINDER;
    disk.action = visualization_msgs::msg::Marker::ADD;
    disk.pose.position.x = x;
    disk.pose.position.y = y;
    disk.pose.position.z = 0.025;
    disk.pose.orientation.w = 1.0;
    disk.scale.x = 2.0 * radius;
    disk.scale.y = 2.0 * radius;
    disk.scale.z = 0.05;
    disk.color.r = 0.03F;
    disk.color.g = 0.50F;
    disk.color.b = 0.55F;
    disk.color.a = 1.0F;
    // marker_end
    publisher_->publish(disk);
    RCLCPP_INFO(get_logger(), "Disk at (%.2f, %.2f), radius %.2f m", x, y, radius);
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
};
}  // namespace motion2d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<motion2d::HelloScene>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("hello_scene"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
