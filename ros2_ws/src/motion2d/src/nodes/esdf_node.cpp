#include <algorithm>
#include <bit>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include "motion2d/mapping/esdf.hpp"
#include "motion2d/ros/mapping_messages.hpp"

namespace motion2d
{
/** @brief Observed-map distance and gradient visualization; no truth/TF input.
 * @details Rebuilds a field from each independent map snapshot. The PointCloud2
 * distance channel is signed centre distance in metres, not occupancy or height.
 */
class EsdfNode : public rclcpp::Node
{
public:
  EsdfNode() : Node("esdf")
  {
    threshold_ = declare_parameter("planning.free_threshold", 35);
    unknown_ = declare_parameter("planning.unknown_blocked", true);
    stride_ = declare_parameter("esdf.gradient_stride", 10);
    if (stride_ <= 0 || threshold_ < 0 || threshold_ > 100) {throw std::invalid_argument("invalid ESDF parameters");}
    const auto retained = rclcpp::QoS(1).transient_local();
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/esdf/cloud", retained);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/esdf/gradients", retained);
    status_pub_ = create_publisher<std_msgs::msg::String>("/esdf/status", retained);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", retained,
      [this](const nav_msgs::msg::OccupancyGrid & m) {receive(m);});
  }
private:
  void receive(const nav_msgs::msg::OccupancyGrid & map)
  {
    try {
      const auto g = geometryFromOccupancyGrid(map);
      const Esdf2D field(g, map.data, threshold_, unknown_);
      sensor_msgs::msg::PointCloud2 cloud; cloud.header = map.header;
      cloud.height = g.height; cloud.width = g.width; cloud.is_dense = true;
      cloud.is_bigendian = std::endian::native == std::endian::big;
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      const auto f32 = sensor_msgs::msg::PointField::FLOAT32;
      modifier.setPointCloud2Fields(4, "x", 1, f32, "y", 1, f32, "z", 1, f32, "distance", 1, f32);
      modifier.resize(map.data.size());
      sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"), d(cloud, "distance");
      for (int i = 0; i < static_cast<int>(map.data.size()); ++i, ++x, ++y, ++z, ++d) {
        *x = g.origin.x()+(i%g.width+.5)*g.resolution;
        *y = g.origin.y()+(i/g.width+.5)*g.resolution; *z = 0;
        *d = field.distances()[i];
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*d)) {cloud.is_dense = false;}
      }
      cloud_pub_->publish(cloud);
      publishGradients(field, map.header);
      std_msgs::msg::String status;
      status.data = cloud.is_dense ? "ready" : "no_finite_field"; status_pub_->publish(status);
    } catch (const std::invalid_argument & e) {
      sensor_msgs::msg::PointCloud2 empty; empty.header = map.header; empty.header.frame_id = "map";
      empty.height = 1; cloud_pub_->publish(empty);
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker clear; clear.action = clear.DELETEALL; markers.markers.push_back(clear);
      marker_pub_->publish(markers);
      std_msgs::msg::String status; status.data = "invalid_map"; status_pub_->publish(status);
      RCLCPP_WARN(get_logger(), "%s", e.what());
    }
  }
  void publishGradients(const Esdf2D & field, const std_msgs::msg::Header & header)
  {
    visualization_msgs::msg::Marker marker; marker.header = header; marker.ns = "esdf_gradient";
    marker.id = 0; marker.type = marker.LINE_LIST; marker.action = marker.ADD;
    marker.pose.orientation.w = 1; marker.scale.x = .015;
    marker.color.r = .75; marker.color.g = .32; marker.color.b = .06; marker.color.a = 1.;
    const auto & g = field.geometry();
    auto add = [&](const Eigen::Vector2d & p) {
        geometry_msgs::msg::Point point; point.x = p.x(); point.y = p.y(); point.z = .04;
        marker.points.push_back(point);
      };
    for (int y = 1; y < g.height-1; y += stride_) {
      for (int x = 1; x < g.width-1; x += stride_) {
        const Eigen::Vector2d p = g.origin+g.resolution*Eigen::Vector2d(x+.5, y+.5);
        const auto s = field.sample(p);
        if (!s || s->distance <= 0 || s->gradient.norm() < 1e-6) {continue;}
        // esdf_arrow_begin
        const Eigen::Vector2d direction = s->gradient.normalized();
        const Eigen::Vector2d normal(-direction.y(), direction.x());
        const double length = std::min(.4, .2*s->gradient.norm());
        const double head = std::min(.06, .3*length);
        const Eigen::Vector2d end = p+length*direction;
        add(p); add(end);
        add(end); add(end-head*direction+.4*head*normal);
        add(end); add(end-head*direction-.4*head*normal);
        // esdf_arrow_end
      }
    }
    visualization_msgs::msg::MarkerArray array; array.markers.push_back(std::move(marker));
    marker_pub_->publish(array);
  }
  int threshold_, stride_;
  bool unknown_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};
}  // namespace motion2d
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<motion2d::EsdfNode>());}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("esdf"), "%s", e.what()); return 1;}
  rclcpp::shutdown(); return 0;
}
