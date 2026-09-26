#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "motion2d/geometry/se2.hpp"
#include "motion2d/sim/sim_clock.hpp"

namespace motion2d
{
/** @brief Chapter 02 coordinate rig: an analytic rotation, not a dynamics model. */
class FrameDemo : public rclcpp::Node
{
public:
  FrameDemo() : Node("frame_demo"), clock_(readStep()),
    broadcaster_(*this), static_broadcaster_(*this)
  {
    x_ = declare_parameter("x", 1.0);
    y_ = declare_parameter("y", 2.0);
    yaw_ = declare_parameter("yaw", kPi / 2.0);
    yaw_rate_ = declare_parameter("yaw_rate", 0.2);
    radius_ = declare_parameter("radius", 0.2);
    if (!std::isfinite(x_) || !std::isfinite(y_) || !std::isfinite(yaw_) ||
      !std::isfinite(yaw_rate_) || !std::isfinite(radius_) || radius_ <= 0.0)
    {
      throw std::invalid_argument("Pose/rate must be finite and radius positive");
    }
    clock_.setPaused(declare_parameter("start_paused", false));
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(1));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/visualization/frames", rclcpp::QoS(1).reliable().transient_local());

    publishStaticFrames();

    pause_service_ = create_service<std_srvs::srv::SetBool>(
      "/sim/pause", [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response) {
        clock_.setPaused(request->data);
        response->success = true;
        response->message = request->data ? "paused" : "running";
      });
    step_service_ = create_service<std_srvs::srv::Trigger>(
      "/sim/step", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = clock_.singleStep();
        response->message = response->success ? "advanced one tick" : "pause before stepping";
        if (response->success) {
          publishClock();
          publishFrames();
        }
      });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/sim/reset", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        clock_.reset();
        publishClock();
        publishFrames();
        response->success = true;
        response->message = "new paused run at t=0";
      });
    timer_ = create_wall_timer(clock_.stepDuration(), [this]() {
        const bool advanced = clock_.advance();
        // Repeat the same clock stamp while paused, so late subscribers get time.
        publishClock();
        if (advanced) {
          publishFrames();
        }
      });
    // A time reset clears consumers' TF caches. Repeat the static geometry so
    // they can recover even if /clock and /tf_static arrive in different order.
    static_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        publishStaticFrames();
        if (clock_.paused()) {publishFrames();}
      });
    publishClock();
    publishFrames();
    RCLCPP_INFO(get_logger(), "Coordinate rig: /clock, TF, /sim/pause, /sim/step, /sim/reset");
  }

private:
  std::chrono::nanoseconds readStep()
  {
    const double dt = declare_parameter("dt", 0.005);
    if (!std::isfinite(dt) || dt < 1e-6 || dt > 1.0) {
      throw std::invalid_argument("dt must be in [1e-6, 1] seconds");
    }
    return std::chrono::nanoseconds(std::llround(dt * 1e9));
  }

  void publishStaticFrames()
  {
    std::vector<geometry_msgs::msg::TransformStamped> fixed;
    for (const auto & pair : std::vector<std::pair<std::string, std::string>>{
        {"map", "odom"}, {"base_link", "laser"}, {"base_link", "imu_link"}})
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = pair.first;
      transform.child_frame_id = pair.second;
      transform.transform.rotation.w = 1.0;
      fixed.push_back(transform);
    }
    static_broadcaster_.sendTransform(fixed);
  }

  void publishClock()
  {
    rosgraph_msgs::msg::Clock message;
    message.clock = rclcpp::Time(clock_.nanoseconds(), RCL_ROS_TIME);
    clock_pub_->publish(message);
  }

  void publishFrames()
  {
    // frame_begin
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Time(clock_.nanoseconds(), RCL_ROS_TIME);
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    const double angle = wrapAngle(yaw_ + yaw_rate_ * clock_.seconds());
    transform.transform.rotation.z = std::sin(angle / 2.0);
    transform.transform.rotation.w = std::cos(angle / 2.0);
    broadcaster_.sendTransform(transform);
    // frame_end

    visualization_msgs::msg::Marker disk;
    disk.header.stamp = transform.header.stamp;
    disk.header.frame_id = "base_link";
    disk.ns = "body";
    disk.type = visualization_msgs::msg::Marker::CYLINDER;
    disk.pose.position.z = 0.025;
    disk.pose.orientation.w = 1.0;
    disk.scale.x = disk.scale.y = 2.0 * radius_;
    disk.scale.z = 0.05;
    disk.color.g = 0.5F;
    disk.color.b = 0.55F;
    disk.color.a = 1.0F;
    auto ray = disk;
    ray.header.frame_id = "laser";
    ray.ns = "example_ray";
    ray.type = visualization_msgs::msg::Marker::ARROW;
    ray.pose.position.z = 0.06;
    ray.points.resize(2);
    ray.points[1].x = 2.0;
    ray.scale.x = 0.035;
    ray.scale.y = 0.09;
    ray.scale.z = 0.12;
    ray.color.r = 0.2F;
    ray.color.b = 0.9F;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers = {disk, ray};
    marker_pub_->publish(markers);
  }

  SimClock clock_;
  double x_, y_, yaw_, yaw_rate_, radius_;
  tf2_ros::TransformBroadcaster broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr step_service_, reset_service_;
  rclcpp::TimerBase::SharedPtr timer_, static_timer_;
};
}  // namespace motion2d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<motion2d::FrameDemo>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("frame_demo"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
