#include <chrono>
#include <memory>
#include <stdexcept>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace motion2d
{
/** @brief Explicit truth-mode adapter; owns map->odom and odom->base_link.
 * @details world, map and odom share the same origin/axes in this chapter.
 * Never launch this node with a SLAM TF source or simulator.publish_truth_tf=true.
 * The body origin is the disk centre; no first-pose rebasing is performed.
 */
class TruthOdometry : public rclcpp::Node
{
public:
  TruthOdometry() : Node("truth_odometry"), dynamic_(*this), fixed_(*this)
  {
    output_ = create_publisher<nav_msgs::msg::Odometry>("/odometry", rclcpp::QoS(100).reliable());
    input_ = create_subscription<nav_msgs::msg::Odometry>("/ground_truth/odometry",
      rclcpp::QoS(100).reliable().transient_local(),
      [this](const nav_msgs::msg::Odometry & truth) {
        if (truth.header.frame_id != "world" || truth.child_frame_id != "ground_truth_base") {
          RCLCPP_ERROR(get_logger(), "Truth adapter expects world / ground_truth_base");
          return;
        }
        // truth_adapter_begin
        auto selected = truth;
        selected.header.frame_id = "odom";
        selected.child_frame_id = "base_link";
        geometry_msgs::msg::TransformStamped transform;
        transform.header = selected.header;
        transform.child_frame_id = selected.child_frame_id;
        transform.transform.translation.x = selected.pose.pose.position.x;
        transform.transform.translation.y = selected.pose.pose.position.y;
        transform.transform.rotation = selected.pose.pose.orientation;
        dynamic_.sendTransform(transform);
        output_->publish(selected);
        // truth_adapter_end
      });
    publishAlignment();
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publishAlignment();});
    RCLCPP_INFO(get_logger(), "TRUTH localization: world=map=odom; source /ground_truth/odometry");
  }

private:
  void publishAlignment()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";
    transform.transform.rotation.w = 1;
    fixed_.sendTransform(transform);
  }

  tf2_ros::TransformBroadcaster dynamic_;
  tf2_ros::StaticTransformBroadcaster fixed_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr input_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace motion2d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motion2d::TruthOdometry>());
  rclcpp::shutdown();
}
