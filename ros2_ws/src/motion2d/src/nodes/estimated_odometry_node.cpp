#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include "motion2d/ros/mapping_messages.hpp"

/** @brief Select sensor estimates as /odometry and sole owner of odom->base_link.
 * @details map=odom until ch10; then disable publish_map_alignment and let the
 * pose graph own map->odom. Does not read ground truth, sensor data or TF.
 */
class EstimatedOdometry : public rclcpp::Node
{
public:
  EstimatedOdometry() : Node("estimated_odometry"), dynamic_(*this), fixed_(*this)
  {
    output_ = create_publisher<nav_msgs::msg::Odometry>("/odometry", 100);
    path_output_ = create_publisher<nav_msgs::msg::Path>("/path/estimated", 1);
    input_ = create_subscription<nav_msgs::msg::Odometry>("/odometry/estimated", 100,
      [this](const nav_msgs::msg::Odometry & message) {
        try {motion2d::poseFromOdometry(message);}
        catch (const std::invalid_argument & error) {
          RCLCPP_WARN(get_logger(), "%s", error.what()); return;
        }
        const auto stamp = rclcpp::Time(message.header.stamp).nanoseconds();
        if (stamp == 0 || stamp < last_stamp_) {path_.poses.clear();}
        if (stamp > 0 && stamp == last_stamp_) {return;}
        last_stamp_ = stamp;
        geometry_msgs::msg::TransformStamped tf;
        tf.header = message.header; tf.child_frame_id = "base_link";
        tf.transform.translation.x = message.pose.pose.position.x;
        tf.transform.translation.y = message.pose.pose.position.y;
        tf.transform.rotation = message.pose.pose.orientation;
        dynamic_.sendTransform(tf);
        output_->publish(message);
        geometry_msgs::msg::PoseStamped pose;
        pose.header = message.header; pose.pose = message.pose.pose;
        path_.header = message.header; path_.poses.push_back(pose);
        if (path_.poses.size() > 5000) {path_.poses.erase(path_.poses.begin());}
        path_output_->publish(path_);
      });
    if (declare_parameter("publish_map_alignment", true)) {
      publishAlignment();
      timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publishAlignment();});
    }
  }
private:
  void publishAlignment()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "map"; tf.child_frame_id = "odom";
    tf.transform.rotation.w = 1;
    fixed_.sendTransform(tf);
  }
  std::int64_t last_stamp_ = -1;
  nav_msgs::msg::Path path_;
  tf2_ros::TransformBroadcaster dynamic_;
  tf2_ros::StaticTransformBroadcaster fixed_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_output_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr input_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstimatedOdometry>());
  rclcpp::shutdown();
}
