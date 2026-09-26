#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "motion2d/estimation/lidar_odometry.hpp"
#include "motion2d/ros/mapping_messages.hpp"

/** @brief Scan-only ROS boundary. Publishes estimates, never subscribes to truth or TF. */
class LidarOdometryNode : public rclcpp::Node
{
public:
  LidarOdometryNode() : Node("lidar_odometry")
  {
    motion2d::LidarOdometryConfig config;
    const auto metric = declare_parameter("estimation.metric", "line");
    if (metric != "point" && metric != "line") {throw std::invalid_argument("metric: point or line");}
    config.match.metric = metric == "line" ? motion2d::MatchMetric::PointToLine : motion2d::MatchMetric::PointToPoint;
    config.match.association_distance = declare_parameter("estimation.association_distance", .5);
    config.match.max_rmse = declare_parameter("estimation.max_rmse", .15);
    config.voxel_size = declare_parameter("estimation.voxel_size", .06);
    config.normal_radius = declare_parameter("estimation.normal_radius", .4);
    config.max_keyframes = declare_parameter("estimation.max_keyframes", 5);
    config.keyframe_distance = declare_parameter("estimation.keyframe_distance", .2);
    config.keyframe_angle = declare_parameter("estimation.keyframe_angle", .15);
    config.max_gap = declare_parameter("estimation.max_gap", 1.0);
    odometry_ = std::make_unique<motion2d::LidarOdometry>(config);
    output_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/estimated", 100);
    status_ = create_publisher<std_msgs::msg::String>("/estimation/status", 10);
    submap_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud/local_map", 1);
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan & scan) {onScan(scan);});
  }
private:
  void onScan(const sensor_msgs::msg::LaserScan & scan)
  {
    try {
      const auto result = odometry_->update(motion2d::projectScan(motion2d::fromLaserScan(scan)),
        rclcpp::Time(scan.header.stamp).nanoseconds());
      std_msgs::msg::String status;
      status.data = result.status;
      status_->publish(status);
      if (!result.accepted) {return;}
      nav_msgs::msg::Odometry message;
      message.header = scan.header; message.header.frame_id = "odom";
      message.child_frame_id = "base_link";
      message.pose.pose.position.x = result.pose.position.x();
      message.pose.pose.position.y = result.pose.position.y();
      message.pose.pose.orientation.z = std::sin(result.pose.yaw / 2);
      message.pose.pose.orientation.w = std::cos(result.pose.yaw / 2);
      // Fixed measurement scales for teaching, not a claim of calibrated uncertainty.
      for (int i : {0, 7}) {message.pose.covariance[i] = .03 * .03;}
      message.pose.covariance[35] = .01 * .01;
      for (int i : {14, 21, 28}) {message.pose.covariance[i] = 1e6;}
      const Eigen::Vector2d body = motion2d::rotation(result.pose.yaw).transpose() * result.velocity;
      message.twist.twist.linear.x = body.x(); message.twist.twist.linear.y = body.y();
      message.twist.twist.angular.z = result.yaw_rate;
      for (int i : {0, 7, 14, 21, 28, 35}) {message.twist.covariance[i] = 1e6;}
      output_->publish(message);
      submap_->publish(motion2d::toPointCloud(odometry_->submap(), message.header));
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN(get_logger(), "%s", error.what());
    }
  }
  std::unique_ptr<motion2d::LidarOdometry> odometry_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<LidarOdometryNode>());}
  catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_odometry"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown();
}
