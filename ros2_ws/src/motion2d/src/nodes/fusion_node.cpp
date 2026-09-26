#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include "motion2d/estimation/fusion_timeline.hpp"
#include "motion2d/ros/mapping_messages.hpp"

/** @brief Loose IMU/lidar fusion; only sensor-derived messages enter this node.
 * @details /odometry/estimated is high-rate prediction. /odometry/scan contains
 * accepted corrected scan poses for mapping; its original scan stamp is retained.
 * Late corrections affect high-rate output on the next IMU, avoiding duplicate TF.
 */
class FusionNode : public rclcpp::Node
{
public:
  FusionNode() : Node("fusion")
  {
    motion2d::EkfConfig config;
    config.estimate_bias = declare_parameter("fusion.estimate_bias", true);
    config.innovation_gate = declare_parameter("fusion.innovation_gate", 16.3);
    timeline_ = std::make_unique<motion2d::FusionTimeline>(config,
      declare_parameter("fusion.history_seconds", 2.0));
    stale_seconds_ = declare_parameter("fusion.stale_seconds", .5);
    if (!std::isfinite(stale_seconds_) || stale_seconds_ <= 0) {
      throw std::invalid_argument("fusion.stale_seconds must be positive");
    }
    output_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/estimated", 200);
    scan_output_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/scan", 100);
    status_ = create_publisher<std_msgs::msg::String>("/fusion/status", 10);
    imu_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data_raw", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu & message) {onImu(message);});
    laser_ = create_subscription<nav_msgs::msg::Odometry>("/odometry/lidar", 100,
      [this](const nav_msgs::msg::Odometry & message) {onLaser(message);});
  }
private:
  void status(const std::string & value)
  {
    std_msgs::msg::String message; message.data = value; status_->publish(message);
  }
  void onImu(const sensor_msgs::msg::Imu & message)
  {
    try {
      if (message.header.frame_id != "imu_link" || message.angular_velocity_covariance[0] < 0 ||
        message.linear_acceleration_covariance[0] < 0) {
        throw std::invalid_argument("IMU must use aligned imu_link and provide gyro/acceleration");
      }
      motion2d::TimedImu imu;
      imu.stamp = rclcpp::Time(message.header.stamp).nanoseconds();
      imu.sample.acceleration = {message.linear_acceleration.x, message.linear_acceleration.y};
      imu.sample.yaw_rate = message.angular_velocity.z;
      imu.sample.variance = {message.linear_acceleration_covariance[0],
        message.linear_acceleration_covariance[4], message.angular_velocity_covariance[8]};
      if (imu.stamp == 0 || imu.stamp < last_imu_) {
        timeline_->reset(); last_published_ = -1;
        // Each stream announces reset independently; keep a new laser origin
        // that arrived first, but discard the previous experiment's queue.
        if (!laser_origin_waiting_imu_) {pending_.clear();}
        imu_origin_waiting_laser_ = !laser_origin_waiting_imu_;
        laser_origin_waiting_imu_ = false;
      }
      last_imu_ = imu.stamp;
      if (!timeline_->pushImu(imu)) {status(timeline_->status()); return;}
      if (!timeline_->ready()) {status(timeline_->status());}
      processLaser(); publishLatest();
    } catch (const std::invalid_argument & error) {RCLCPP_WARN(get_logger(), "%s", error.what());}
  }
  void onLaser(const nav_msgs::msg::Odometry & message)
  {
    try {
      const auto pose = motion2d::poseFromOdometry(message);
      const auto stamp = rclcpp::Time(message.header.stamp).nanoseconds();
      if (stamp == 0 || stamp < last_laser_) {
        pending_.clear(); last_published_ = -1;
        // An IMU origin may already be followed by several new samples.
        if (!imu_origin_waiting_laser_) {timeline_->reset();}
        laser_origin_waiting_imu_ = !imu_origin_waiting_laser_;
        imu_origin_waiting_laser_ = false;
      }
      if (stamp > 0 && stamp == last_laser_) {return;}
      last_laser_ = stamp; pending_.emplace_back(stamp, pose);
      if (pending_.size() > 50) {pending_.pop_front(); status("laser_queue_overflow");}
      processLaser(); publishLatest();
    } catch (const std::invalid_argument & error) {RCLCPP_WARN(get_logger(), "%s", error.what());}
  }
  void processLaser()
  {
    while (!pending_.empty() && pending_.front().first <= timeline_->newestStamp()) {
      const auto [stamp, pose] = pending_.front(); pending_.pop_front();
      const auto corrected = timeline_->correct(stamp, pose);
      status(timeline_->status());
      if (corrected) {scan_output_->publish(toMessage(*corrected));}
    }
  }
  nav_msgs::msg::Odometry toMessage(const motion2d::FilterSnapshot & snapshot) const
  {
    nav_msgs::msg::Odometry message;
    message.header.stamp = rclcpp::Time(snapshot.stamp);
    message.header.frame_id = "odom"; message.child_frame_id = "base_link";
    const auto & x = snapshot.filter.state();
    const auto & p = snapshot.filter.covariance();
    message.pose.pose.position.x = x[0]; message.pose.pose.position.y = x[1];
    message.pose.pose.orientation.z = std::sin(x[4] / 2);
    message.pose.pose.orientation.w = std::cos(x[4] / 2);
    constexpr int pose_index[]{0, 1, 4}, ros_index[]{0, 1, 5};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        message.pose.covariance[ros_index[i] * 6 + ros_index[j]] = p(pose_index[i], pose_index[j]);
      }
    }
    const Eigen::Matrix2d rt = motion2d::rotation(x[4]).transpose();
    const Eigen::Vector2d body = rt * x.segment<2>(2);
    message.twist.twist.linear.x = body.x(); message.twist.twist.linear.y = body.y();
    // Scan messages are used for pose only; their twist is explicitly unavailable.
    if (snapshot.stamp == timeline_->newestStamp()) {
      message.twist.twist.angular.z = timeline_->latestImu().yaw_rate - x[7];
      Eigen::Matrix<double, 3, 8> j = Eigen::Matrix<double, 3, 8>::Zero();
      j.block<2, 2>(0, 2) = rt; j(0, 4) = body.y(); j(1, 4) = -body.x(); j(2, 7) = -1;
      Eigen::Matrix3d covariance = j * p * j.transpose();
      covariance(2, 2) += timeline_->latestImu().variance.z();
      for (int i = 0; i < 3; ++i) {
        for (int k = 0; k < 3; ++k) {
          message.twist.covariance[ros_index[i] * 6 + ros_index[k]] = covariance(i, k);
        }
      }
    } else {for (int i : {0, 7, 35}) {message.twist.covariance[i] = 1e6;}}
    for (int i : {14, 21, 28}) {message.pose.covariance[i] = message.twist.covariance[i] = 1e6;}
    return message;
  }
  void publishLatest()
  {
    if (!timeline_->ready() || timeline_->latest().stamp <= last_published_) {return;}
    output_->publish(toMessage(timeline_->latest()));
    last_published_ = timeline_->latest().stamp;
    status(timeline_->laserAge() > stale_seconds_ ? "laser_stale" : "imu_predicting");
  }
  std::unique_ptr<motion2d::FusionTimeline> timeline_;
  std::deque<std::pair<std::int64_t, motion2d::Pose2D>> pending_;
  std::int64_t last_imu_ = -1, last_laser_ = -1, last_published_ = -1;
  double stale_seconds_;
  bool imu_origin_waiting_laser_ = false, laser_origin_waiting_imu_ = false;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_, scan_output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr laser_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<FusionNode>());}
  catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("fusion"), "%s", error.what()); rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown();
}
