#include <cstdint>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include "motion2d/ros/mapping_messages.hpp"

namespace
{
std::int64_t stampNs(const builtin_interfaces::msg::Time & stamp)
{
  return rclcpp::Time(stamp).nanoseconds();
}
}

/** @brief Observe /scan using exact-time /odometry; never reads true geometry or TF.
 * @details Single-threaded callbacks and small bounded queues make arrival order
 * explicit. The simulator emits an odometry sample at every scan timestamp.
 * Real unsynchronized sensors will require interpolation, introduced separately.
 */
class Mapping : public rclcpp::Node
{
public:
  Mapping() : Node("mapping")
  {
    laser_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud/scan", rclcpp::SensorDataQoS());
    registered_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud/registered", rclcpp::SensorDataQoS());
    odometry_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", rclcpp::QoS(100).reliable(),
      [this](const nav_msgs::msg::Odometry & message) {onOdometry(message);});
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan & message) {onScan(message);});
    RCLCPP_INFO(get_logger(), "Exact-time scan projection; fixed laser=base_link, map=odom");
  }

private:
  struct PendingScan
  {
    motion2d::Scan2D scan;
    std_msgs::msg::Header header;
  };

  void onOdometry(const nav_msgs::msg::Odometry & message)
  {
    try {
      const auto pose = motion2d::poseFromOdometry(message);
      const auto stamp = stampNs(message.header.stamp);
      if (stamp < last_odometry_) {
        poses_.clear();
        // A new t=0 scan may have arrived first. Positive pending scans are old.
        pending_.erase(pending_.upper_bound(stamp), pending_.end());
      }
      last_odometry_ = stamp;
      poses_[stamp] = pose;
      while (poses_.size() > 1000) {poses_.erase(poses_.begin());}
      processReady();
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN(get_logger(), "%s", error.what());
    }
  }

  void onScan(const sensor_msgs::msg::LaserScan & message)
  {
    try {
      auto scan = motion2d::fromLaserScan(message);
      const auto stamp = stampNs(message.header.stamp);
      if (stamp < last_scan_) {
        pending_.clear();
        // Reset is paused at t=0. Keep its pose if the odometry arrived first.
        // The fixed initial pose at t=0 is identical in every trial.
        poses_.erase(poses_.upper_bound(0), poses_.end());
        last_processed_ = -1;
      }
      last_scan_ = stamp;
      if (stamp <= last_processed_) {return;}  // Duplicate delivery is not a new observation.
      pending_[stamp] = {std::move(scan), message.header};
      while (pending_.size() > 20) {
        RCLCPP_WARN(get_logger(), "Dropping oldest scan: exact-time odometry missing");
        pending_.erase(pending_.begin());
      }
      processReady();
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN(get_logger(), "%s", error.what());
    }
  }

  void processReady()
  {
    while (!pending_.empty()) {
      const auto scan = pending_.begin();
      const auto pose = poses_.find(scan->first);
      if (pose == poses_.end()) {break;}
      // scan_pose_join_begin
      const auto points = motion2d::projectScan(scan->second.scan);
      laser_cloud_->publish(motion2d::toPointCloud(points, scan->second.header));
      auto header = scan->second.header;
      header.frame_id = "odom";
      registered_cloud_->publish(motion2d::toPointCloud(
        motion2d::registerPoints(points, pose->second), header));
      // scan_pose_join_end
      last_processed_ = scan->first;
      pending_.erase(scan);
    }
  }

  std::map<std::int64_t, motion2d::Pose2D> poses_;
  std::map<std::int64_t, PendingScan> pending_;
  std::int64_t last_odometry_ = -1, last_scan_ = -1, last_processed_ = -1;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr laser_cloud_, registered_cloud_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mapping>());
  rclcpp::shutdown();
  return 0;
}
