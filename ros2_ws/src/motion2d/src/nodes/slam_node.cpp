#include <chrono>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include "motion2d/estimation/keyframes.hpp"
#include "motion2d/ros/mapping_messages.hpp"

/** @brief Small-scene SLAM back end; front-end scan/IMU estimates stay in odom.
 * @details Exact-time /scan + /odometry/scan only, no true world or TF input.
 * Publishes rebuilt map products and is the only owner of map->odom in ch10.
 */
class SlamNode : public rclcpp::Node
{
public:
  SlamNode() : Node("slam"), alignment_tf_(*this)
  {
    motion2d::KeyframeConfig config;
    config.enable_loops = declare_parameter("slam.enable_loops", true);
    config.distance = declare_parameter("slam.keyframe_distance", .3);
    config.angle = declare_parameter("slam.keyframe_angle", .25);
    config.max_seconds = declare_parameter("slam.keyframe_seconds", 2.);
    config.candidate_distance = declare_parameter("slam.candidate_distance", .8);
    const int separation = declare_parameter("slam.min_separation", 20);
    const int capacity = declare_parameter("slam.max_frames", 200);
    if (separation < 2 || capacity < 2) {throw std::invalid_argument("invalid slam keyframe counts");}
    config.min_separation = separation; config.max_frames = capacity;
    slam_ = std::make_unique<motion2d::KeyframeSlam>(config);
    grid_config_.resolution = declare_parameter("mapping.resolution", .1);
    grid_config_.width = declare_parameter("mapping.width_cells", 220);
    grid_config_.height = declare_parameter("mapping.height_cells", 220);
    grid_config_.origin.x() = declare_parameter("mapping.origin_x", -11.);
    grid_config_.origin.y() = declare_parameter("mapping.origin_y", -11.);
    motion2d::OccupancyGrid2D validate_grid(grid_config_);
    const auto retained = rclcpp::QoS(1).reliable().transient_local();
    map_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", retained);
    cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("/map/cloud", retained);
    registered_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud/registered", rclcpp::SensorDataQoS());
    before_ = create_publisher<nav_msgs::msg::Path>("/path/graph_before", retained);
    after_ = create_publisher<nav_msgs::msg::Path>("/path/graph_after", retained);
    edges_ = create_publisher<visualization_msgs::msg::MarkerArray>("/slam/edges", retained);
    status_ = create_publisher<std_msgs::msg::String>("/slam/status", retained);
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan & m) {onScan(m);});
    pose_ = create_subscription<nav_msgs::msg::Odometry>("/odometry/scan", 100,
      [this](const nav_msgs::msg::Odometry & m) {onPose(m);});
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {publishAlignment();});
  }
private:
  void onScan(const sensor_msgs::msg::LaserScan & m)
  {
    try {
      auto scan = motion2d::fromLaserScan(m);
      const auto stamp = rclcpp::Time(m.header.stamp).nanoseconds();
      if (stamp == 0 || stamp < last_scan_) {
        scans_.clear(); slam_->reset(); last_processed_ = -1;
        poses_.erase(poses_.upper_bound(0), poses_.end());
      }
      last_scan_ = stamp;
      if (stamp <= last_processed_) {return;}
      scans_[stamp] = std::move(scan);
      while (scans_.size() > 20) {scans_.erase(scans_.begin());}
      processReady();
    } catch (const std::invalid_argument & error) {RCLCPP_WARN(get_logger(), "%s", error.what());}
  }
  void onPose(const nav_msgs::msg::Odometry & m)
  {
    try {
      const auto pose = motion2d::poseFromOdometry(m);
      const auto stamp = rclcpp::Time(m.header.stamp).nanoseconds();
      if (stamp < last_pose_) {poses_.clear(); scans_.erase(scans_.upper_bound(stamp), scans_.end());}
      last_pose_ = stamp; poses_[stamp] = pose;
      while (poses_.size() > 100) {poses_.erase(poses_.begin());}
      processReady();
    } catch (const std::invalid_argument & error) {RCLCPP_WARN(get_logger(), "%s", error.what());}
  }
  void processReady()
  {
    while (!scans_.empty()) {
      auto scan = scans_.begin();
      while (scan != scans_.end() && poses_.count(scan->first) == 0) {++scan;}
      if (scan == scans_.end()) {return;}
      scans_.erase(scans_.begin(), scan);
      const auto pose = poses_.at(scan->first);
      const auto result = slam_->update(scan->second, pose, scan->first);
      std_msgs::msg::Header header;
      header.stamp = rclcpp::Time(scan->first); header.frame_id = "map";
      registered_->publish(motion2d::toPointCloud(motion2d::registerPoints(motion2d::projectScan(scan->second),
        motion2d::compose(slam_->mapToOdom(), pose)), header));
      std_msgs::msg::String status;
      status.data = result.status + " frames=" + std::to_string(slam_->frames().size()) +
        " loops=" + std::to_string(slam_->loopCount());
      status_->publish(status);
      if (result.keyframe_added) {publishMap(header);}
      last_processed_ = scan->first; scans_.erase(scan);
    }
  }
  void publishMap(const std_msgs::msg::Header & header)
  {
    const auto products = motion2d::rebuildKeyframeMap(slam_->frames(), grid_config_);
    if (products.outside_scans) {RCLCPP_WARN(get_logger(), "%zu keyframes outside fixed grid", products.outside_scans);}
    map_->publish(motion2d::toOccupancyGrid(products.grid, header.stamp,
      rclcpp::Time(slam_->frames().front().stamp)));
    cloud_->publish(motion2d::toPointCloud(products.cloud, header));
    nav_msgs::msg::Path before, after; before.header = after.header = header;
    for (const auto & frame : slam_->frames()) {
      geometry_msgs::msg::PoseStamped p; p.header = header; p.header.stamp = rclcpp::Time(frame.stamp);
      auto set = [&p](const motion2d::Pose2D & pose) {
        p.pose.position.x = pose.position.x(); p.pose.position.y = pose.position.y();
        p.pose.orientation.z = std::sin(pose.yaw/2); p.pose.orientation.w = std::cos(pose.yaw/2);
      };
      set(frame.odom_pose); before.poses.push_back(p);  // Baseline in the fixed first-frame gauge.
      set(frame.map_pose); after.poses.push_back(p);
    }
    before_->publish(before); after_->publish(after);
    visualization_msgs::msg::MarkerArray markers;
    for (int type = 0; type < 2; ++type) {
      visualization_msgs::msg::Marker marker;
      marker.header = header; marker.ns = "pose_graph"; marker.id = type;
      marker.type = visualization_msgs::msg::Marker::LINE_LIST; marker.pose.orientation.w = 1.;
      marker.scale.x = type ? .05 : .02; marker.color.a = 1.;
      marker.color.r = type ? .8 : .1; marker.color.g = type ? .2 : .6; marker.color.b = .7;
      for (const auto & edge : slam_->edges()) {
        if (edge.loop != static_cast<bool>(type)) {continue;}
        for (auto index : {edge.from, edge.to}) {
          geometry_msgs::msg::Point p; const auto & pos = slam_->frames()[index].map_pose.position;
          p.x = pos.x(); p.y = pos.y(); p.z = .05; marker.points.push_back(p);
        }
      }
      markers.markers.push_back(marker);
    }
    edges_->publish(markers);
    publishAlignment();
  }
  void publishAlignment()
  {
    if (slam_->frames().empty()) {return;}
    const auto pose = slam_->mapToOdom();
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now(); tf.header.frame_id = "map"; tf.child_frame_id = "odom";
    tf.transform.translation.x = pose.position.x(); tf.transform.translation.y = pose.position.y();
    tf.transform.rotation.z = std::sin(pose.yaw/2); tf.transform.rotation.w = std::cos(pose.yaw/2);
    alignment_tf_.sendTransform(tf);
  }
  std::unique_ptr<motion2d::KeyframeSlam> slam_;
  motion2d::GridConfig grid_config_;
  std::map<std::int64_t, motion2d::Scan2D> scans_;
  std::map<std::int64_t, motion2d::Pose2D> poses_;
  std::int64_t last_scan_ = -1, last_pose_ = -1, last_processed_ = -1;
  tf2_ros::TransformBroadcaster alignment_tf_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_, registered_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr before_, after_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr edges_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<SlamNode>());}
  catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("slam"), "%s", error.what()); rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown();
}
