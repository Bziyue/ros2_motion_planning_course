#include <cmath>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include "motion2d/planning/astar.hpp"
#include "motion2d/ros/mapping_messages.hpp"

namespace motion2d
{
/** @brief Chapter 11: plan in map from an observed grid and current odometry.
 * @details The timer coalesces inputs. No world geometry or truth subscription is used.
 * A Path is a geometric proposal, not an executable trajectory or control command.
 */
class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode() : Node("planner"), buffer_(get_clock()), listener_(buffer_, *this, false)
  {
    config_.radius = declare_parameter("radius", .2);
    config_.margin = declare_parameter("margin", .05);
    config_.free_threshold = declare_parameter("planning.free_threshold", 35);
    config_.unknown_blocked = declare_parameter("planning.unknown_blocked", true);
    diagonal_ = declare_parameter("planning.diagonal", true);
    // Validate startup configuration even before the first map arrives.
    inflateGrid(GridConfig{}, std::vector<int8_t>(220*220, -1), config_);
    const auto retained = rclcpp::QoS(1).transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/plan/path", retained);
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/planning/grid", retained);
    status_pub_ = create_publisher<std_msgs::msg::String>("/plan/status", retained);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", retained,
      [this](const nav_msgs::msg::OccupancyGrid & m) {receiveMap(m);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odometry", 20,
      [this](const nav_msgs::msg::Odometry & m) {
        try {poseFromOdometry(m);} catch (const std::invalid_argument &) {
          odom_.reset(); publish({}, "invalid_odometry"); return;
        }
        if (odom_ && rclcpp::Time(m.header.stamp) < rclcpp::Time(odom_->header.stamp)) {
          goal_.reset(); last_start_.reset();
          if (map_stamp_ > rclcpp::Time(m.header.stamp)) {grid_.reset();}
          publish({}, "reset");
        }
        odom_ = m; dirty_ = true;
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped & m) {
        goal_.reset();
        const auto & p = m.pose.position;
        if (m.header.frame_id != "map" || !std::isfinite(p.x) || !std::isfinite(p.y) ||
          !std::isfinite(p.z) || std::abs(p.z) > 1e-9) {publish({}, "invalid_goal_frame_or_position"); return;}
        goal_ = Eigen::Vector2d(p.x, p.y); new_goal_ = true; dirty_ = true;
        // Yaw is intentionally unused: chapter 11 plans only the centre position.
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] {plan();});
    publish({}, "waiting_goal");
  }

private:
  void receiveMap(const nav_msgs::msg::OccupancyGrid & m)
  {
    try {
      const auto & o = m.info.origin;
      if (m.header.frame_id != "map" || !std::isfinite(o.position.z) ||
        std::abs(o.position.z) > 1e-9 || !std::isfinite(o.orientation.w) ||
        !std::isfinite(o.orientation.x) || !std::isfinite(o.orientation.y) ||
        !std::isfinite(o.orientation.z) || std::abs(o.orientation.x) > 1e-9 ||
        std::abs(o.orientation.y) > 1e-9 || std::abs(o.orientation.z) > 1e-9 ||
        std::abs(std::abs(o.orientation.w)-1) > 1e-9 ||
        m.info.width > 4000000 || m.info.height > 4000000) {
        throw std::invalid_argument("Only finite, axis-aligned map grids are supported");
      }
      GridConfig g; g.resolution = m.info.resolution;
      g.width = static_cast<int>(m.info.width); g.height = static_cast<int>(m.info.height);
      g.origin = {o.position.x, o.position.y};
      auto next = inflateGrid(g, m.data, config_);
      if (received_map_ && rclcpp::Time(m.header.stamp) < map_stamp_) {
        goal_.reset(); last_start_.reset(); publish({}, "reset");
      }
      received_map_ = true; map_stamp_ = rclcpp::Time(m.header.stamp);
      grid_ = std::move(next); map_changed_ = true; dirty_ = true;
      nav_msgs::msg::OccupancyGrid mask; mask.header = m.header; mask.info = m.info;
      mask.data.reserve(grid_->blocked.size());
      for (const auto b : grid_->blocked) {mask.data.push_back(b ? 100 : 0);}
      grid_pub_->publish(mask);
    } catch (const std::invalid_argument & e) {
      grid_.reset(); publish({}, "invalid_map"); RCLCPP_WARN(get_logger(), "%s", e.what());
    }
  }

  void plan()
  {
    if (!dirty_) {return;} dirty_ = false;
    if (!goal_) {return;}
    if (!grid_) {publish({}, "waiting_map"); return;}
    if (!odom_) {publish({}, "waiting_odometry"); return;}
    Pose2D start;
    try {
      const auto tf = buffer_.lookupTransform("map", "odom", rclcpp::Time(odom_->header.stamp));
      // Reuse the planar quaternion validation at the ROS boundary.
      nav_msgs::msg::Odometry alignment; alignment.header.frame_id = "odom";
      alignment.child_frame_id = "base_link";
      alignment.pose.pose.position.x = tf.transform.translation.x;
      alignment.pose.pose.position.y = tf.transform.translation.y;
      alignment.pose.pose.position.z = tf.transform.translation.z;
      alignment.pose.pose.orientation = tf.transform.rotation;
      start = compose(poseFromOdometry(alignment), poseFromOdometry(*odom_));
    } catch (const tf2::TransformException &) {
      dirty_ = true; publish({}, "waiting_transform"); return;
    } catch (const std::invalid_argument &) {
      publish({}, "invalid_transform"); return;
    }
    if (!new_goal_ && !map_changed_ && last_start_ && (start.position-*last_start_).norm() < .05) {return;}
    new_goal_ = false; map_changed_ = false; last_start_ = start.position;
    // planner_snapshot_begin
    const auto result = astar(*grid_, start.position, *goal_, diagonal_);
    if (!result.success) {publish({}, result.status); return;}
    const auto path = simplifyPath(*grid_, result.path);
    publish(path, "found");
    // planner_snapshot_end
  }

  void publish(const std::vector<Eigen::Vector2d> & points, const std::string & status)
  {
    nav_msgs::msg::Path path; path.header.frame_id = "map";
    path.header.stamp = odom_ ? odom_->header.stamp : static_cast<builtin_interfaces::msg::Time>(now());
    for (const auto & p : points) {
      geometry_msgs::msg::PoseStamped pose; pose.header = path.header;
      pose.pose.position.x = p.x(); pose.pose.position.y = p.y(); pose.pose.orientation.w = 1;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
    std_msgs::msg::String message; message.data = status; status_pub_->publish(message);
  }

  InflationConfig config_;
  bool diagonal_{true}, dirty_{false}, new_goal_{false}, map_changed_{false}, received_map_{false};
  rclcpp::Time map_stamp_{0, 0, RCL_ROS_TIME};
  std::optional<PlanningGrid> grid_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::optional<Eigen::Vector2d> goal_, last_start_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace motion2d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<motion2d::PlannerNode>());}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("planner"), "%s", e.what()); return 1;}
  rclcpp::shutdown(); return 0;
}
