#include <cmath>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include "motion2d/planning/astar.hpp"
#include "motion2d/trajectory/trajectory_optimizer.hpp"
#include "motion2d/trajectory/bezier_bounds.hpp"
#include "motion2d/ros/trajectory_messages.hpp"
#include "motion2d/ros/mapping_messages.hpp"
#include "motion2d/ros/planning_messages.hpp"

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
    corridor_enabled_ = declare_parameter("corridor.enabled", false);
    merge_convex_ = declare_parameter("corridor.merge_convex", true);
    extension_ = declare_parameter("corridor.max_extension", .6);
    if (!std::isfinite(extension_) || extension_ < 0) {throw std::invalid_argument("invalid corridor extension");}
    optimize_enabled_=declare_parameter("optimization.enabled",false);
    backend_=declare_parameter("optimization.backend",std::string("minco"));
    certify_=declare_parameter("optimization.publish_certified",false);
    optimization_.bezier_penalties=declare_parameter("optimization.bezier_penalties",false);
    if(backend_!="minco" && backend_!="spline") throw std::invalid_argument("Expected minco or spline backend");
    optimization_.cost.corridor_weight=declare_parameter("optimization.corridor_weight",500.);
    optimization_.cost.corridor_margin=declare_parameter("optimization.corridor_margin",.02);
    optimization_.solver.max_iterations=declare_parameter("optimization.max_iterations",200);
    optimization_.solver.max_wall_seconds=declare_parameter("optimization.max_wall_seconds",.2);
    optimization_.limits.clearance=config_.radius+config_.margin;
    validateCostConfig(optimization_.cost);
    if(optimization_.solver.max_iterations<1 || !std::isfinite(optimization_.solver.max_wall_seconds) ||
      optimization_.solver.max_wall_seconds<0) throw std::invalid_argument("Invalid optimization budget");
    diagonal_ = declare_parameter("planning.diagonal", true);
    // Validate startup configuration even before the first map arrives.
    inflateGrid(GridConfig{}, std::vector<int8_t>(220*220, -1), config_);
    const auto retained = rclcpp::QoS(1).transient_local();
    certified_pub_=create_publisher<motion2d_interfaces::msg::Trajectory2D>("/plan/certified_candidate",retained);
    optimized_pub_=create_publisher<nav_msgs::msg::Path>("/plan/optimized_preview",retained);
    optimized_status_pub_=create_publisher<std_msgs::msg::String>("/plan/optimization_status",retained);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/plan/path", retained);
    raw_pub_ = create_publisher<nav_msgs::msg::Path>("/plan/path_raw", retained);
    corridor_path_pub_ = create_publisher<nav_msgs::msg::Path>("/plan/corridor_path", retained);
    corridor_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/plan/corridors", retained);
    corridor_status_pub_ = create_publisher<std_msgs::msg::String>("/plan/corridor_status", retained);
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
      const auto g = geometryFromOccupancyGrid(m);
      auto next = inflateGrid(g, m.data, config_);
      if (received_map_ && rclcpp::Time(m.header.stamp) < map_stamp_) {
        goal_.reset(); last_start_.reset(); publish({}, "reset");
      }
      received_map_ = true; map_stamp_ = rclcpp::Time(m.header.stamp);
      if(optimize_enabled_) field_.emplace(g,m.data,config_.free_threshold,config_.unknown_blocked);
      grid_ = std::move(next); map_changed_ = true; dirty_ = true;
      nav_msgs::msg::OccupancyGrid mask; mask.header = m.header; mask.info = m.info;
      mask.data.reserve(grid_->blocked.size());
      for (const auto b : grid_->blocked) {mask.data.push_back(b ? 100 : 0);}
      grid_pub_->publish(mask);
    } catch (const std::invalid_argument & e) {
      grid_.reset(); field_.reset(); publish({}, "invalid_map"); RCLCPP_WARN(get_logger(), "%s", e.what());
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
      map_from_odom_=poseFromOdometry(alignment);
      start = compose(map_from_odom_, poseFromOdometry(*odom_));
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
    publish(path, "found", result.path);
    // planner_snapshot_end
  }

  void publish(const std::vector<Eigen::Vector2d> & points, const std::string & status,
    const std::vector<Eigen::Vector2d> & raw = {})
  {
    std_msgs::msg::Header header; header.frame_id = "map";
    header.stamp = odom_ ? odom_->header.stamp : static_cast<builtin_interfaces::msg::Time>(now());
    path_pub_->publish(toPath(points,header)); raw_pub_->publish(toPath(raw,header));
    std_msgs::msg::String message; message.data = status; status_pub_->publish(message);
    // corridor_snapshot_begin
    CorridorResult corridor; corridor.status = corridor_enabled_ ? status : "disabled";
    if (corridor_enabled_ && status == "found") {
      corridor = buildCorridor(*grid_,raw,merge_convex_,extension_);
    }
    corridor_pub_->publish(toCorridorMarkers(corridor.regions,header));
    corridor_path_pub_->publish(toPath(corridor.waypoints,header));
    message.data = corridor.status; corridor_status_pub_->publish(message);
    // corridor_snapshot_end
    // The corridor and ESDF below come from this same map callback snapshot.
    motion2d_interfaces::msg::Trajectory2D candidate;
    candidate.header.frame_id="odom";candidate.header.stamp=header.stamp;
    std::vector<Eigen::Vector2d> optimized;
    std::string diagnostic=optimize_enabled_ ? "waiting_corridor" : "disabled";
    if(optimize_enabled_ && corridor.success && corridor.waypoints.size()>=2 && field_) {
      try {
        TranslationState start,finish;start.position=corridor.waypoints.front();finish.position=corridor.waypoints.back();
        std::vector<Eigen::Vector2d> q(corridor.waypoints.begin()+1,corridor.waypoints.end()-1);
        std::vector<double> times;
        for(std::size_t i=1;i<corridor.waypoints.size();++i)
          times.push_back(std::max(.5,(corridor.waypoints[i]-corridor.waypoints[i-1]).norm()/.5));
        const auto solution=(backend_=="spline" ? optimizeSpline : optimizeMinco)(start,finish,q,times,optimization_,corridor.regions,&*field_);
        diagnostic=solution.solver.status+";"+solution.samples.status+";preview_only";
        if(solution.curve) {
          const int n=std::max(1,int(std::ceil(solution.curve->duration()/.05)));
          for(int j=0;j<=n;++j) optimized.push_back(solution.curve->sample((double(j)/n)*solution.curve->duration()).position);
          if(certify_) {
            const auto certificate=certifyBezier(*solution.curve,corridor.regions,optimization_.limits);
            diagnostic+=certificate.certified ? ";continuous_certificate_passed" : ";continuous_certificate_failed";
            if(certificate.certified && solution.solver.converged()) {
              TimedTrajectory frozen{transformTrajectory(*solution.curve,inverse(map_from_odom_)),0,poseFromOdometry(*odom_).yaw};
              candidate=toTrajectoryMessage(frozen,header.stamp);
            }
          }
          diagnostic+=";corridor_residual="+std::to_string(solution.samples.max_corridor_residual);
        }
      } catch(const std::exception & e) {diagnostic=std::string("optimization_failed:")+e.what();}
    }
    certified_pub_->publish(candidate);
    optimized_pub_->publish(toPath(optimized,header));
    message.data=diagnostic;optimized_status_pub_->publish(message);
  }

  InflationConfig config_;
  bool optimize_enabled_,certify_;
  std::string backend_;
  Pose2D map_from_odom_;
  rclcpp::Publisher<motion2d_interfaces::msg::Trajectory2D>::SharedPtr certified_pub_;
  TrajectoryOptimizationConfig optimization_;
  std::optional<Esdf2D> field_;
  bool corridor_enabled_, merge_convex_;
  double extension_;
  bool diagonal_{true}, dirty_{false}, new_goal_{false}, map_changed_{false}, received_map_{false};
  rclcpp::Time map_stamp_{0, 0, RCL_ROS_TIME};
  std::optional<PlanningGrid> grid_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::optional<Eigen::Vector2d> goal_, last_start_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr optimized_status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, raw_pub_, corridor_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr corridor_status_pub_;
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
