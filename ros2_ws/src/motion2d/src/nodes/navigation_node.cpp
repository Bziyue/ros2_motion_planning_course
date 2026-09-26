#include <rclcpp/rclcpp.hpp>
#include "motion2d/ros/dynamics_parameters.hpp"
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <motion2d_interfaces/msg/navigation_status.hpp>
#include "motion2d/planning/replanner.hpp"
#include "motion2d/ros/navigation_messages.hpp"
#include "motion2d/ros/control_messages.hpp"
#include "motion2d/ros/mapping_messages.hpp"
#include "motion2d/ros/planning_messages.hpp"

namespace motion2d {
/** @brief Chapter 19 single-threaded navigation coordinator, observed map only.
 * @details Map inflation/ESDF share one immutable snapshot per solve. The selected
 * odometry drives cadence; scan stamps detect sensor loss even if an EKF predicts.
 * A reference is mirrored locally only after the controller acknowledges its start.
 */
class NavigationNode : public rclcpp::Node {
public:
  NavigationNode():Node("navigation"),buffer_(get_clock()),listener_(buffer_,*this,false) {
    inflation_.radius=declare_parameter("radius",.2);inflation_.margin=declare_parameter("navigation.tracking_margin",.15);
    rate_=declare_parameter("navigation.rate_hz",5.);lead_=declare_parameter("navigation.lead_seconds",.15);
    scan_timeout_=declare_parameter("navigation.scan_timeout",.35);map_timeout_=declare_parameter("navigation.map_timeout",3.);
    goal_tolerance_=declare_parameter("navigation.goal_tolerance",.12);
    config_.max_route_length=declare_parameter("navigation.local_length",2.);
    config_.optimization.bezier_penalties=declare_parameter("navigation.bezier_penalties",false);
    config_.optimization.solver.max_wall_seconds=declare_parameter("navigation.solve_budget",.04);
    readPlanningDynamics(*this,config_.optimization);
    for(double x:{rate_,lead_,scan_timeout_,map_timeout_,goal_tolerance_})
      if(!std::isfinite(x) || x<=0) throw std::invalid_argument("Invalid navigation timing/tolerance");
    if(rate_>50 || lead_>.5 || config_.optimization.solver.max_wall_seconds<=0 ||
       config_.optimization.solver.max_wall_seconds>=lead_) throw std::invalid_argument("Need positive solve budget shorter than handover lead");
    inflateGrid(GridConfig{},std::vector<std::int8_t>(220*220,-1),inflation_);
    period_=std::llround(1e9/rate_);lead_ns_=std::llround(1e9*lead_);
    const auto retained=rclcpp::QoS(1).transient_local();
    reference_pub_=create_publisher<motion2d_interfaces::msg::NavigationReference>("/navigation/reference",1);
    stop_pub_=create_publisher<std_msgs::msg::Empty>("/navigation/stop",1);
    status_pub_=create_publisher<motion2d_interfaces::msg::NavigationStatus>("/navigation/status",rclcpp::QoS(100).transient_local());
    path_pub_=create_publisher<nav_msgs::msg::Path>("/plan/path",retained);
    corridor_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("/plan/corridors",retained);
    grid_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("/planning/grid",retained);
    map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",retained,[this](const nav_msgs::msg::OccupancyGrid & m){receiveMap(m);});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odometry",100,[this](const nav_msgs::msg::Odometry & m){
      try {
        auto state=stateFromOdometry(m);const auto ns=rclcpp::Time(m.header.stamp).nanoseconds();
        if(odom_ns_ && ns<*odom_ns_) {stop("clock_reset");goal_.reset();snapshot_.reset();scan_ns_.reset();last_attempt_.reset();}
        state_=state;odom_ns_=ns;schedule_.advance(ns);
      } catch(const std::exception &){stop("invalid_odometry");}
    });
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS(),[this](const sensor_msgs::msg::LaserScan & m){scan_ns_=rclcpp::Time(m.header.stamp).nanoseconds();});
    goal_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",10,[this](const geometry_msgs::msg::PoseStamped & m){
      stop("new_goal");goal_.reset();last_attempt_.reset();
      const auto & p=m.pose.position;
      if(m.header.frame_id!="map" || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || std::abs(p.z)>1e-9) {status("invalid_goal");return;}
      goal_=Eigen::Vector2d(p.x,p.y);status("waiting_observations");
    });
    ack_sub_=create_subscription<std_msgs::msg::Int64>("/control/accepted_reference",10,[this](const std_msgs::msg::Int64 & m){
      if(!awaiting_ || awaiting_->motion.start_ns!=m.data) return;
      const auto accepted=schedule_.accept(*awaiting_,odom_ns_.value_or(0));
      if(!accepted.accepted) {stop("late_or_invalid_ack:"+accepted.reason);return;}
      latest_start_=m.data;awaiting_.reset();++accepted_;status("executing");
    });
    control_sub_=create_subscription<std_msgs::msg::String>("/control/status",retained,[this](const std_msgs::msg::String & m){
      if(!goal_) return;
      if(m.data.rfind("mpc_",0)==0 || m.data.rfind("invalid_control_input",0)==0 || m.data=="stale_odometry_braking" ||
         (awaiting_ && m.data.rfind("navigation_reference_rejected:",0)==0)) stop("controller:"+m.data);
    });
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{plan();});status("waiting_goal");
  }
private:
  struct Snapshot {
    std::vector<std::int8_t> raw;PlanningGrid grid;Esdf2D field;std::int64_t stamp;
  };
  std_msgs::msg::Header header(const std::string & frame="odom") const {
    std_msgs::msg::Header out;out.frame_id=frame;out.stamp=rclcpp::Time(odom_ns_.value_or(0),RCL_ROS_TIME);return out;
  }
  void status(const std::string & label) {
    motion2d_interfaces::msg::NavigationStatus m;m.header=header();m.state=label;m.planner_status=planner_status_;
    m.optimization_status=optimization_status_;m.planning_seconds=planning_seconds_;m.accepted_plans=accepted_;m.stop_count=stops_;status_pub_->publish(m);
  }
  void stop(const std::string & reason) {
    const bool had_reference=!schedule_.empty() || awaiting_.has_value();
    schedule_.reset(state_.pose);awaiting_.reset();
    if(had_reference) {++stops_;stop_pub_->publish(std_msgs::msg::Empty{});}
    path_pub_->publish(toPath({},header("map")));corridor_pub_->publish(toCorridorMarkers({},header("map")));status(reason);
  }
  void receiveMap(const nav_msgs::msg::OccupancyGrid & m) {
    try {
      const auto geometry=geometryFromOccupancyGrid(m);const auto stamp=rclcpp::Time(m.header.stamp).nanoseconds();
      if(snapshot_ && stamp<snapshot_->stamp) {stop("map_reset");goal_.reset();last_attempt_.reset();}
      snapshot_.emplace(Snapshot{m.data,inflateGrid(geometry,m.data,inflation_),Esdf2D(geometry,m.data),stamp});
      map_changed_=true;nav_msgs::msg::OccupancyGrid mask=m;mask.data.clear();
      for(auto b:snapshot_->grid.blocked) mask.data.push_back(b ? 100 : 0);
      grid_pub_->publish(mask);
    } catch(const std::exception &){snapshot_.reset();stop("invalid_map");}
  }
  Pose2D alignment() const {
    // Latest held SLAM alignment; no extrapolation to the faster IMU timestamp.
    const auto tf=buffer_.lookupTransform("map","odom",tf2::TimePointZero);
    nav_msgs::msg::Odometry m;m.header.frame_id="odom";m.child_frame_id="base_link";
    m.pose.pose.position.x=tf.transform.translation.x;m.pose.pose.position.y=tf.transform.translation.y;
    m.pose.pose.position.z=tf.transform.translation.z;m.pose.pose.orientation=tf.transform.rotation;return poseFromOdometry(m);
  }
  bool regionsStillFree(const NavigationReference * plan,std::int64_t now,const Pose2D & map_from_odom) const {
    if(!plan) return true;
    const auto mapped=transformReference(*plan,map_from_odom);double end=0;
    for(std::size_t i=0;i<mapped.regions.size();++i) {
      end+=mapped.motion.curve.pieces()[i].duration;
      if((now-mapped.motion.start_ns)*1e-9>end) continue;
      if(!regionIsFree(snapshot_->grid,mapped.regions[i])) return false;
    }
    return true;
  }
  void plan() {
    if(!goal_) return;
    const auto ns=now().nanoseconds();
    if(!odom_ns_ || !scan_ns_ || !snapshot_) {status("waiting_observations");return;}
    if(ns<*odom_ns_ || ns<*scan_ns_) return; // Clock and sensor messages can arrive in either order.
    if((ns-*odom_ns_)*1e-9>.12 || (ns-*scan_ns_)*1e-9>scan_timeout_ || (ns-snapshot_->stamp)*1e-9>map_timeout_) {
      stop("stale_observations_braking");return;
    }
    Pose2D map_from_odom;
    try {map_from_odom=alignment();} catch(const std::exception &){stop("waiting_transform");return;}
    const auto position=compose(map_from_odom,state_.pose).position;
    if((position-*goal_).norm()<goal_tolerance_ && state_.velocity.norm()<.08) {stop("reached");goal_.reset();return;}
    if(map_changed_) {
      map_changed_=false;
      if(!regionsStillFree(schedule_.reference(*odom_ns_),*odom_ns_,map_from_odom) ||
         !regionsStillFree(schedule_.reference(std::max(latest_start_,*odom_ns_)),*odom_ns_,map_from_odom) ||
         (awaiting_ && !regionsStillFree(&*awaiting_,*odom_ns_,map_from_odom))) {stop("changed_map_braking");return;}
    }
    if(awaiting_) {if(ns>=awaiting_->motion.start_ns) stop("ack_timeout_braking");return;}
    if(!replanDue(*odom_ns_,last_attempt_,period_,schedule_.pending())) return;
    if(schedule_.empty()) {
      if(state_.velocity.norm()>.02 || std::abs(state_.yaw_rate)>.02) {status("waiting_rest");return;}
      schedule_.reset(state_.pose);
    }
    last_attempt_=*odom_ns_;const auto start_ns=*odom_ns_+lead_ns_;
    // navigation_snapshot_begin
    const auto old=schedule_.sample(start_ns);const auto R=rotation(map_from_odom.yaw);
    TranslationState boundary{map_from_odom.position+R*old.pose.position,R*old.velocity,R*old.acceleration};
    const auto result=replanObserved(snapshot_->grid,snapshot_->raw,snapshot_->field,boundary,*goal_,config_);
    planner_status_=result.status;optimization_status_=result.optimization_status;planning_seconds_=result.seconds;
    if(!result.success) {stop("planning_failed_braking");return;}
    NavigationReference mapped{{*result.curve,start_ns,wrapAngle(old.pose.yaw+map_from_odom.yaw)},result.regions,snapshot_->stamp};
    awaiting_=transformReference(mapped,inverse(map_from_odom));
    reference_pub_->publish(toNavigationMessage(*awaiting_,header().stamp));
    // navigation_snapshot_end
    path_pub_->publish(toPath(result.path,header("map")));corridor_pub_->publish(toCorridorMarkers(result.regions,header("map")));status("awaiting_ack");
  }
  InflationConfig inflation_;ReplanConfig config_;ReferenceSchedule schedule_;State2D state_;
  std::optional<Snapshot> snapshot_;std::optional<Eigen::Vector2d> goal_;std::optional<NavigationReference> awaiting_;
  std::optional<std::int64_t> odom_ns_,scan_ns_,last_attempt_;std::int64_t period_,lead_ns_,latest_start_=0;
  double rate_,lead_,scan_timeout_,map_timeout_,goal_tolerance_,planning_seconds_=0;bool map_changed_=false;
  std::string planner_status_,optimization_status_;std::uint64_t accepted_=0,stops_=0;
  tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<motion2d_interfaces::msg::NavigationReference>::SharedPtr reference_pub_;
  rclcpp::Publisher<motion2d_interfaces::msg::NavigationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr ack_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
};
}
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<motion2d::NavigationNode>());}
  catch(const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("navigation"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
