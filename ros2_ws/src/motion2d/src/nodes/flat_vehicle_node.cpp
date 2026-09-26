#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <motion2d_interfaces/msg/ackermann_command.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include "motion2d/control/ackermann_tracker.hpp"
#include "motion2d/control/pd_tracker.hpp"
#include "motion2d/ros/ackermann_parameters.hpp"
#include "motion2d/ros/ackermann_messages.hpp"
#include "motion2d/ros/control_messages.hpp"
#include "motion2d/ros/mapping_messages.hpp"
#include "motion2d/ros/planning_messages.hpp"
#include "motion2d/ros/trajectory_messages.hpp"
#include "motion2d/trajectory/trajectory_optimizer.hpp"
#include "motion2d/trajectory/bezier_bounds.hpp"
#include "motion2d/planning/astar.hpp"

namespace motion2d {
/** @brief Rest-to-rest observed-map experiment for either physical model.
 * @details Goal solving is synchronous only while stopped. A fresh odometry
 * callback after solving assigns a future start. Control uses selected odometry
 * and an explicit ideal steering encoder, never ground_truth or sim pose topics.
 * This deliberately small coordinator is not the moving-handover ch19 navigator.
 */
class FlatVehicleNode:public rclcpp::Node {
public:
  FlatVehicleNode():Node("flat_vehicle"),buffer_(get_clock()),listener_(buffer_,*this,false) {
    model_=declare_parameter("vehicle.model",std::string("ackermann"));
    backend_=declare_parameter("planning.backend",std::string("spline"));
    if(model_!="ackermann" && model_!="inertial")throw std::invalid_argument("vehicle.model must be inertial or ackermann");
    if(backend_!="minco" && backend_!="spline")throw std::invalid_argument("planning.backend must be minco or spline");
    inflation_.radius=declare_parameter("radius",.4);inflation_.margin=declare_parameter("tracking_margin",.15);
    tolerance_=declare_parameter("goal_tolerance",.08);rate_=declare_parameter("control.rate_hz",50.);
    budget_=declare_parameter("planning.solve_budget",1.);
    if(rate_<5 || rate_>100 || !std::isfinite(rate_) || !std::isfinite(tolerance_) || tolerance_<=0 ||
      !std::isfinite(budget_) || budget_<=0 || budget_>5 || inflation_.margin<=0)throw std::invalid_argument("Invalid flat vehicle timing or margin");
    inflateGrid(GridConfig{},std::vector<std::int8_t>(220*220,-1),inflation_);
    if(model_=="ackermann") {
      car_=readAckermannParameters(*this);ack_config_.limits.vehicle=car_;
      auto & l=ack_config_.limits;
      l.speed=declare_parameter("planning.speed_max",.7);l.force=declare_parameter("planning.force_max",1.);
      l.steering=declare_parameter("planning.steering_max",.5);l.steering_rate=declare_parameter("planning.steering_rate_max",.7);
      l.lateral_acceleration=declare_parameter("planning.lateral_acceleration_max",.8);validateAckermannLimits(l);
      ack_config_.solver.max_iterations=400;ack_config_.solver.gradient_tolerance=1e-5;ack_config_.solver.max_wall_seconds=budget_;
      ack_gains_.position=declare_parameter("control.position_gain",2.);ack_gains_.speed=declare_parameter("control.speed_gain",3.);
      ack_gains_.yaw=declare_parameter("control.yaw_gain",2.);ack_gains_.lateral=declare_parameter("control.lateral_gain",1.5);
      ack_gains_.steering=declare_parameter("control.steering_gain",6.);validateAckermannGains(ack_gains_);
    }else {
      omni_.mass=declare_parameter("mass",1.);omni_.linear_drag=declare_parameter("linear_drag",.15);
      omni_.inertia_z=declare_parameter("inertia_z",.08);omni_.angular_drag=declare_parameter("angular_drag",.01);
      omni_.force_max=declare_parameter("force_max",2.);omni_.torque_max=declare_parameter("torque_max",.2);validateInertialParameters(omni_);
      OmniDynamicLimits d;d.parameters=omni_;d.force=declare_parameter("planning.force_max",1.);
      d.force_rate=declare_parameter("planning.force_rate_max",2.);
      if(d.force>omni_.force_max)throw std::invalid_argument("Planning force norm must fit plant axis limits");
      omni_config_.limits.dynamics=d;d.force*=.85;d.force_rate*=.85;omni_config_.cost.dynamics=d;
      omni_config_.limits.speed=declare_parameter("planning.speed_max",.7);omni_config_.cost.speed_max=.85*omni_config_.limits.speed;
      omni_config_.cost.acceleration_max=.6;omni_config_.cost.energy_weight=.01;omni_config_.bezier_penalties=true;
      omni_config_.solver.max_iterations=400;omni_config_.solver.max_wall_seconds=budget_;omni_config_.solver.gradient_tolerance=1e-5;
      validateCostConfig(omni_config_.cost);
    }
    const auto retained=rclcpp::QoS(1).transient_local();
    status_pub_=create_publisher<std_msgs::msg::String>("/flat/status",retained);
    plan_status_pub_=create_publisher<std_msgs::msg::String>("/flat/plan_status",retained);
    reference_pub_=create_publisher<nav_msgs::msg::Odometry>("/flat/reference",10);
    path_pub_=create_publisher<nav_msgs::msg::Path>("/plan/path",retained);
    corridor_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("/plan/corridors",retained);
    ack_curve_pub_=create_publisher<motion2d_interfaces::msg::AckermannTrajectory2D>("/flat/ackermann_trajectory",retained);
    omni_curve_pub_=create_publisher<motion2d_interfaces::msg::Trajectory2D>("/flat/inertial_trajectory",retained);
    if(model_=="ackermann")ack_pub_=create_publisher<motion2d_interfaces::msg::AckermannCommand>("/command/ackermann",1);
    else wrench_pub_=create_publisher<geometry_msgs::msg::WrenchStamped>("/command/wrench",1);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odometry",1,[this](const nav_msgs::msg::Odometry & m) {
      try {
        const auto x=stateFromOdometry(m);const auto ns=rclcpp::Time(m.header.stamp).nanoseconds();
        if(odom_ns_ && ns<*odom_ns_) {stop("clock_reset");goal_.reset();grid_.reset();scan_ns_.reset();joint_ns_.reset();last_control_=-1;}
        state_=x;odom_ns_=ns;
        if(pending_start_ && ns>solved_at_ns_) {start_ns_=ns+300000000;pending_start_=false;publishCurve(m.header.stamp);status("waiting_start");}
      }catch(const std::exception &){stop("invalid_odometry");odom_ns_.reset();}
    });
    joint_sub_=create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,[this](const sensor_msgs::msg::JointState & m) {
      if(m.name.size()!=1 || m.name[0]!="front_steering" || m.position.size()!=1 || !std::isfinite(m.position[0]) ||
        std::abs(m.position[0])>car_.steering_max+1e-8) {joint_ns_.reset();return;}
      steering_=m.position[0];joint_ns_=rclcpp::Time(m.header.stamp).nanoseconds();
    });
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS(),[this](const sensor_msgs::msg::LaserScan & m){scan_ns_=rclcpp::Time(m.header.stamp).nanoseconds();});
    map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",retained,[this](const nav_msgs::msg::OccupancyGrid & m){receiveMap(m);});
    goal_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",10,[this](const geometry_msgs::msg::PoseStamped & m) {
      stop("new_goal_braking");goal_.reset();wait_map_revision_.reset();
      try {
        if(m.header.frame_id!="map")throw std::invalid_argument("Goal must be in map");
        nav_msgs::msg::Odometry q;q.header.frame_id="odom";q.child_frame_id="base_link";q.pose.pose=m.pose;
        goal_=poseFromOdometry(q);status("waiting_observations");
      }catch(const std::exception &){status("invalid_goal");}
    });
    stop_sub_=create_subscription<std_msgs::msg::Empty>("/flat/stop",1,[this](const std_msgs::msg::Empty &){goal_.reset();stop("requested_stop");});
    timer_=create_wall_timer(std::chrono::milliseconds(10),[this]{tick();});status("waiting_goal");
  }
private:
  void status(const std::string & s) {if(s==status_)return;status_=s;std_msgs::msg::String m;m.data=s;status_pub_->publish(m);}
  std_msgs::msg::Header header(const std::string & frame="odom") const {
    std_msgs::msg::Header h;h.frame_id=frame;h.stamp=rclcpp::Time(odom_ns_.value_or(0),RCL_ROS_TIME);return h;
  }
  Pose2D alignment() {
    const auto tf=buffer_.lookupTransform("map","odom",tf2::TimePointZero);
    nav_msgs::msg::Odometry m;m.header.frame_id="odom";m.child_frame_id="base_link";
    m.pose.pose.position.x=tf.transform.translation.x;m.pose.pose.position.y=tf.transform.translation.y;
    m.pose.pose.position.z=tf.transform.translation.z;m.pose.pose.orientation=tf.transform.rotation;return poseFromOdometry(m);
  }
  void stop(const std::string & reason) {
    ack_curve_.reset();omni_curve_.reset();regions_odom_.clear();pending_start_=false;
    ack_curve_pub_->publish(motion2d_interfaces::msg::AckermannTrajectory2D{});
    omni_curve_pub_->publish(motion2d_interfaces::msg::Trajectory2D{});
    path_pub_->publish(toPath({},header()));corridor_pub_->publish(toCorridorMarkers({},header()));status(reason);
  }
  void receiveMap(const nav_msgs::msg::OccupancyGrid & m) {
    try {
      if(m.header.frame_id!="map")throw std::invalid_argument("Map frame required");
      const auto stamp=rclcpp::Time(m.header.stamp).nanoseconds();
      if(map_ns_ && stamp<*map_ns_){stop("map_reset");goal_.reset();}
      grid_=inflateGrid(geometryFromOccupancyGrid(m),m.data,inflation_);map_ns_=stamp;
      if(!regions_odom_.empty() && odom_ns_) {
        const auto tf=alignment();
        for(const auto & r:regions_odom_) {
          Polygon polygon;for(const auto & v:r.polygon.vertices)polygon.vertices.push_back(transformPoint(tf,v));
          if(!regionIsFree(*grid_,makeRegion(polygon))){stop("map_invalidated_corridor");goal_.reset();break;}
        }
      }
    }catch(const tf2::TransformException &){stop("map_transform_unavailable");goal_.reset();}
    catch(const std::exception &){grid_.reset();stop("invalid_map");goal_.reset();}
  }
  void publishCurve(const builtin_interfaces::msg::Time & stamp) {
    if(ack_curve_)ack_curve_pub_->publish(toAckermannMessage({*ack_curve_,start_ns_},stamp));
    if(omni_curve_)omni_curve_pub_->publish(toTrajectoryMessage({*omni_curve_,start_ns_,fixed_yaw_},stamp));
  }
  void solve() {
    if(!buffer_.canTransform("map","odom",tf2::TimePointZero)) {status("waiting_transform");return;}
    const auto goal=*goal_;goal_.reset();status("planning_at_rest");
    try {
      const auto map_from_odom=alignment(),odom_from_map=inverse(map_from_odom);
      const auto start=compose(map_from_odom,state_.pose);
      const auto route=astar(*grid_,start.position,goal.position);
      if(!route.success) {
        if(route.status=="invalid_start") {
          goal_=goal;wait_map_revision_=map_ns_;status("waiting_observed_free_start");
        }else status("plan_failed:"+route.status);
        return;
      }
      wait_map_revision_.reset();
      const auto corridor=buildCorridor(*grid_,route.path,true,.8);
      if(!corridor.success || corridor.waypoints.size()<2 || corridor.regions.size()>20){status("plan_failed:unusable_corridor");return;}
      const auto & points=corridor.waypoints;std::vector<double> times;
      for(std::size_t i=1;i<points.size();++i)times.push_back(std::max(2.,(points[i]-points[i-1]).norm()/.3));
      std::string detail;
      if(model_=="ackermann") {
        std::vector<Pose2D> knots;
        for(std::size_t i=0;i<points.size();++i) {
          Eigen::Vector2d direction;
          if(i==0)direction=points[1]-points[0];
          else if(i+1==points.size())direction=points[i]-points[i-1];
          else direction=(points[i]-points[i-1]).normalized()+(points[i+1]-points[i]).normalized();
          const double yaw=i==0?start.yaw:i+1==points.size()?goal.yaw:std::atan2(direction.y(),direction.x());
          knots.push_back({points[i],yaw});
        }
        const auto plan=planAckermann(knots,times,corridor.regions,ack_config_);
        if(!plan.curve){status("plan_failed:"+plan.status);return;}
        ack_curve_=transformAckermann(*plan.curve,odom_from_map);
        detail=plan.status+";retiming="+std::to_string(plan.retiming_steps)+";force_bound="+std::to_string(plan.certificate.force);
      }else {
        TranslationState a,b;a.position=points.front();b.position=points.back();
        const std::vector<Eigen::Vector2d> inside(points.begin()+1,points.end()-1);
        const auto plan=(backend_=="spline"?optimizeSpline:optimizeMinco)(a,b,inside,times,omni_config_,corridor.regions,nullptr);
        if(!plan.curve || !plan.solver.converged()){status("plan_failed:"+plan.solver.status);return;}
        const auto cert=certifyBezier(*plan.curve,corridor.regions,omni_config_.limits);
        if(!cert.certified){status("plan_failed:continuous_bounds");return;}
        omni_curve_=transformTrajectory(*plan.curve,odom_from_map);fixed_yaw_=state_.pose.yaw;
        detail="certified;force_bound="+std::to_string(cert.force_bound)+";force_rate_bound="+std::to_string(cert.force_rate_bound);
      }
      regions_odom_.clear();
      for(const auto & r:corridor.regions) {Polygon q;for(const auto & v:r.polygon.vertices)q.vertices.push_back(transformPoint(odom_from_map,v));regions_odom_.push_back(makeRegion(q));}
      duration_=ack_curve_?ack_curve_->duration():omni_curve_->duration();pending_start_=true;solved_at_ns_=*odom_ns_;
      std_msgs::msg::String m;m.data="model="+model_+";"+detail+";duration="+std::to_string(duration_);plan_status_pub_->publish(m);
      std::vector<Eigen::Vector2d> preview;
      // Form the unit fraction first: (duration * count) / count can exceed
      // duration by one ULP, violating the polynomial's closed time interval.
      for(int k=0;k<=200;++k) {const double t=(double(k)/200.)*duration_;preview.push_back(ack_curve_?ack_curve_->sample(t,car_).state.pose.position:omni_curve_->sample(t).position);}
      path_pub_->publish(toPath(preview,header()));corridor_pub_->publish(toCorridorMarkers(regions_odom_,header()));status("waiting_fresh_odometry");
    }catch(const tf2::TransformException &){goal_=goal;status("waiting_transform");}
    catch(const std::exception & e){stop(std::string("plan_failed:")+e.what());}
  }
  void command(const AckermannInput & u) {
    motion2d_interfaces::msg::AckermannCommand m;m.header=header("base_link");m.header.stamp=now();m.force=u.force;m.steering_rate=u.steering_rate;ack_pub_->publish(m);
  }
  void command(const Wrench2D & u) {
    geometry_msgs::msg::WrenchStamped m;m.header=header();m.header.stamp=now();m.wrench.force.x=u.force.x();m.wrench.force.y=u.force.y();m.wrench.torque.z=u.torque;wrench_pub_->publish(m);
  }
  void brake() {
    if(model_=="ackermann") {
      const double speed=(rotation(state_.pose.yaw).transpose()*state_.velocity).x();
      command(limitAckermannInput({state_.pose,speed,steering_},{-3*car_.mass*speed,-4*steering_},car_,1/rate_));
    }else command(dampingBrake(state_,omni_,3));
  }
  void tick() {
    if(!odom_ns_)return;
    const auto clock_now=now().nanoseconds();
    if(clock_now-*odom_ns_>120000000) {
      stop("stale_odometry_zero_input");goal_.reset();
      if(model_=="ackermann")command(AckermannInput{});else command(Wrench2D{});
      return; // Without fresh velocity, repeated old-state braking could reverse the vehicle.
    }
    if(last_control_>=0 && *odom_ns_-last_control_<std::llround(1e9/rate_))return;
    last_control_=*odom_ns_;
    const bool sensors=scan_ns_ && *odom_ns_-*scan_ns_<=400000000 && map_ns_ && *odom_ns_-*map_ns_<=3000000000LL &&
      (model_!="ackermann" || (joint_ns_ && std::abs(*odom_ns_-*joint_ns_)<=120000000));
    if(!sensors) {if(ack_curve_ || omni_curve_)stop("stale_observation_braking");brake();return;}
    if(goal_ && grid_ && (!wait_map_revision_ || wait_map_revision_!=map_ns_) && state_.velocity.norm()<.01 && std::abs(state_.yaw_rate)<.01 &&
      (model_!="ackermann" || std::abs(steering_)<.01)) {brake();solve();return;}
    if(pending_start_ || (!ack_curve_ && !omni_curve_)){brake();return;}
    const double t=(*odom_ns_-start_ns_)*1e-9;State2D reference;AckermannReference ack_reference;
    if(ack_curve_){ack_reference=ack_curve_->sample(t,car_);reference=ack_reference.state;}
    else {const auto s=omni_curve_->sample(std::clamp(t,0.,duration_));reference.pose={s.position,fixed_yaw_};reference.velocity=s.velocity;reference.acceleration=s.acceleration;}
    const double error=(reference.pose.position-state_.pose.position).norm();
    if(error>inflation_.margin*.8){stop("tracking_margin_exceeded");brake();return;}
    if(t>duration_ && error<tolerance_ && state_.velocity.norm()<.03 &&
      (model_!="ackermann" || std::abs(wrapAngle(reference.pose.yaw-state_.pose.yaw))<.1)) {
      stop("completed");brake();return;
    }
    if(t>duration_+5){stop("tracking_did_not_settle");brake();return;}
    reference_pub_->publish(referenceOdometry(reference,header().stamp));
    if(model_=="ackermann") {
      const double speed=(rotation(state_.pose.yaw).transpose()*state_.velocity).x();
      command(trackAckermann({state_.pose,speed,steering_},ack_reference,car_,ack_gains_,1/rate_));
    }else command(trackPd(state_,reference,{},omni_).applied);
    status(t<0?"waiting_start":"executing");
  }
  std::string model_,backend_,status_;double rate_,tolerance_,budget_,steering_=0,duration_=0,fixed_yaw_=0;
  InflationConfig inflation_;InertialParameters omni_;AckermannParameters car_;AckermannGains ack_gains_;
  TrajectoryOptimizationConfig omni_config_;AckermannPlanningConfig ack_config_;
  State2D state_;std::optional<Pose2D> goal_;std::optional<PlanningGrid> grid_;
  std::optional<PolynomialTrajectory> omni_curve_;std::optional<AckermannTrajectory> ack_curve_;
  std::vector<ConvexRegion> regions_odom_;
  std::optional<std::int64_t> odom_ns_,scan_ns_,map_ns_,joint_ns_,wait_map_revision_;
  std::int64_t start_ns_=0,solved_at_ns_=0,last_control_=-1;bool pending_start_=false;
  tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_,plan_status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr reference_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_pub_;
  rclcpp::Publisher<motion2d_interfaces::msg::AckermannTrajectory2D>::SharedPtr ack_curve_pub_;
  rclcpp::Publisher<motion2d_interfaces::msg::Trajectory2D>::SharedPtr omni_curve_pub_;
  rclcpp::Publisher<motion2d_interfaces::msg::AckermannCommand>::SharedPtr ack_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<motion2d::FlatVehicleNode>());}
  catch(const std::exception & e){RCLCPP_ERROR(rclcpp::get_logger("flat_vehicle"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
