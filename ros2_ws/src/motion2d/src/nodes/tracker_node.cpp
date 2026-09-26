#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int64.hpp>
#include "motion2d/ros/navigation_messages.hpp"
#include "motion2d/trajectory/bezier_bounds.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include "motion2d/control/pd_tracker.hpp"
#include "motion2d/control/tracking_reference.hpp"
#include "motion2d/ros/control_messages.hpp"
#include "motion2d/ros/trajectory_messages.hpp"
#include <optional>
#include <memory>
#include "motion2d/control/linear_mpc.hpp"
#include <motion2d_interfaces/msg/mpc_status.hpp>
namespace motion2d {
/** @brief Ch17 force tracker; selected odometry is its sole state observation.
 * @details Simulation-time cadence on unique odometry timestamps; watchdog uses
 * advancing /clock, not elapsed wall time while paused. No truth/TF/obstacle input.
 */
class TrackerNode : public rclcpp::Node {
public:
  TrackerNode():Node("tracker") {
    model_.mass=declare_parameter("control.mass",1.);model_.linear_drag=declare_parameter("control.linear_drag",.15);
    model_.inertia_z=declare_parameter("control.inertia_z",.02);model_.angular_drag=declare_parameter("control.angular_drag",.01);
    model_.force_max=declare_parameter("control.force_max",2.);model_.torque_max=declare_parameter("control.torque_max",.2);
    validateInertialParameters(model_);
    gains_.kp.setConstant(declare_parameter("control.kp",4.));gains_.kd.setConstant(declare_parameter("control.kd",3.));
    gains_.yaw_kp=declare_parameter("control.yaw_kp",4.);gains_.yaw_kd=declare_parameter("control.yaw_kd",3.);
    gains_.feedforward=declare_parameter("control.feedforward",true);validatePdGains(gains_);
    const double rate=declare_parameter("control.rate_hz",50.);timeout_=declare_parameter("control.odometry_timeout",.12);
    if(!std::isfinite(rate) || rate<=0 || rate>1000 || !std::isfinite(timeout_) || timeout_<=0) throw std::invalid_argument("Invalid controller rate or timeout");
    period_=std::llround(1e9/rate);
    controller_=declare_parameter("control.controller",std::string("pd"));
    if(controller_!="pd" && controller_!="mpc" && controller_!="ideal") throw std::invalid_argument("Expected pd, mpc or ideal controller");
    mpc_config_.dt=period_*1e-9;mpc_config_.horizon=declare_parameter("mpc.horizon",20);
    mpc_config_.position_weight=declare_parameter("mpc.position_weight",40.);
    mpc_config_.velocity_weight=declare_parameter("mpc.velocity_weight",2.);
    mpc_config_.force_weight=declare_parameter("mpc.force_weight",.02);
    mpc_config_.increment_weight=declare_parameter("mpc.increment_weight",.005);
    mpc_config_.velocity_max=declare_parameter("mpc.velocity_max",1.);
    mpc_config_.force_rate_max=declare_parameter("mpc.force_rate_max",5.);
    mpc_config_.solver.max_iterations=declare_parameter("mpc.max_iterations",1500);
    mpc_config_.solver.max_wall_seconds=declare_parameter("mpc.max_wall_seconds",.01);
    if(controller_=="mpc") mpc_=std::make_unique<LinearMpc>(model_,mpc_config_);
    mpc_pub_=create_publisher<motion2d_interfaces::msg::MpcStatus>("/control/mpc_status",100);
    prediction_pub_=create_publisher<nav_msgs::msg::Path>("/control/prediction",rclcpp::QoS(1).transient_local());
    source_=declare_parameter("reference.source",std::string("analytic"));
    if(source_!="analytic" && source_!="trajectory" && source_!="navigation") throw std::invalid_argument("Expected analytic, trajectory or navigation reference");
    const auto shape=declare_parameter("reference.shape",std::string("figure_eight"));
    if(shape!="circle" && shape!="figure_eight") throw std::invalid_argument("Expected circle or figure_eight");
    reference_config_.shape=shape=="circle" ? ReferenceShape::Circle : ReferenceShape::FigureEight;
    reference_config_.scale=declare_parameter("reference.scale",1.);reference_config_.omega=declare_parameter("reference.omega",.5);
    reference_config_.ramp_seconds=declare_parameter("reference.ramp_seconds",2.);validateTrackingReference(reference_config_);
    wrench_=create_publisher<geometry_msgs::msg::WrenchStamped>("/command/wrench",1);
    requested_=create_publisher<geometry_msgs::msg::WrenchStamped>("/control/wrench_requested",10);
    reference_pub_=create_publisher<nav_msgs::msg::Odometry>("/control/reference",100);
    acceleration_pub_=create_publisher<geometry_msgs::msg::AccelStamped>("/control/reference_acceleration",100);
    path_pub_=create_publisher<nav_msgs::msg::Path>("/control/reference_path",rclcpp::QoS(1).transient_local());
    status_pub_=create_publisher<std_msgs::msg::String>("/control/status",rclcpp::QoS(1).transient_local());
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odometry",100,[this](const nav_msgs::msg::Odometry & m){receive(m);});
    if(source_=="trajectory") trajectory_sub_=create_subscription<motion2d_interfaces::msg::Trajectory2D>("/plan/trajectory",1,
      [this](const motion2d_interfaces::msg::Trajectory2D & m){receiveTrajectory(m);});
    pose_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("/command/pose",1);
    accepted_pub_=create_publisher<std_msgs::msg::Int64>("/control/accepted_reference",10);
    if(source_=="navigation") {
      navigation_sub_=create_subscription<motion2d_interfaces::msg::NavigationReference>("/navigation/reference",1,
        [this](const motion2d_interfaces::msg::NavigationReference & m){receiveNavigation(m);});
      stop_sub_=create_subscription<std_msgs::msg::Empty>("/navigation/stop",1,
        [this](const std_msgs::msg::Empty &){if(last_stamp_) brake(now(),"navigation_stopped_braking");});
    }
    navigation_limits_.speed=declare_parameter("navigation.reference_speed_max",.7);
    navigation_limits_.acceleration=declare_parameter("navigation.reference_acceleration_max",.8);
    enable_=create_service<std_srvs::srv::SetBool>("/tracker/enable",[this](const std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response){enabled_=request->data;response->success=true;response->message=enabled_ ? "enabled" : "bounded damping brake";
        if(!enabled_ && last_stamp_) brake(now(),"disabled_braking");});
    watchdog_=create_wall_timer(std::chrono::milliseconds(20),[this]{
      if(last_stamp_ && (now().nanoseconds()-*last_stamp_)*1e-9>timeout_)
        brake(now(),"stale_odometry_braking");
    });
    status("waiting_odometry");
  }
private:
  void status(const std::string & value) {
    if(value==last_status_) return;
    last_status_=value;std_msgs::msg::String m;m.data=value;status_pub_->publish(m);
  }
  void send(const Wrench2D & command,const rclcpp::Time & stamp,const std::string & label) {
    geometry_msgs::msg::WrenchStamped m;m.header.frame_id="odom";m.header.stamp=stamp;
    m.wrench.force.x=command.force.x();m.wrench.force.y=command.force.y();m.wrench.torque.z=command.torque;
    wrench_->publish(m);previous_force_=command.force;status(label);
  }
  void sendPose(const Pose2D & target,const rclcpp::Time & stamp,const std::string & label) {
    geometry_msgs::msg::PoseStamped m;m.header.frame_id="odom";m.header.stamp=stamp;
    m.pose.position.x=target.position.x();m.pose.position.y=target.position.y();
    m.pose.orientation.z=std::sin(target.yaw/2);m.pose.orientation.w=std::cos(target.yaw/2);
    pose_pub_->publish(m);previous_force_.setZero();status(label);
  }
  void brake(const rclcpp::Time & stamp,const std::string & label) {
    if(mpc_) mpc_->reset();
    nav_msgs::msg::Path empty;empty.header.frame_id="odom";empty.header.stamp=stamp;prediction_pub_->publish(empty);
    if(source_=="navigation") {schedule_.reset(state_.pose);path_pub_->publish(empty);}
    if(controller_=="ideal") sendPose(state_.pose,stamp,label);
    else send(dampingBrake(state_,model_),stamp,label);
  }
  void publishPrediction(const MpcResult & result,std::int64_t stamp) {
    nav_msgs::msg::Path path;path.header.frame_id="odom";path.header.stamp=rclcpp::Time(stamp,RCL_ROS_TIME);
    if(result.solver.solved()) for(int k=0;k<mpc_config_.horizon;++k) {
      geometry_msgs::msg::PoseStamped p;p.header=path.header;p.header.stamp=rclcpp::Time(stamp+(k+1)*period_,RCL_ROS_TIME);
      p.pose.position.x=result.predicted_states[4*k];p.pose.position.y=result.predicted_states[4*k+1];p.pose.orientation.w=1.;path.poses.push_back(p);
    }
    prediction_pub_->publish(path);
  }
  State2D referenceAt(std::int64_t stamp) const {
    if(source_=="navigation") return schedule_.sample(stamp);
    if(source_=="analytic") return sampleTrackingReference(reference_config_,(stamp-epoch_)*1e-9);
    return sampleHeldTrajectory(*trajectory_,stamp);
  }
  void publishPath(std::int64_t stamp,const TimedTrajectory * supplied=nullptr) {
    const auto * curve=supplied ? supplied : trajectory_ ? &*trajectory_ : nullptr;
    nav_msgs::msg::Path path;path.header.frame_id="odom";path.header.stamp=rclcpp::Time(stamp,RCL_ROS_TIME);
    const double duration=source_=="analytic" ? 24. : curve->curve.duration();
    const auto begin=source_=="analytic" ? epoch_ : curve->start_ns;
    const int n=std::max(1,int(std::ceil(duration/.04)));
    for(int j=0;j<=n;++j) {
      const auto t=begin+std::llround((double(j)/n)*duration*1e9);const auto ref=supplied ? sampleHeldTrajectory(*supplied,t) : referenceAt(t);
      geometry_msgs::msg::PoseStamped pose;pose.header=path.header;pose.header.stamp=rclcpp::Time(t,RCL_ROS_TIME);
      pose.pose.position.x=ref.pose.position.x();pose.pose.position.y=ref.pose.position.y();
      pose.pose.orientation.z=std::sin(ref.pose.yaw/2);pose.pose.orientation.w=std::cos(ref.pose.yaw/2);path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }
  void receiveNavigation(const motion2d_interfaces::msg::NavigationReference & message) {
    try {
      auto next=fromNavigationMessage(message);
      if(!last_stamp_ || !enabled_) throw std::invalid_argument("Need current enabled odometry");
      if(!certifyBezier(next.motion.curve,next.regions,navigation_limits_).certified) throw std::invalid_argument("Reference certificate failed");
      if(schedule_.empty()) {
        const auto first=next.motion.curve.sample(0);
        if(state_.velocity.norm()>.02 || std::abs(state_.yaw_rate)>.02 ||
           (first.position-state_.pose.position).norm()>.03 || std::abs(wrapAngle(next.motion.yaw-state_.pose.yaw))>.02)
          throw std::invalid_argument("Need a nearby stationary restart");
        schedule_.reset(Pose2D{first.position,next.motion.yaw});
      }
      const auto result=schedule_.accept(next,*last_stamp_);
      if(!result.accepted) throw std::invalid_argument(result.reason);
      if(mpc_) mpc_->reset();
      publishPath(*last_stamp_,&next.motion);
      std_msgs::msg::Int64 ack;ack.data=next.motion.start_ns;accepted_pub_->publish(ack);
      status("navigation_reference_accepted");
    } catch(const std::exception & e) {status(std::string("navigation_reference_rejected:")+e.what());}
  }
  void receiveTrajectory(const motion2d_interfaces::msg::Trajectory2D & m) {
    try {
      auto next=fromTrajectoryMessage(m);
      if(!last_stamp_ || next.start_ns<*last_stamp_) throw std::invalid_argument("Need current odometry and a future start");
      if(trajectory_ && (*last_stamp_-trajectory_->start_ns)*1e-9<trajectory_->curve.duration()) throw std::invalid_argument("Busy; online handover belongs to ch19");
      const auto first=next.curve.sample(0),last=next.curve.sample(next.curve.duration());
      if((first.position-state_.pose.position).norm()>1e-5 || state_.velocity.norm()>1e-5 || std::abs(state_.yaw_rate)>1e-5 ||
        first.velocity.norm()>1e-7 || first.acceleration.norm()>1e-7 || last.velocity.norm()>1e-7 || last.acceleration.norm()>1e-7 ||
        std::abs(wrapAngle(next.yaw-state_.pose.yaw))>1e-5) throw std::invalid_argument("Need matching stationary endpoints and yaw");
      trajectory_=std::move(next);if(mpc_) mpc_->reset();publishPath(*last_stamp_);status("trajectory_accepted");
    } catch(const std::exception & e) {status(std::string("trajectory_rejected:")+e.what());}
  }
  void receive(const nav_msgs::msg::Odometry & message) {
    try {
      const auto stamp=rclcpp::Time(message.header.stamp);const auto ns=stamp.nanoseconds();
      const auto state=stateFromOdometry(message);
      if(last_stamp_ && ns==*last_stamp_) return;
      if((now().nanoseconds()-ns)*1e-9>timeout_) {
        if(last_stamp_) brake(now(),"stale_odometry_braking");
        else status("stale_odometry_braking");
        return;
      }
      const bool reset=!last_stamp_ || ns<*last_stamp_;
      state_=state;last_stamp_=ns;
      if(reset) {
        epoch_=ns;last_control_.reset();trajectory_.reset();reference_config_.origin=state.pose;
        previous_force_.setZero();if(mpc_) mpc_->reset();schedule_.reset(state.pose);
        if(source_=="analytic") publishPath(ns);
        else {nav_msgs::msg::Path empty;empty.header=message.header;path_pub_->publish(empty);}
      }
      if(source_=="navigation") schedule_.advance(ns);
      if(last_control_ && ns-*last_control_<period_) return;
      last_control_=ns;
      if(!enabled_) {brake(stamp,"disabled_braking");return;}
      if(source_=="trajectory" && !trajectory_) {brake(stamp,"waiting_trajectory_braking");return;}
      if(source_=="navigation" && schedule_.empty()) {brake(stamp,"waiting_navigation_braking");return;}
      // tracker_observation_begin
      const auto reference=referenceAt(ns);
      auto command=trackPd(state_,reference,gains_,model_);
      std::string label=command.saturated ? "saturated" : "tracking";
      if(mpc_) {
        std::vector<State2D> future;std::vector<ConvexRegion> regions;
        for(int j=1;j<=mpc_config_.horizon;++j) {
          const auto time=ns+j*period_;future.push_back(referenceAt(time));
          if(source_=="navigation") {
            const auto * region=schedule_.region(time);
            if(!region) throw std::invalid_argument("Missing navigation region");
            regions.push_back(*region);
          }
        }
        const auto begin=std::chrono::steady_clock::now();
        const auto result=mpc_->step(state_,future,previous_force_,regions);
        motion2d_interfaces::msg::MpcStatus diagnostic;diagnostic.header=message.header;
        diagnostic.compute_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
        diagnostic.status=result.solver.status;diagnostic.iterations=result.solver.iterations;
        diagnostic.primal_residual=result.solver.primal_residual;diagnostic.dual_residual=result.solver.dual_residual;
        diagnostic.max_violation=result.violation;mpc_pub_->publish(diagnostic);publishPrediction(result,ns);
        command.requested.force=command.applied.force=result.force;
        label=result.solver.solved() ? "tracking_mpc" : "mpc_"+result.solver.status+"_braking";
        if(!result.solver.solved()) command.requested=command.applied=dampingBrake(state_,model_);
      }
      if(controller_=="ideal") sendPose(referenceAt(ns+period_).pose,stamp,"tracking_ideal");
      else send(command.applied,stamp,label);
      reference_pub_->publish(referenceOdometry(reference,message.header.stamp));
      geometry_msgs::msg::AccelStamped acceleration;acceleration.header=message.header;
      acceleration.accel.linear.x=reference.acceleration.x();acceleration.accel.linear.y=reference.acceleration.y();
      acceleration.accel.angular.z=reference.yaw_acceleration;acceleration_pub_->publish(acceleration);
      // tracker_observation_end
      geometry_msgs::msg::WrenchStamped requested;requested.header=message.header;
      requested.wrench.force.x=command.requested.force.x();requested.wrench.force.y=command.requested.force.y();
      requested.wrench.torque.z=command.requested.torque;requested_->publish(requested);
    } catch(const std::exception & e) {
      if(last_stamp_) brake(now(),std::string("invalid_control_input_braking:")+e.what());
      else status(std::string("invalid_odometry:")+e.what());
    }
  }
  std::string controller_;ReferenceSchedule schedule_;TrajectoryLimits navigation_limits_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr accepted_pub_;
  rclcpp::Subscription<motion2d_interfaces::msg::NavigationReference>::SharedPtr navigation_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  MpcConfig mpc_config_;std::unique_ptr<LinearMpc> mpc_;Eigen::Vector2d previous_force_=Eigen::Vector2d::Zero();
  rclcpp::Publisher<motion2d_interfaces::msg::MpcStatus>::SharedPtr mpc_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr prediction_pub_;
  InertialParameters model_;PdGains gains_;TrackingReferenceConfig reference_config_;State2D state_;
  std::string source_,last_status_;bool enabled_=true;double timeout_;std::int64_t period_,epoch_=0;
  std::optional<std::int64_t> last_stamp_,last_control_;std::optional<TimedTrajectory> trajectory_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_,requested_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr acceleration_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr reference_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<motion2d_interfaces::msg::Trajectory2D>::SharedPtr trajectory_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;rclcpp::TimerBase::SharedPtr watchdog_;
};
}
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<motion2d::TrackerNode>());}
  catch(const std::exception & e) {RCLCPP_ERROR(rclcpp::get_logger("tracker"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
