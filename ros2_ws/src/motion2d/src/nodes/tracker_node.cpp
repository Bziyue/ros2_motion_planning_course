#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "motion2d/control/pd_tracker.hpp"
#include "motion2d/control/tracking_reference.hpp"
#include "motion2d/ros/control_messages.hpp"
#include "motion2d/ros/trajectory_messages.hpp"
#include <optional>
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
    source_=declare_parameter("reference.source",std::string("analytic"));
    if(source_!="analytic" && source_!="trajectory") throw std::invalid_argument("Expected analytic or trajectory reference");
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
    enable_=create_service<std_srvs::srv::SetBool>("/tracker/enable",[this](const std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response){enabled_=request->data;response->success=true;response->message=enabled_ ? "enabled" : "bounded damping brake";
        if(!enabled_ && last_stamp_) send(dampingBrake(state_,model_),now(),"disabled_braking");});
    watchdog_=create_wall_timer(std::chrono::milliseconds(20),[this]{
      if(last_stamp_ && (now().nanoseconds()-*last_stamp_)*1e-9>timeout_)
        send(dampingBrake(state_,model_),now(),"stale_odometry_braking");
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
    wrench_->publish(m);status(label);
  }
  State2D referenceAt(std::int64_t stamp) const {
    if(source_=="analytic") return sampleTrackingReference(reference_config_,(stamp-epoch_)*1e-9);
    return sampleHeldTrajectory(*trajectory_,stamp);
  }
  void publishPath(std::int64_t stamp) {
    nav_msgs::msg::Path path;path.header.frame_id="odom";path.header.stamp=rclcpp::Time(stamp,RCL_ROS_TIME);
    const double duration=source_=="analytic" ? 24. : trajectory_->curve.duration();
    const auto begin=source_=="analytic" ? epoch_ : trajectory_->start_ns;
    const int n=std::max(1,int(std::ceil(duration/.04)));
    for(int j=0;j<=n;++j) {
      const auto t=begin+std::llround((double(j)/n)*duration*1e9);const auto ref=referenceAt(t);
      geometry_msgs::msg::PoseStamped pose;pose.header=path.header;pose.header.stamp=rclcpp::Time(t,RCL_ROS_TIME);
      pose.pose.position.x=ref.pose.position.x();pose.pose.position.y=ref.pose.position.y();
      pose.pose.orientation.z=std::sin(ref.pose.yaw/2);pose.pose.orientation.w=std::cos(ref.pose.yaw/2);path.poses.push_back(pose);
    }
    path_pub_->publish(path);
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
      trajectory_=std::move(next);publishPath(*last_stamp_);status("trajectory_accepted");
    } catch(const std::exception & e) {status(std::string("trajectory_rejected:")+e.what());}
  }
  void receive(const nav_msgs::msg::Odometry & message) {
    try {
      const auto stamp=rclcpp::Time(message.header.stamp);const auto ns=stamp.nanoseconds();
      const auto state=stateFromOdometry(message);
      if(last_stamp_ && ns==*last_stamp_) return;
      if((now().nanoseconds()-ns)*1e-9>timeout_) {
        if(last_stamp_) send(dampingBrake(state_,model_),now(),"stale_odometry_braking");
        else status("stale_odometry_braking");
        return;
      }
      const bool reset=!last_stamp_ || ns<*last_stamp_;
      state_=state;last_stamp_=ns;
      if(reset) {
        epoch_=ns;last_control_.reset();trajectory_.reset();reference_config_.origin=state.pose;
        if(source_=="analytic") publishPath(ns);
        else {nav_msgs::msg::Path empty;empty.header=message.header;path_pub_->publish(empty);}
      }
      if(last_control_ && ns-*last_control_<period_) return;
      last_control_=ns;
      if(!enabled_) {send(dampingBrake(state_,model_),stamp,"disabled_braking");return;}
      if(source_=="trajectory" && !trajectory_) {send(dampingBrake(state_,model_),stamp,"waiting_trajectory_braking");return;}
      // tracker_observation_begin
      const auto reference=referenceAt(ns);
      const auto command=trackPd(state_,reference,gains_,model_);
      send(command.applied,stamp,command.saturated ? "saturated" : "tracking");
      reference_pub_->publish(referenceOdometry(reference,message.header.stamp));
      geometry_msgs::msg::AccelStamped acceleration;acceleration.header=message.header;
      acceleration.accel.linear.x=reference.acceleration.x();acceleration.accel.linear.y=reference.acceleration.y();
      acceleration.accel.angular.z=reference.yaw_acceleration;acceleration_pub_->publish(acceleration);
      // tracker_observation_end
      geometry_msgs::msg::WrenchStamped requested;requested.header=message.header;
      requested.wrench.force.x=command.requested.force.x();requested.wrench.force.y=command.requested.force.y();
      requested.wrench.torque.z=command.requested.torque;requested_->publish(requested);
    } catch(const std::exception & e) {
      if(last_stamp_) send(dampingBrake(state_,model_),now(),std::string("invalid_odometry_braking:")+e.what());
      else status(std::string("invalid_odometry:")+e.what());
    }
  }
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
