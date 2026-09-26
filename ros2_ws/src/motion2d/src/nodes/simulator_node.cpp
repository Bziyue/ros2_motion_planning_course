#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "motion2d/ros/world_parameters.hpp"
#include "motion2d/sim/robot_model.hpp"
#include "motion2d/sim/sim_clock.hpp"

namespace motion2d
{
/** @brief Single-clock robot plant. Chapter 04 uses aligned map/odom truth TF.
 *  @details Commands are consumed at a tick boundary. A rejected sweep freezes
 *  the last accepted state and time; reset starts a new trial. This node's
 *  The sim namespace contains truth, not a localization algorithm's output.
 */
class Simulator : public rclcpp::Node
{
public:
  Simulator() : Node("simulator"), clock_(readStep()),
    broadcaster_(*this), static_broadcaster_(*this)
  {
    const WorldConfig config = readWorldConfig(*this);
    world_ = generateWorld(config);
    radius_ = config.robot_radius;
    initial_.position = world_.start;
    initial_.yaw = declare_parameter("yaw", 0.0);
    model_ = declare_parameter<std::string>("model", "ideal");
    command_timeout_ = declare_parameter("command_timeout", 0.5);
    if (!std::isfinite(initial_.yaw) || !std::isfinite(command_timeout_) ||
      command_timeout_ <= 0.0 || model_ != "ideal")
    {
      throw std::invalid_argument("Expected model=ideal, finite yaw and positive command_timeout");
    }
    state_ = idealPose(initial_);
    clock_.setPaused(declare_parameter("start_paused", false));
    const auto retained = rclcpp::QoS(1).reliable().transient_local();
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/sim/pose", retained);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/sim/velocity", retained);
    acceleration_pub_ = create_publisher<geometry_msgs::msg::AccelStamped>("/sim/acceleration", retained);
    status_pub_ = create_publisher<std_msgs::msg::String>("/sim/status", retained);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/robot", retained);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/command/pose", rclcpp::QoS(1),
      [this](const geometry_msgs::msg::PoseStamped & message) {receivePose(message);});
    createServices();
    publishStaticFrames();
    rememberPosition();
    setStatus(clock_.paused() ? "paused" : "running");
    timer_ = create_wall_timer(clock_.stepDuration(), [this]() {
        if (!clock_.paused()) {advanceTick(false);}
        publishState();  // Frozen timestamps also serve late subscribers when paused.
      });
    static_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {publishStaticFrames();});
    marker_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {publishMarkers();});
    publishState();
    publishMarkers();
    RCLCPP_INFO(get_logger(), "model=%s, radius=%.3f m; /sim/* is simulation truth",
      model_.c_str(), radius_);
  }

private:
  std::chrono::nanoseconds readStep()
  {
    const double dt = declare_parameter("dt", 0.005);
    if (!std::isfinite(dt) || dt < 1e-6 || dt > 1.0) {
      throw std::invalid_argument("dt must be in [1e-6, 1] seconds");
    }
    return std::chrono::nanoseconds(std::llround(dt * 1e9));
  }

  bool freshHeader(const std_msgs::msg::Header & header) const
  {
    const double stamp = header.stamp.sec + header.stamp.nanosec * 1e-9;
    // Zero stamp means "apply at the next tick"; useful for manual commands.
    return stamp == 0.0 || (stamp <= clock_.seconds() + dt() &&
           stamp >= clock_.seconds() - command_timeout_);
  }

  void receivePose(const geometry_msgs::msg::PoseStamped & message)
  {
    const auto & p = message.pose.position;
    const auto & q = message.pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (blocked_ || (message.header.frame_id != "odom" && message.header.frame_id != "map") ||
      !freshHeader(message.header) || !std::isfinite(p.x) || !std::isfinite(p.y) ||
      !std::isfinite(p.z) || !std::isfinite(norm) || std::abs(p.z) > 1e-9 ||
      std::abs(q.x) > 1e-9 || std::abs(q.y) > 1e-9 || std::abs(norm - 1.0) > 1e-6)
    {
      RCLCPP_WARN(get_logger(), "Pose rejected: check planar pose, frame, stamp, or reset collision");
      return;
    }
    pending_pose_ = Pose2D{{p.x, p.y}, 2.0 * std::atan2(q.z, q.w)};
  }

  // tick_begin
  bool advanceTick(bool single_step)
  {
    if (blocked_) {return false;}
    State2D next = state_;
    if (pending_pose_) {next = idealPose(*pending_pose_);}
    pending_pose_.reset();
    if (!sweptDiskIsFree(world_, state_.pose.position, next.pose.position, radius_)) {
      blocked_ = true;
      clock_.setPaused(true);
      setStatus("collision_predicted");
      RCLCPP_WARN(get_logger(), "Swept disk blocked; state/time frozen. Reset to begin a new trial");
      return false;
    }
    state_ = next;
    if (single_step) {clock_.singleStep();} else {clock_.advance();}
    rememberPosition();
    return true;
  }
  // tick_end

  void createServices()
  {
    pause_service_ = create_service<std_srvs::srv::SetBool>("/sim/pause",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response) {
        response->success = request->data || !blocked_;
        if (response->success) {
          clock_.setPaused(request->data);
          if (!blocked_) {setStatus(request->data ? "paused" : "running");}
        }
        response->message = blocked_ ? "collision_predicted; reset required" : status_;
      });
    step_service_ = create_service<std_srvs::srv::Trigger>("/sim/step",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = clock_.paused() && advanceTick(true);
        response->message = response->success ? "advanced one tick" : "pause first; reset if blocked";
        publishState();
        publishMarkers();
      });
    reset_service_ = create_service<std_srvs::srv::Trigger>("/sim/reset",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        clock_.reset();
        state_ = idealPose(initial_);
        pending_pose_.reset();
        blocked_ = false;
        trail_.clear();
        rememberPosition();
        setStatus("paused");
        publishStaticFrames();
        publishState();
        publishMarkers();
        response->success = true;
        response->message = "new paused trial; state, command and trail cleared";
      });
  }

  double dt() const {return clock_.stepDuration().count() * 1e-9;}

  void setStatus(const std::string & status)
  {
    status_ = status;
    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
  }

  void publishStaticFrames()
  {
    std::vector<geometry_msgs::msg::TransformStamped> fixed;
    for (const auto & frames : std::vector<std::pair<std::string, std::string>>{
        {"map", "odom"}, {"base_link", "laser"}, {"base_link", "imu_link"}})
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = frames.first;
      t.child_frame_id = frames.second;
      t.transform.rotation.w = 1.0;
      fixed.push_back(t);
    }
    static_broadcaster_.sendTransform(fixed);
  }

  void publishState()
  {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = rclcpp::Time(clock_.nanoseconds(), RCL_ROS_TIME);
    clock_pub_->publish(clock);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = clock.clock;
    pose.header.frame_id = "odom";
    pose.pose.position.x = state_.pose.position.x();
    pose.pose.position.y = state_.pose.position.y();
    pose.pose.orientation.z = std::sin(state_.pose.yaw / 2.0);
    pose.pose.orientation.w = std::cos(state_.pose.yaw / 2.0);
    pose_pub_->publish(pose);
    geometry_msgs::msg::TwistStamped velocity;
    velocity.header = pose.header;
    velocity.twist.linear.x = state_.velocity.x();
    velocity.twist.linear.y = state_.velocity.y();
    velocity.twist.angular.z = state_.yaw_rate;
    velocity_pub_->publish(velocity);
    geometry_msgs::msg::AccelStamped acceleration;
    acceleration.header = pose.header;
    acceleration.accel.linear.x = state_.acceleration.x();
    acceleration.accel.linear.y = state_.acceleration.y();
    acceleration.accel.angular.z = state_.yaw_acceleration;
    acceleration_pub_->publish(acceleration);
    geometry_msgs::msg::TransformStamped tf;
    tf.header = pose.header;
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = pose.pose.position.x;
    tf.transform.translation.y = pose.pose.position.y;
    tf.transform.rotation = pose.pose.orientation;
    broadcaster_.sendTransform(tf);
  }

  void rememberPosition()
  {
    geometry_msgs::msg::Point p;
    p.x = state_.pose.position.x();
    p.y = state_.pose.position.y();
    p.z = 0.03;
    if (trail_.empty() || std::hypot(p.x - trail_.back().x, p.y - trail_.back().y) > 0.002) {
      trail_.push_back(p);
      if (trail_.size() > 2000) {trail_.pop_front();}
    }
  }

  void publishMarkers()
  {
    using Marker = visualization_msgs::msg::Marker;
    Marker disk;
    disk.header.frame_id = "base_link";
    disk.header.stamp = rclcpp::Time(clock_.nanoseconds(), RCL_ROS_TIME);
    disk.ns = "robot";
    disk.type = Marker::CYLINDER;
    disk.pose.orientation.w = 1.0;
    disk.pose.position.z = 0.06;
    disk.scale.x = disk.scale.y = std::max(2.0 * radius_, 0.04);  // Visible dot for radius=0.
    disk.scale.z = 0.08;
    disk.color.r = blocked_ ? 0.9F : 0.05F;
    disk.color.g = blocked_ ? 0.15F : 0.6F;
    disk.color.b = 0.6F;
    disk.color.a = 1.0F;
    Marker trail = disk;
    trail.header.frame_id = "odom";
    trail.ns = "trail";
    trail.type = Marker::LINE_STRIP;
    trail.pose.position.z = 0.0;
    trail.scale.x = 0.035;
    trail.points.assign(trail_.begin(), trail_.end());
    trail.action = trail_.size() >= 2 ? Marker::ADD : Marker::DELETE;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers = {disk, trail};
    marker_pub_->publish(markers);
  }

  SimClock clock_;
  World2D world_;
  Pose2D initial_;
  State2D state_;
  std::optional<Pose2D> pending_pose_;
  double radius_, command_timeout_;
  bool blocked_ = false;
  std::string model_, status_;
  std::deque<geometry_msgs::msg::Point> trail_;
  tf2_ros::TransformBroadcaster broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr acceleration_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr step_service_, reset_service_;
  rclcpp::TimerBase::SharedPtr timer_, static_timer_, marker_timer_;
};
}  // namespace motion2d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<motion2d::Simulator>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("simulator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
