#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "motion2d/ros/world_parameters.hpp"
#include "motion2d/ros/lidar_messages.hpp"
#include "motion2d/ros/imu_messages.hpp"
#include "motion2d/ros/odometry_messages.hpp"
#include "motion2d/ros/trajectory_messages.hpp"
#include "motion2d/sim/robot_model.hpp"
#include "motion2d/sim/sim_clock.hpp"
#include "motion2d/sim/sensor_scheduler.hpp"

namespace motion2d
{
/** @brief Single-clock robot plant. Chapter 04 uses aligned map/odom truth TF.
 *  @details Commands are consumed at a tick boundary. A rejected sweep freezes
 *  the last accepted state and time; reset starts a new trial.
 *  The sim namespace contains truth, not a localization algorithm's output.
 */
class Simulator : public rclcpp::Node
{
public:
  Simulator() : Node("simulator"), clock_(readStep()),
    static_broadcaster_(*this)
  {
    publish_truth_tf_ = declare_parameter("publish_truth_tf", true);
    if (publish_truth_tf_) {broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);}
    const WorldConfig config = readWorldConfig(*this);
    world_ = generateWorld(config);
    radius_ = config.robot_radius;
    initial_.position = world_.start;
    initial_.yaw = declare_parameter("yaw", 0.0);
    model_ = declare_parameter<std::string>("model", "ideal");
    command_timeout_ = declare_parameter("command_timeout", 0.5);
    speed_max_ = declare_parameter("speed_max", 1.0);
    yaw_rate_max_ = declare_parameter("yaw_rate_max", 1.0);
    if (!std::isfinite(initial_.yaw) || !std::isfinite(command_timeout_) ||
      !std::isfinite(speed_max_) || !std::isfinite(yaw_rate_max_) ||
      command_timeout_ <= 0.0 || speed_max_ <= 0.0 || yaw_rate_max_ <= 0.0 ||
      (model_ != "ideal" && model_ != "velocity" && model_ != "inertial" && model_ != "reference" && model_ != "trajectory"))
    {
      throw std::invalid_argument("Unknown model or invalid limit/timeout; see ch04 configuration");
    }
    if (model_ == "inertial") {readInertia();}
    if (model_ == "reference") {
      reference_radius_ = declare_parameter("reference_radius", 1.0);
      reference_omega_ = declare_parameter("reference_omega", .4);
      if (!std::isfinite(reference_radius_) || reference_radius_ <= 0 ||
        !std::isfinite(reference_omega_))
      {
        throw std::invalid_argument("Reference radius must be positive, omega finite");
      }
    }
    state_ = initialState();
    clock_.setPaused(declare_parameter("start_paused", false));
    const auto retained = rclcpp::QoS(1).reliable().transient_local();
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
    truth_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/ground_truth/odometry", rclcpp::QoS(100).reliable().transient_local());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/sim/pose", retained);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/sim/velocity", retained);
    acceleration_pub_ = create_publisher<geometry_msgs::msg::AccelStamped>("/sim/acceleration", retained);
    status_pub_ = create_publisher<std_msgs::msg::String>("/sim/status", retained);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/robot", retained);
    if (declare_parameter("lidar.enabled", false)) {readLidar();}
    if (declare_parameter("imu.enabled", false)) {readImu();}
    if (model_ == "ideal") {
      pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/command/pose", rclcpp::QoS(1),
        [this](const geometry_msgs::msg::PoseStamped & message) {receivePose(message);});
    } else if (model_ == "velocity") {
      velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/command/velocity", rclcpp::QoS(1),
        [this](const geometry_msgs::msg::TwistStamped & message) {receiveVelocity(message);});
    } else if (model_ == "inertial") {
      wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/command/wrench", rclcpp::QoS(1),
        [this](const geometry_msgs::msg::WrenchStamped & message) {receiveWrench(message);});
    }
    if (model_ == "trajectory") {
      trajectory_status_pub_ = create_publisher<std_msgs::msg::String>("/sim/trajectory_status", retained);
      trajectory_sub_ = create_subscription<motion2d_interfaces::msg::Trajectory2D>(
        "/plan/trajectory", 1, [this](const motion2d_interfaces::msg::Trajectory2D & m) {receiveTrajectory(m);});
      setTrajectoryStatus("idle");
    }
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
  SensorScheduler readSchedule(const std::string & name, double default_rate)
  {
    const double rate = declare_parameter(name, default_rate);
    SensorScheduler schedule(rate);
    if (rate > 1.0 / dt()) {
      throw std::invalid_argument(name + " must not exceed the simulation tick rate");
    }
    return schedule;
  }

  /** @brief Exact sample inside the last accepted ZOH/reference interval.
   * @details A boundary sample belongs to the interval that just ended (left
   * limit of acceleration); a later command cannot rewrite a published sample.
   * Ideal pose changes happen only at the tick endpoint, with IMU disabled.
   */
  State2D acquisitionState(std::int64_t stamp) const
  {
    if (stamp == clock_.nanoseconds()) {return state_;}
    const double offset = (stamp - interval_start_ns_) * 1e-9;
    if (model_ == "trajectory" && trajectory_) {return sampleHeldTrajectory(*trajectory_,stamp);}
    if (model_ == "reference") {
      return sampleCircle(initial_, reference_radius_, reference_omega_, stamp * 1e-9);
    }
    if (model_ == "inertial") {
      return stepInertial(interval_start_, interval_wrench_, inertia_, offset);
    }
    if (model_ == "velocity") {
      return stepVelocity(interval_start_, interval_velocity_, offset);
    }
    return interval_start_;  // No interpolation through a nonphysical teleport.
  }

  // sensor_dispatch_begin
  void publishSensors()
  {
    const auto now = clock_.nanoseconds();
    std::int64_t last_transform = -1;
    while (true) {
      const auto never = std::numeric_limits<std::int64_t>::max();
      const auto laser_time = lidar_schedule_ ? lidar_schedule_->nextStamp() : never;
      const auto imu_time = imu_schedule_ ? imu_schedule_->nextStamp() : never;
      const auto stamp = std::min(laser_time, imu_time);
      if (stamp > now) {break;}
      const auto sampled = acquisitionState(stamp);
      publishTruth(sampled, stamp);
      last_transform = stamp;
      if (stamp == laser_time) {publishLidar(sampled, stamp); lidar_schedule_->advance();}
      if (stamp == imu_time) {publishImu(sampled, stamp); imu_schedule_->advance();}
    }
    if (last_transform != now) {publishTruth(state_, now);}
  }
  // sensor_dispatch_end

  void readImu()
  {
    if (model_ != "reference" && model_ != "inertial" && model_ != "trajectory") {
      throw std::invalid_argument("IMU requires model=reference, inertial or trajectory; jumps have no physical IMU");
    }
    gravity_ = declare_parameter("imu.gravity", 9.81);
    if (!std::isfinite(gravity_) || gravity_ <= 0) {
      throw std::invalid_argument("imu.gravity must be finite and positive (m/s^2)");
    }
    const auto read_axes = [this](const std::string & name, Eigen::Vector3d & values) {
        const auto input = declare_parameter<std::vector<double>>(
          name, {values.x(), values.y(), values.z()});
        if (input.size() != 3) {throw std::invalid_argument(name + " needs [x,y,z]");}
        values = {input[0], input[1], input[2]};
      };
    read_axes("imu.gyro_stddev", imu_noise_.gyro_stddev);
    read_axes("imu.accel_stddev", imu_noise_.accel_stddev);
    validateImuNoise(imu_noise_);
    const auto seed = declare_parameter<std::int64_t>("imu.noise_seed", 6060);
    if (seed < 0 || seed > std::numeric_limits<std::uint32_t>::max()) {
      throw std::invalid_argument("IMU noise_seed must fit an unsigned 32-bit integer");
    }
    imu_seed_ = static_cast<std::uint32_t>(seed);
    imu_random_.seed(imu_seed_);
    imu_schedule_ = readSchedule("imu.rate", 200.0);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", rclcpp::SensorDataQoS());
    if (declare_parameter("imu.visualize", true)) {
      imu_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization/imu", rclcpp::QoS(1).reliable().transient_local());
    }
    RCLCPP_INFO(get_logger(), "Raw IMU: %.3f Hz, seed=%u, gravity=%.3f; no orientation",
      1.0 / imu_schedule_->period(), imu_seed_, gravity_);
  }

  void publishImu(const State2D & sampled, std::int64_t stamp)
  {
    const auto sample = addImuNoise(idealImu(sampled, gravity_), imu_noise_, imu_random_);
    const auto message = toImuMessage(sample, imu_noise_, rclcpp::Time(stamp, RCL_ROS_TIME));
    imu_pub_->publish(message);
    if (imu_marker_pub_ && (last_imu_marker_ns_ < 0 || stamp - last_imu_marker_ns_ >= 50000000)) {
      imu_marker_pub_->publish(imuMarkers(message, sampled.pose));
      last_imu_marker_ns_ = stamp;
    }
  }

  void readLidar()
  {
    lidar_config_.beams = declare_parameter("lidar.beams", lidar_config_.beams);
    lidar_config_.angle_min = declare_parameter("lidar.angle_min", lidar_config_.angle_min);
    lidar_config_.fov = declare_parameter("lidar.fov", lidar_config_.fov);
    lidar_config_.range_min = declare_parameter("lidar.range_min", lidar_config_.range_min);
    lidar_config_.range_max = declare_parameter("lidar.range_max", lidar_config_.range_max);
    lidar_config_.range_stddev = declare_parameter("lidar.range_stddev", 0.0);
    validateLidarConfig(lidar_config_);
    const auto seed = declare_parameter<std::int64_t>("lidar.noise_seed", 4242);
    if (seed < 0 || seed > std::numeric_limits<std::uint32_t>::max()) {
      throw std::invalid_argument("Lidar noise_seed must fit an unsigned 32-bit integer");
    }
    lidar_seed_ = static_cast<std::uint32_t>(seed);
    lidar_random_.seed(lidar_seed_);
    lidar_schedule_ = readSchedule("lidar.rate", 10.0);
    if (declare_parameter<std::string>("lidar.backend", "cpu") != "cpu" ||
      declare_parameter<std::string>("lidar.scan_model", "snapshot") != "snapshot")
    {
      throw std::invalid_argument("Chapter 05 supports lidar backend=cpu, scan_model=snapshot only");
    }
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());
    if (declare_parameter("lidar.visualize_beams", true)) {
      beam_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization/lidar", rclcpp::QoS(1).reliable().transient_local());
    }
    RCLCPP_INFO(get_logger(),
      "CPU snapshot lidar: %d beams, %.3f Hz, range [%.3f, %.3f] m, sigma=%.4f m, seed=%u",
      lidar_config_.beams, 1.0 / lidar_schedule_->period(), lidar_config_.range_min, lidar_config_.range_max,
      lidar_config_.range_stddev, lidar_seed_);
  }

  void publishLidar(const State2D & sampled, std::int64_t stamp)
  {
    auto ranges = scanCpu(world_, sampled.pose, lidar_config_);
    addRangeNoise(ranges, lidar_config_, lidar_random_);
    const auto scan = toLaserScan(lidar_config_, ranges,
      rclcpp::Time(stamp, RCL_ROS_TIME), lidar_schedule_->period());
    scan_pub_->publish(scan);
    if (beam_pub_) {beam_pub_->publish(lidarBeamMarkers(scan, sampled.pose));}
  }

  void readInertia()
  {
    inertia_.mass = declare_parameter("mass", 1.0);
    inertia_.linear_drag = declare_parameter("linear_drag", 0.0);
    inertia_.angular_drag = declare_parameter("angular_drag", 0.0);
    inertia_.inertia_z = declare_parameter("inertia_z", -1.0);
    inertia_.force_max = declare_parameter("force_max", 2.0);
    inertia_.torque_max = declare_parameter("torque_max", 0.2);
    if (inertia_.inertia_z == -1.0) {
      inertia_.inertia_z = 0.5 * inertia_.mass * radius_ * radius_;
    }
    validateInertialParameters(inertia_);
    RCLCPP_INFO(get_logger(), "mass=%.3f kg, inertia_z=%.5f kg m^2; force inputs are in odom",
      inertia_.mass, inertia_.inertia_z);
  }

  std::chrono::nanoseconds readStep()
  {
    const double dt = declare_parameter("dt", 0.005);
    if (!std::isfinite(dt) || dt < 1e-6 || dt > 1.0) {
      throw std::invalid_argument("dt must be in [1e-6, 1] seconds");
    }
    return std::chrono::nanoseconds(std::llround(dt * 1e9));
  }

  State2D initialState() const
  {
    return model_ == "reference" ? sampleCircle(initial_, reference_radius_, reference_omega_, 0.0) :
           idealPose(initial_);
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

  void receiveVelocity(const geometry_msgs::msg::TwistStamped & message)
  {
    const auto & v = message.twist.linear;
    const auto & w = message.twist.angular;
    if (blocked_ || (message.header.frame_id != "odom" && message.header.frame_id != "base_link") ||
      !freshHeader(message.header) || !std::isfinite(v.x) || !std::isfinite(v.y) ||
      !std::isfinite(w.z) || v.z != 0.0 || w.x != 0.0 || w.y != 0.0)
    {
      RCLCPP_WARN(get_logger(), "Velocity rejected: expected finite planar odom/base_link command");
      return;
    }
    velocity_command_ = limitVelocity(
      {{v.x, v.y}, w.z, message.header.frame_id == "base_link"}, speed_max_, yaw_rate_max_);
    acceptHeldCommand(message.header);
  }

  void acceptHeldCommand(const std_msgs::msg::Header & header)
  {
    const std::int64_t stamp = static_cast<std::int64_t>(header.stamp.sec) * 1000000000 +
      header.stamp.nanosec;
    command_deadline_ = (stamp == 0 ? clock_.nanoseconds() : stamp) +
      std::llround(command_timeout_ * 1e9);
    have_command_ = true;
    setStatus(clock_.paused() ? "paused" : "running");
  }

  void receiveWrench(const geometry_msgs::msg::WrenchStamped & message)
  {
    const auto & f = message.wrench.force;
    const auto & t = message.wrench.torque;
    if (blocked_ || message.header.frame_id != "odom" || !freshHeader(message.header) ||
      !std::isfinite(f.x) || !std::isfinite(f.y) || !std::isfinite(t.z) ||
      f.z != 0.0 || t.x != 0.0 || t.y != 0.0)
    {
      RCLCPP_WARN(get_logger(), "Wrench rejected: expected finite planar force/torque in odom");
      return;
    }
    wrench_command_ = {{f.x, f.y}, t.z};
    acceptHeldCommand(message.header);
  }

  void setTrajectoryStatus(const std::string & status)
  {
    if (!trajectory_status_pub_ || status == trajectory_status_) {return;}
    trajectory_status_ = status; std_msgs::msg::String message; message.data = status;
    trajectory_status_pub_->publish(message);
  }

  void receiveTrajectory(const motion2d_interfaces::msg::Trajectory2D & message)
  {
    try {
      auto command = fromTrajectoryMessage(message);
      if (blocked_) {throw std::invalid_argument("collision requires reset");}
      if (command.start_ns < clock_.nanoseconds()) {throw std::invalid_argument("start is in the past");}
      if (trajectory_ && (clock_.nanoseconds()-trajectory_->start_ns)*1e-9 < trajectory_->curve.duration())
      {throw std::invalid_argument("busy; ch14 executes one curve at a time");}
      const auto first=command.curve.sample(0), last=command.curve.sample(command.curve.duration());
      if ((first.position-state_.pose.position).norm()>1e-7 || first.velocity.norm()>1e-7 ||
        first.acceleration.norm()>1e-7 || state_.velocity.norm()>1e-7 || state_.acceleration.norm()>1e-7 ||
        last.velocity.norm()>1e-7 || last.acceleration.norm()>1e-7 ||
        std::abs(wrapAngle(command.yaw-state_.pose.yaw))>1e-7)
      {throw std::invalid_argument("continuous rest endpoints and unchanged yaw required");}
      trajectory_=std::move(command);
      setTrajectoryStatus("accepted");
    } catch (const std::invalid_argument & error) {
      setTrajectoryStatus(std::string("rejected: ")+error.what());
    }
  }

  // tick_begin
  bool advanceTick(bool single_step)
  {
    if (blocked_) {return false;}
    State2D next = state_;
    double padding = 0.0;
    if (have_command_ && clock_.nanoseconds() >= command_deadline_) {
      have_command_ = false;
      setStatus("command_timeout");
    }
    if (model_ == "ideal" && pending_pose_) {next = idealPose(*pending_pose_);}
    if (model_ == "velocity") {
      const auto command = have_command_ ? velocity_command_ : VelocityCommand{};
      next = stepVelocity(state_, command, dt());
      padding = velocitySweepPadding(command, dt());
    }
    if (model_ == "inertial") {
      const auto command = have_command_ ? wrench_command_ : Wrench2D{};
      next = stepInertial(state_, command, inertia_, dt());
      padding = inertialSweepPadding(state_, command, inertia_, dt());
    }
    if (model_ == "reference") {
      next = sampleCircle(initial_, reference_radius_, reference_omega_, clock_.seconds() + dt());
      padding = reference_radius_ * reference_omega_ * reference_omega_ * dt() * dt() / 8.0;
    }
    if (model_ == "trajectory" && trajectory_) {
      next=sampleHeldTrajectory(*trajectory_,clock_.nanoseconds()+clock_.stepDuration().count());
      const double t=(clock_.nanoseconds()-trajectory_->start_ns)*1e-9;
      padding=trajectoryAccelerationBound(trajectory_->curve,t,t+dt())*dt()*dt()/8;
    }
    pending_pose_.reset();
    if (!next.pose.position.allFinite() || !next.velocity.allFinite() ||
      !next.acceleration.allFinite() || !std::isfinite(next.pose.yaw) ||
      !std::isfinite(next.yaw_rate) || !std::isfinite(next.yaw_acceleration) ||
      !std::isfinite(padding))
    {
      blocked_ = true;
      clock_.setPaused(true);
      setStatus("invalid_state");
      setTrajectoryStatus("invalid_state");
      RCLCPP_ERROR(get_logger(), "Non-finite candidate; check physical scales and reset");
      return false;
    }
    if (!sweptDiskIsFree(world_, state_.pose.position, next.pose.position, radius_ + padding)) {
      blocked_ = true;
      clock_.setPaused(true);
      setStatus("collision_predicted");
      setTrajectoryStatus("collision_predicted");
      RCLCPP_WARN(get_logger(), "Swept disk blocked; state/time frozen. Reset to begin a new trial");
      return false;
    }
    interval_start_ = state_;
    interval_start_ns_ = clock_.nanoseconds();
    interval_velocity_ = have_command_ ? velocity_command_ : VelocityCommand{};
    interval_wrench_ = have_command_ ? wrench_command_ : Wrench2D{};
    state_ = next;
    if (single_step) {clock_.singleStep();} else {clock_.advance();}
    if (model_ == "trajectory" && trajectory_) {
      const double t=(clock_.nanoseconds()-trajectory_->start_ns)*1e-9;
      setTrajectoryStatus(t<0 ? "waiting" : t<trajectory_->curve.duration() ? "executing" : "completed");
    }
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
        response->message = blocked_ ? status_ + "; reset required" : status_;
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
        state_ = initialState();
        pending_pose_.reset();
        trajectory_.reset();
        setTrajectoryStatus("idle");
        have_command_ = false;
        velocity_command_ = VelocityCommand{};
        wrench_command_ = Wrench2D{};
        blocked_ = false;
        trail_.clear();
        rememberPosition();
        setStatus("paused");
        interval_start_ = state_;
        interval_start_ns_ = 0;
        interval_velocity_ = {};
        interval_wrench_ = {};
        if (lidar_schedule_) {lidar_schedule_->reset();}
        if (imu_schedule_) {imu_schedule_->reset();}
        lidar_random_.seed(lidar_seed_);
        last_imu_marker_ns_ = -1;
        imu_random_.seed(imu_seed_);
        publishStaticFrames();
        publishState();
        publishMarkers();
        response->success = true;
        response->message = "new paused trial; state, command, trail, sensor grid and RNG reset";
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
      if (frames.first == "map" && !publish_truth_tf_) {continue;}
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
    publishSensors();  // Includes exact acquisition TF and the endpoint/paused heartbeat TF.
  }

  void publishTruth(const State2D & state, std::int64_t stamp)
  {
    const auto time = rclcpp::Time(stamp, RCL_ROS_TIME);
    truth_pub_->publish(truthOdometry(state, time));
    if (!publish_truth_tf_) {return;}
    const auto & pose = state.pose;
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = rclcpp::Time(stamp, RCL_ROS_TIME);
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = pose.position.x();
    tf.transform.translation.y = pose.position.y();
    tf.transform.rotation.z = std::sin(pose.yaw / 2);
    tf.transform.rotation.w = std::cos(pose.yaw / 2);
    broadcaster_->sendTransform(tf);
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
    disk.scale.x = disk.scale.y = radius_ == 0.0 ? 0.04 : 2.0 * radius_;  // Point display only.
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
  State2D interval_start_;
  std::int64_t interval_start_ns_ = 0;
  VelocityCommand interval_velocity_;
  Wrench2D interval_wrench_;
  std::optional<Pose2D> pending_pose_;
  std::optional<TimedTrajectory> trajectory_;
  std::string trajectory_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_status_pub_;
  rclcpp::Subscription<motion2d_interfaces::msg::Trajectory2D>::SharedPtr trajectory_sub_;
  VelocityCommand velocity_command_;
  Wrench2D wrench_command_;
  InertialParameters inertia_;
  LidarConfig lidar_config_;
  ImuNoise imu_noise_;
  double gravity_ = 9.81;
  std::uint32_t imu_seed_ = 6060;
  std::mt19937 imu_random_;
  std::int64_t last_imu_marker_ns_ = -1;
  std::optional<SensorScheduler> lidar_schedule_, imu_schedule_;
  std::uint32_t lidar_seed_ = 4242;
  std::mt19937 lidar_random_;
  double radius_, command_timeout_, speed_max_, yaw_rate_max_;
  double reference_radius_ = 1.0, reference_omega_ = .4;
  std::int64_t command_deadline_ = 0;
  bool blocked_ = false, have_command_ = false;
  bool publish_truth_tf_ = true;  ///< Legacy ch04-06 view; ch07 assigns TF to its adapter.
  std::string model_, status_;
  std::deque<geometry_msgs::msg::Point> trail_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr acceleration_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr imu_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr beam_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
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
