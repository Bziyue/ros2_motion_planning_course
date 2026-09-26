#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include "motion2d/ros/trajectory_messages.hpp"
#include "motion2d/ros/mapping_messages.hpp"
#include "motion2d/trajectory/quintic.hpp"

namespace motion2d
{
/** @brief Explicit one-shot conversion of an observed corridor route to a timed curve.
 * @details Trigger service is deliberate: clicking a goal plans but does not silently
 * start a moving robot. Reference is frozen in odom at publication. Online replanning
 * and handover belong to chapter 19. This node never reads simulation truth.
 */
class TrajectoryNode : public rclcpp::Node
{
public:
  TrajectoryNode() : Node("trajectory"), buffer_(get_clock()), listener_(buffer_,*this,false)
  {
    speed_=declare_parameter("trajectory.nominal_speed",.4);
    lead_=declare_parameter("trajectory.start_lead",.25);
    if(!std::isfinite(speed_) || speed_<=0 || !std::isfinite(lead_) || lead_<=0)
    {throw std::invalid_argument("Positive trajectory speed and lead required");}
    pub_=create_publisher<motion2d_interfaces::msg::Trajectory2D>("/plan/trajectory",1);
    preview_=create_publisher<nav_msgs::msg::Path>("/plan/trajectory_path",rclcpp::QoS(1).transient_local());
    path_sub_=create_subscription<nav_msgs::msg::Path>("/plan/corridor_path",rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::Path::SharedPtr m){path_=m;});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odometry",100,
      [this](nav_msgs::msg::Odometry::SharedPtr m){
        if(odom_ && rclcpp::Time(m->header.stamp)<rclcpp::Time(odom_->header.stamp)) {
          path_.reset(); nav_msgs::msg::Path empty; empty.header.frame_id="odom";
          empty.header.stamp=m->header.stamp; preview_->publish(empty);
        }
        odom_=m;
      });
    execute_=create_service<std_srvs::srv::Trigger>("/trajectory/execute",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response){
        try {publish(); response->success=true; response->message="published; check /sim/trajectory_status for acceptance";}
        catch(const std::exception & e){response->success=false; response->message=e.what();}
      });
  }
private:
  void publish()
  {
    if(!path_ || path_->poses.size()<2 || path_->header.frame_id!="map" || !odom_)
    {throw std::invalid_argument("Need a current map-frame corridor route and odometry");}
    const auto start=poseFromOdometry(*odom_);
    const auto transform=buffer_.lookupTransform("odom","map",rclcpp::Time(odom_->header.stamp));
    nav_msgs::msg::Odometry proxy; proxy.header.frame_id="odom"; proxy.child_frame_id="base_link";
    proxy.pose.pose.position.x=transform.transform.translation.x;
    proxy.pose.pose.position.y=transform.transform.translation.y;
    proxy.pose.pose.position.z=transform.transform.translation.z;
    proxy.pose.pose.orientation=transform.transform.rotation;
    const auto odom_from_map=poseFromOdometry(proxy);
    // trajectory_publish_begin
    std::vector<Eigen::Vector2d> points;
    for(const auto & pose : path_->poses) {
      const auto & p=pose.pose.position;
      if(pose.header.frame_id!="map" || !std::isfinite(p.x) || !std::isfinite(p.y) || p.z!=0)
      {throw std::invalid_argument("Expected finite planar path points in map");}
      points.push_back(transformPoint(odom_from_map,{p.x,p.y}));
    }
    if((points.front()-start.position).norm()>1e-7)
    {throw std::invalid_argument("Route start differs from current pose; wait for replanning");}
    const auto stamp=now();
    TimedTrajectory command{stopAtWaypoints(points,speed_,.5),
      stamp.nanoseconds()+std::llround(lead_*1e9),start.yaw};
    pub_->publish(toTrajectoryMessage(command,stamp));
    // trajectory_publish_end
    nav_msgs::msg::Path preview; preview.header.frame_id="odom"; preview.header.stamp=stamp;
    const int count=std::max(1,static_cast<int>(std::ceil(command.curve.duration()/.05)));
    for(int i=0;i<=count;++i) {
      const double t=command.curve.duration()*i/count; const auto value=command.curve.sample(t);
      geometry_msgs::msg::PoseStamped p; p.header.frame_id="odom";
      p.header.stamp=rclcpp::Time(command.start_ns+std::llround(t*1e9),RCL_ROS_TIME);
      p.pose.position.x=value.position.x(); p.pose.position.y=value.position.y();
      p.pose.orientation.z=std::sin(command.yaw/2); p.pose.orientation.w=std::cos(command.yaw/2);
      preview.poses.push_back(p);
    }
    preview_->publish(preview);
  }
  double speed_,lead_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  nav_msgs::msg::Path::SharedPtr path_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<motion2d_interfaces::msg::Trajectory2D>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr preview_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_;
};
}  // namespace motion2d
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<motion2d::TrajectoryNode>());}
  catch(const std::exception & e) {RCLCPP_ERROR(rclcpp::get_logger("trajectory"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
