#include <gtest/gtest.h>
#include "motion2d/ros/control_messages.hpp"
using namespace motion2d;
TEST(ControlMessages, BodyVelocityRotatesIntoOdom) {
  nav_msgs::msg::Odometry m;m.header.frame_id="odom";m.child_frame_id="base_link";
  m.pose.pose.orientation.z=std::sin(kPi/4);m.pose.pose.orientation.w=std::cos(kPi/4);
  m.twist.twist.linear.x=.2;m.twist.twist.linear.y=.3;m.twist.twist.angular.z=.4;
  auto s=stateFromOdometry(m);EXPECT_LT((s.velocity-Eigen::Vector2d(-.3,.2)).norm(),1e-12);EXPECT_EQ(s.yaw_rate,.4);
  m.twist.twist.linear.z=1;EXPECT_THROW(stateFromOdometry(m),std::invalid_argument);
}
TEST(ControlMessages, ReferenceTwistUsesReferenceBodyFrame) {
  State2D s;s.pose={{3,-2},kPi/2};s.velocity={1,2};s.yaw_rate=.4;
  const auto m=referenceOdometry(s,builtin_interfaces::msg::Time{});EXPECT_EQ(m.header.frame_id,"odom");EXPECT_EQ(m.child_frame_id,"reference_base");
  EXPECT_NEAR(m.twist.twist.linear.x,2,1e-12);EXPECT_NEAR(m.twist.twist.linear.y,-1,1e-12);EXPECT_EQ(m.twist.twist.angular.z,.4);
}
