#pragma once
#include "motion2d/sim/robot_model.hpp"
#include <nav_msgs/msg/odometry.hpp>
namespace motion2d {
/** @brief Validate odom/base_link and rotate body twist into odom; acceleration is unobserved zero. */
State2D stateFromOdometry(const nav_msgs::msg::Odometry & message);
/** @brief Reference diagnostic: odom pose, reference_base body twist, no TF publication. */
nav_msgs::msg::Odometry referenceOdometry(const State2D & state,const builtin_interfaces::msg::Time & stamp);
}
