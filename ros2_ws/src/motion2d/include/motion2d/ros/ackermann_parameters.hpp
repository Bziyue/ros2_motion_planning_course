#pragma once
#include <rclcpp/rclcpp.hpp>
#include "motion2d/sim/ackermann_model.hpp"
namespace motion2d {
/** @brief Read the same explicit bicycle constants at simulator/control boundaries. */
inline AckermannParameters readAckermannParameters(rclcpp::Node & node) {
  AckermannParameters p;
  p.wheelbase=node.declare_parameter("ackermann.wheelbase",.3);
  p.mass=node.declare_parameter("ackermann.mass",1.);
  p.linear_drag=node.declare_parameter("ackermann.linear_drag",.15);
  p.force_max=node.declare_parameter("ackermann.force_max",2.);
  p.steering_max=node.declare_parameter("ackermann.steering_max",.6);
  p.steering_rate_max=node.declare_parameter("ackermann.steering_rate_max",1.);
  validateAckermannParameters(p);return p;
}
}
