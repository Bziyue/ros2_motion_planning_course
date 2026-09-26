#pragma once
#include <rclcpp/rclcpp.hpp>
#include "motion2d/trajectory/trajectory_optimizer.hpp"
namespace motion2d {
/** @brief Read optional force planning settings once; shared by planner/navigation.
 * @details The launch file must supply the same mass/drag as the plant. Cost targets
 * are 85% of validation limits, reserving room for soft penalties and tracking.
 */
inline void readPlanningDynamics(rclcpp::Node & node,TrajectoryOptimizationConfig & config)
{
  if(!node.declare_parameter("planning.dynamics.enabled",false)) return;
  OmniDynamicLimits d;
  d.parameters.mass=node.declare_parameter("planning.dynamics.mass",1.);
  d.parameters.linear_drag=node.declare_parameter("planning.dynamics.linear_drag",.15);
  d.force=node.declare_parameter("planning.dynamics.force_max",1.5);
  d.force_rate=node.declare_parameter("planning.dynamics.force_rate_max",4.);
  validateDynamicLimits(d);config.limits.dynamics=d;
  d.force*=.85;d.force_rate*=.85;config.cost.dynamics=d;
}
}
