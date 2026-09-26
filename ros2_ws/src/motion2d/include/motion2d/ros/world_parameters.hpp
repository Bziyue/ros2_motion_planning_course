#pragma once
#include <limits>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include "motion2d/sim/world.hpp"

namespace motion2d
{
/** @brief Read the same world parameters in the simulator and visualization node.
 *  @details Use the YAML wildcard node selector so both nodes see the same map.
 */
inline WorldConfig readWorldConfig(rclcpp::Node & node)
{
  WorldConfig c;
  const auto seed = node.declare_parameter<std::int64_t>("seed", 42);
  if (seed < 0 || seed > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("seed must fit an unsigned 32-bit integer");
  }
  c.seed = static_cast<std::uint32_t>(seed);
  c.width = node.declare_parameter("width", 20.0);
  c.height = node.declare_parameter("height", 20.0);
  c.circle_count = node.declare_parameter("circle_count", 8);
  c.polygon_count = node.declare_parameter("polygon_count", 8);
  c.size_min = node.declare_parameter("size_min", 0.35);
  c.size_max = node.declare_parameter("size_max", 1.2);
  c.polygon_samples = node.declare_parameter("polygon_samples", 8);
  c.start.x() = node.declare_parameter("x", -8.0);
  c.start.y() = node.declare_parameter("y", -8.0);
  c.goal.x() = node.declare_parameter("goal_x", 8.0);
  c.goal.y() = node.declare_parameter("goal_y", 8.0);
  c.robot_radius = node.declare_parameter("radius", 0.2);
  c.margin = node.declare_parameter("margin", 0.05);
  c.require_connected = node.declare_parameter("require_connected", true);
  c.check_resolution = node.declare_parameter("check_resolution", 0.25);
  return c;
}
}  // namespace motion2d
