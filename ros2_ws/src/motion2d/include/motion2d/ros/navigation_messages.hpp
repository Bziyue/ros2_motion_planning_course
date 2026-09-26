#pragma once
#include "motion2d/trajectory/reference_schedule.hpp"
#include <motion2d_interfaces/msg/navigation_reference.hpp>
namespace motion2d {
/** @brief Validate odom motion, finite CCW planar regions, exact piece count and map time.
 * @details Geometry decoding alone cannot certify regions against a map that the
 * controller does not own; the planner is responsible for configuration-space validation.
 */
NavigationReference fromNavigationMessage(const motion2d_interfaces::msg::NavigationReference & message);
/** @brief Serialize curve and double-precision region vertices atomically. */
motion2d_interfaces::msg::NavigationReference toNavigationMessage(const NavigationReference & reference,
  const builtin_interfaces::msg::Time & published);
}
