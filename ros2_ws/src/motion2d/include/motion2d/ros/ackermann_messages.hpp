#pragma once
#include <motion2d_interfaces/msg/ackermann_trajectory2_d.hpp>
#include "motion2d/trajectory/ackermann_trajectory.hpp"
namespace motion2d {
/** @brief Absolute odom trajectory; geometric coefficients remain in unit progress. */
struct TimedAckermannTrajectory {AckermannTrajectory curve;std::int64_t start_ns=0;};
/** @brief Rotate all geometric coefficients, translate only constants. */
AckermannTrajectory transformAckermann(const AckermannTrajectory & curve,const Pose2D & target_from_source);
/** @brief Serialize complete geometric coefficients and physical durations. */
motion2d_interfaces::msg::AckermannTrajectory2D toAckermannMessage(
  const TimedAckermannTrajectory & trajectory,const builtin_interfaces::msg::Time & stamp);
/** @brief Parse an odom curve, checking finite endpoints/joins/durations and size.
 * @details Empty pieces revoke a curve and are handled before parsing. Parsing
 * cannot replace corridor/physical certification or authorize execution.
 */
TimedAckermannTrajectory fromAckermannMessage(const motion2d_interfaces::msg::AckermannTrajectory2D & message);
}
