#pragma once
#include <array>
#include "motion2d/sim/robot_model.hpp"
namespace motion2d {
/** @brief Ideal no-slip wheel geometry, metres; appendix B, not the default plant. */
struct WheelGeometry {double radius=.05,track=.4;};
/** @brief Convert left/right wheel rad/s into body-frame (v,0,omega).
 * @details v=r(w_R+w_L)/2, omega=r(w_R-w_L)/track; positive omega is CCW.
 * Validates positive geometry and finite rates. No force, slip or motor dynamics.
 */
VelocityCommand differentialVelocity(double left,double right,const WheelGeometry & geometry={});
/** @brief Inverse ideal kinematics, returns {left,right} wheel rad/s.
 * @param speed Signed forward body speed, m/s.
 * @param yaw_rate Counterclockwise angular speed, rad/s.
 */
std::array<double,2> differentialWheels(double speed,double yaw_rate,const WheelGeometry & geometry={});
/** @brief Exact constant-wheel-rate integration using the existing SE(2) velocity model.
 * @param dt Nonnegative seconds; discontinuous commands do not define finite impulse acceleration.
 * @details This standalone appendix helper does not add a new ROS simulator mode.
 */
State2D stepDifferential(const State2D & state,double left,double right,double dt,const WheelGeometry & geometry={});
}
