#include "motion2d/dynamics/omni_flatness.hpp"
namespace motion2d
{
// omni_forward_begin
OmniFlatOutput omniForward(const OmniFlatInput & x,const InertialParameters & p)
{
  return {p.mass*x.acceleration+p.linear_drag*x.velocity,
    p.mass*x.jerk+p.linear_drag*x.acceleration,
    p.inertia_z*x.yaw_acceleration+p.angular_drag*x.yaw_rate};
}
// omni_forward_end
// omni_backward_begin
OmniFlatInput omniBackward(const OmniFlatOutput & g,const InertialParameters & p)
{
  return {p.linear_drag*g.force,
    p.mass*g.force+p.linear_drag*g.force_rate,
    p.mass*g.force_rate,p.angular_drag*g.torque,p.inertia_z*g.torque};
}
// omni_backward_end
}  // namespace motion2d
