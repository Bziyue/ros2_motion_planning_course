#include "motion2d/trajectory/trajectory_optimizer.hpp"
#include "motion2d/trajectory/bezier_bounds.hpp"
#include <iostream>
using namespace motion2d;
/** @brief Same endpoint problem with two masses and both trajectory backends. */
int main() {
  std::cout<<"backend,mass,duration,force_bound,force_rate_bound,certified,status\n";
  for(bool spline:{false,true})for(double mass:{1.,3.}) {
    TranslationState a,b;b.position={2,.3};TrajectoryOptimizationConfig c;
    c.optimize_waypoints=false;c.cost.energy_weight=.01;c.solver.max_iterations=300;c.solver.gradient_tolerance=1e-5;
    OmniDynamicLimits d;d.parameters.mass=mass;d.parameters.linear_drag=.3;d.force=1;d.force_rate=2;c.limits.dynamics=d;
    d.force*=.85;d.force_rate*=.85;c.cost.dynamics=d;
    const auto r=spline?optimizeSpline(a,b,{}, {2},c):optimizeMinco(a,b,{}, {2},c);
    if(!r.curve) return 1;const auto cert=certifyForce(*r.curve,*c.limits.dynamics);
    std::cout<<(spline?"spline":"minco")<<','<<mass<<','<<r.curve->duration()<<','<<cert.force_bound<<','<<cert.force_rate_bound<<','<<cert.certified<<','<<r.solver.status<<'\n';
    if(!r.solver.converged() || !cert.certified) return 2;
  }
}
