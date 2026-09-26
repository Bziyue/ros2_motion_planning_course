#include "motion2d/sim/bias_walk.hpp"
#include <cmath>
#include <stdexcept>
namespace motion2d {
// bias_walk_begin
double stepBiasWalk(double bias,double density,double dt,double unit_noise) {
  if(!std::isfinite(bias) || !std::isfinite(density) || density<0 ||
     !std::isfinite(dt) || dt<0 || !std::isfinite(unit_noise))
    throw std::invalid_argument("Bias walk needs finite inputs and nonnegative density/dt");
  return bias+density*std::sqrt(dt)*unit_noise;
}
// bias_walk_end
}
