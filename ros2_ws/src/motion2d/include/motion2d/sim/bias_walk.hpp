#pragma once
namespace motion2d {
/** @brief One scalar Brownian bias step, appendix C; independent of per-sample white noise.
 * @param bias Current bias, e.g. rad/s for a gyroscope axis.
 * @param density Bias random-walk density, bias units divided by sqrt(s); nonnegative.
 * @param dt Nonnegative elapsed seconds, not the nominal sensor period after a gap.
 * @param unit_noise One caller-owned N(0,1) draw; independent across axes and steps.
 * @return Updated bias in the same units. A zero dt or density leaves bias unchanged.
 * @details b_next=b+density*sqrt(dt)*unit_noise. Finite inputs are required.
 * This opt-in standalone helper does not change the main simulator's white-noise model.
 */
double stepBiasWalk(double bias,double density,double dt,double unit_noise);
}
