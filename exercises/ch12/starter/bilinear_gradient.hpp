#pragma once
#include <array>
#include <Eigen/Core>
/** @brief Bilinear field's gradient with respect to metre coordinates.
 * @param d Corner values in metres, ordered 00,10,01,11.
 * @param u X fraction within the interpolation patch, in [0,1].
 * @param v Y fraction in [0,1].
 * @param resolution Centre spacing in metres, positive.
 */
inline Eigen::Vector2d bilinearGradient(const std::array<double,4> & d, double u, double v, double resolution)
{
  // EXERCISE(ch12-1): differentiate the weights and convert cell units to metres.
  (void)d; (void)u; (void)v; (void)resolution;
  return Eigen::Vector2d::Zero();
}
