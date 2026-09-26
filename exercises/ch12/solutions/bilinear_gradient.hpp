#pragma once
#include <array>
#include <Eigen/Core>
/** @brief Bilinear gradient in m/m; d is ordered 00,10,01,11, resolution is in m. */
inline Eigen::Vector2d bilinearGradient(const std::array<double,4> & d, double u, double v, double resolution)
{
  return {((1-v)*(d[1]-d[0])+v*(d[3]-d[2]))/resolution,
          ((1-u)*(d[2]-d[0])+u*(d[3]-d[1]))/resolution};
}
