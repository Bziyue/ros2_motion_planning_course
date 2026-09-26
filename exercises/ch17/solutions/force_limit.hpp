#pragma once
#include <Eigen/Core>
#include <algorithm>
/** @brief Preserve each admissible component, clip each excessive component. */
inline Eigen::Vector2d boundForce(Eigen::Vector2d requested,double limit) {
 for(int k=0;k<2;++k) requested(k)=std::clamp(requested(k),-limit,limit);return requested;
}
