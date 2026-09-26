#pragma once
#include "motion2d/trajectory/polynomial.hpp"
/** @brief Compare reference p/v/a at a shared time, each in the same frame. */
inline bool matchingBoundary(const motion2d::TranslationState & old,const motion2d::TranslationState & next) {
  return (old.position-next.position).norm()<=1e-7 && (old.velocity-next.velocity).norm()<=1e-7 &&
    (old.acceleration-next.acceleration).norm()<=1e-7;
}
