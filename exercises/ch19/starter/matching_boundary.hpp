#pragma once
#include "motion2d/trajectory/polynomial.hpp"
/** @brief Compare reference p/v/a at a shared time, each in the same frame. */
inline bool matchingBoundary(const motion2d::TranslationState & old,const motion2d::TranslationState & next) {
  // EXERCISE(ch19-1): position alone cannot preserve velocity or acceleration.
  return (old.position-next.position).norm()<=1e-7;
}
