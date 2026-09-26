#pragma once
/** @brief Direct T partial of one trapezoidal penalty term at t=s*T. */
inline double movingTimePartial(double weight,double penalty,int count,double dt,double s,
  double gv_dot_a,double ga_dot_j,double gj_dot_snap)
{
  // EXERCISE(ch15-3): include integration weight and moving sample location.
  (void)weight;(void)penalty;(void)count;(void)dt;(void)s;
  (void)gv_dot_a;(void)ga_dot_j;(void)gj_dot_snap;return 0;
}
