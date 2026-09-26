#pragma once
/** @brief Chain the integration weight and v/a/jerk sample locations. */
inline double movingTimePartial(double weight,double penalty,int count,double dt,double s,
  double gv_dot_a,double ga_dot_j,double gj_dot_snap)
{return weight*(penalty/count+dt*s*(gv_dot_a+ga_dot_j+gj_dot_snap));}
