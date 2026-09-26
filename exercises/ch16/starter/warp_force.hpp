#pragma once
/** @brief dF/dT at fixed normalized time and fixed geometric path. */
inline double forceTimeGradient(double mass,double drag,double n,double along,double rate,double accel,double time)
{
  // EXERCISE(ch16-3): rate=h'/T, accel=h''/T^2; n and along are geometric.
  (void)mass;(void)drag;(void)n;(void)along;(void)rate;(void)accel;(void)time;return 0;
}
