#pragma once
/** @brief Positive-time chain rule; durations are seconds. */
inline double timeGradient(double dJdT,double T,double Tmin) {return dJdT*(T-Tmin);}
