#pragma once
/** @brief The inertial part scales as T^-2 and drag as T^-1. */
inline double forceTimeGradient(double mass,double drag,double n,double along,double rate,double accel,double time)
{return -(2*mass*(along*rate*rate+n*accel)+drag*n*rate)/time;}
