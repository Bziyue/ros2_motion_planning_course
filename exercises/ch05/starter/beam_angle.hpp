#pragma once

/** @brief Angle of beam i (rad); full turns exclude their duplicate endpoint.
 * @pre N>=2, 0<=i<N, 0<fov<=2*pi; finite angles.
 */
inline double studentBeamAngle(int i, int beams, double angle_min, double fov)
{
  // EXERCISE(ch05-2): full turn divides by N; partial FOV divides by N-1.
  (void)i;
  (void)beams;
  (void)fov;
  return angle_min;
}
