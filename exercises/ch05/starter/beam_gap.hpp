#pragma once

/** @brief Chord between adjacent beams at the same range, in metres.
 * @pre distance>=0 (m), 0<angle_step<=pi (rad), finite inputs.
 */
inline double studentBeamGap(double distance, double angle_step)
{
  // EXERCISE(ch05-4): use half of the isosceles triangle between two rays.
  (void)distance;
  (void)angle_step;
  return 0;
}
