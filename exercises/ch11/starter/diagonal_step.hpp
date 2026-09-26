#pragma once
#include "motion2d/planning/grid.hpp"

/** @brief Whether an adjacent step is valid in an 8-connected configuration grid.
 * @pre dx/dy are in {-1,0,1} and not both zero; current cell is free.
 */
inline bool allowStep(const motion2d::PlanningGrid & grid, int x, int y, int dx, int dy)
{
  // EXERCISE(ch11-1): destination and, for diagonals, both axial neighbours must be free.
  return grid.freeCell(x+dx,y+dy);
}
