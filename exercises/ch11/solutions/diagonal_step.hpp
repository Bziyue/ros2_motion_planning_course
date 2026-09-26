#pragma once
#include "motion2d/planning/grid.hpp"

/** @brief Reject corner cutting even when the diagonal destination itself is free. */
inline bool allowStep(const motion2d::PlanningGrid & grid, int x, int y, int dx, int dy)
{
  return grid.freeCell(x+dx,y+dy) &&
    (dx == 0 || dy == 0 || (grid.freeCell(x+dx,y) && grid.freeCell(x,y+dy)));
}
