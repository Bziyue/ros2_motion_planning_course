#include <iostream>
#include "diagonal_step.hpp"

int main()
{
  motion2d::GridConfig geometry; geometry.width = geometry.height = 3;
  motion2d::PlanningGrid grid{geometry,std::vector<std::uint8_t>(9,0),0.};
  grid.blocked[1] = 1;
  if (allowStep(grid,0,0,1,1) || !allowStep(grid,0,0,0,1) || allowStep(grid,0,0,-1,0)) {
    std::cerr << "FAIL ch11-1: destination alone cannot certify a diagonal edge\n"; return 1;
  }
  grid.blocked[1] = 0; grid.blocked[3] = 1;
  if (allowStep(grid,0,0,1,1)) {std::cerr << "FAIL: check both axial neighbours\n"; return 1;}
  grid.blocked[3] = 0;
  if (!allowStep(grid,0,0,1,1)) {std::cerr << "FAIL: free diagonal must be accepted\n"; return 1;}
  std::cout << "PASS ch11-1 axial and diagonal neighbours\n";
}
