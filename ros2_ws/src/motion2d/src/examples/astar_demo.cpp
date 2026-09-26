#include <filesystem>
#include <fstream>
#include <iostream>
#include "motion2d/planning/astar.hpp"

/** @brief Hand-specified grid benchmark, deliberately isolated from mapping uncertainty. */
int main(int argc, char ** argv)
{
  const std::filesystem::path output = argc > 1 ? argv[1] : "tmp/ch11_astar";
  std::filesystem::create_directories(output);
  motion2d::GridConfig geometry; geometry.width = 41; geometry.height = 31;
  geometry.resolution = .1; geometry.origin = {0,0};
  std::vector<std::int8_t> occupancy(41*31,0);
  for (int y = 0; y < 31; ++y) {if (y < 12 || y > 18) {occupancy[y*41+20] = 100;}}
  std::ofstream summary(output / "ch11_astar.csv");
  summary << "radius,neighbors,status,grid_cost,expanded,raw_points,simplified_points\n";
  const double radii[]{0., .15, .3};
  for (int r = 0; r < 3; ++r) {
    const auto grid = motion2d::inflateGrid(geometry,occupancy,{radii[r],0.,35,true});
    std::ofstream map(output / ("grid_"+std::to_string(r)+".csv"));
    map << "x,y,source,blocked\n";
    for (int y = 0; y < 31; ++y) {
      for (int x = 0; x < 41; ++x) {
        map << x << ',' << y << ',' << static_cast<int>(occupancy[y*41+x]) << ','
          << static_cast<int>(grid.blocked[y*41+x]) << '\n';
      }
    }
    for (bool diagonal : {false,true}) {
      const auto result = motion2d::astar(grid,{.75,.75},{3.25,2.25},diagonal);
      const auto simplified = motion2d::simplifyPath(grid,result.path);
      summary << radii[r] << ',' << (diagonal ? 8 : 4) << ',' << result.status << ','
        << result.grid_cost << ',' << result.expanded << ',' << result.path.size() << ',' << simplified.size() << '\n';
      std::ofstream path(output / ("path_"+std::to_string(r)+"_"+(diagonal ? "8" : "4")+".csv"));
      path << "type,x,y\n";
      for (const auto & p : result.path) {path << "raw," << p.x() << ',' << p.y() << '\n';}
      for (const auto & p : simplified) {path << "simplified," << p.x() << ',' << p.y() << '\n';}
    }
  }
  std::cout << "Wrote supplied-grid planning experiment to " << output << '\n';
}
