#include <filesystem>
#include <cmath>
#include <fstream>
#include <iostream>
#include "motion2d/planning/astar.hpp"
#include "motion2d/planning/corridor.hpp"

using namespace motion2d;
/** @brief Supplied-grid comparison of local rectangles and validated convex hulls. */
int main(int argc, char ** argv)
{
  const std::filesystem::path out = argc > 1 ? argv[1] : "tmp/ch13_corridor";
  std::filesystem::create_directories(out);
  GridConfig g; g.width = 61; g.height = 41; g.resolution = .1; g.origin.setZero();
  std::vector<std::int8_t> raw(g.width*g.height,0);
  for (int y = 0; y < 29; ++y) {raw[y*g.width+30] = 100;}
  const auto grid = inflateGrid(g,raw,{.15,0,35,true});
  const auto path = astar(grid,{.75,.75},{5.35,.75});
  if (!path.success) {std::cerr << path.status << '\n'; return 1;}
  std::ofstream cells(out/"grid.csv"); cells << "x,y,blocked\n";
  for (int i = 0; i < int(raw.size()); ++i) {
    const auto p = grid.cellCenter(i); cells << p.x() << ',' << p.y() << ',' << int(grid.blocked[i]) << '\n';
  }
  std::ofstream route(out/"astar.csv"); route << "x,y\n";
  for (const auto & p : path.path) {route << p.x() << ',' << p.y() << '\n';}
  std::ofstream summary(out/"summary.csv"); summary << "convex,regions,vertices,waypoints,min_overlap_area_m2\n";
  for (int mode = 0; mode < 2; ++mode) {
    const auto result = buildCorridor(grid,path.path,mode,.6);
    if (!result.success) {std::cerr << result.status << '\n'; return 2;}
    std::ofstream regions(out/("regions_"+std::to_string(mode)+".csv")); regions << "region,x,y\n";
    std::ofstream points(out/("route_"+std::to_string(mode)+".csv")); points << "x,y\n";
    int vertices = 0; double minimum = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < result.regions.size(); ++i) {
      for (const auto & p : result.regions[i].polygon.vertices) {regions << i << ',' << p.x() << ',' << p.y() << '\n'; ++vertices;}
      if (i) {
        const auto overlap = intersectRegions(result.regions[i-1],result.regions[i]);
        double twice_area = 0;
        for (std::size_t j = 0; j < overlap.vertices.size(); ++j) {
          const auto & a = overlap.vertices[j]; const auto & b = overlap.vertices[(j+1)%overlap.vertices.size()];
          twice_area += a.x()*b.y()-a.y()*b.x();
        }
        minimum = std::min(minimum,.5*std::abs(twice_area));
      }
    }
    for (const auto & p : result.waypoints) {points << p.x() << ',' << p.y() << '\n';}
    summary << mode << ',' << result.regions.size() << ',' << vertices << ',' << result.waypoints.size() << ',' << minimum << '\n';
    std::cout << "convex=" << mode << " regions=" << result.regions.size() << " vertices=" << vertices
              << " min_overlap_area=" << minimum << " m^2\n";
  }
}
