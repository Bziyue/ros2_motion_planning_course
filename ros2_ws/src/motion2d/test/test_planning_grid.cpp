#include <gtest/gtest.h>
#include <queue>
#include <random>
#include "motion2d/planning/astar.hpp"
using namespace motion2d;

namespace
{
PlanningGrid blank(int width = 9, int height = 9)
{
  GridConfig geometry; geometry.width = width; geometry.height = height;
  geometry.resolution = 1.; geometry.origin = {0,0};
  return {geometry, std::vector<std::uint8_t>(width*height,0), 0};
}

// Independent uniform-cost search: no heuristic, no shared neighbour helper.
double dijkstra(const PlanningGrid & grid, int start, int goal)
{
  const int width = grid.geometry.width;
  std::vector<double> costs(grid.blocked.size(), INFINITY); costs[start] = 0;
  using Entry = std::pair<double,int>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
  queue.emplace(0., start);
  const int offsets[][2]{{-1,0},{1,0},{0,-1},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}};
  while (!queue.empty()) {
    const auto [cost, cell] = queue.top(); queue.pop();
    if (cost > costs[cell]) {continue;}
    if (cell == goal) {return cost;}
    for (auto & step : offsets) {
      const int x = cell%width, y = cell/width, nx = x+step[0], ny = y+step[1];
      if (!grid.freeCell(nx,ny)) {continue;}
      if (step[0] && step[1] && (!grid.freeCell(nx,y) || !grid.freeCell(x,ny))) {continue;}
      const int next = ny*width+nx;
      const double new_cost = cost + (step[0] && step[1] ? std::sqrt(2.) : 1.)*grid.geometry.resolution;
      if (new_cost < costs[next]) {costs[next] = new_cost; queue.emplace(new_cost,next);}
    }
  }
  return INFINITY;
}
}

TEST(Inflation, WholeSquareDistanceUnknownAndExterior)
{
  auto grid = blank();
  std::vector<std::int8_t> values(81,0); values[4*9+4] = 100;
  auto inflated = inflateGrid(grid.geometry, values, {.4,0,35,true});
  EXPECT_FALSE(inflated.freeCell(5,4)); // Shared edge: minimum square distance zero.
  EXPECT_TRUE(inflated.freeCell(6,4));  // Squares have a 1 m gap.
  EXPECT_FALSE(inflated.freeCell(0,4)); // Exterior touches the whole boundary cell.
  EXPECT_TRUE(inflated.freeCell(1,4));
  values[2*9+2] = -1;
  EXPECT_FALSE(inflateGrid(grid.geometry,values,{0,0,35,true}).freeCell(2,2));
  EXPECT_TRUE(inflateGrid(grid.geometry,values,{0,0,35,false}).freeCell(2,2));
  values[2*9+2] = 50;
  EXPECT_FALSE(inflateGrid(grid.geometry,values,{0,0,35,false}).freeCell(2,2));
}

TEST(Inflation, RadiusClosesANarrowDoor)
{
  auto grid = blank(21,15);
  grid.geometry.resolution = .1;
  std::vector<std::int8_t> cells(315,0);
  for (int y = 0; y < 15; ++y) {if (y < 4 || y > 10) {cells[y*21+10] = 100;}}
  const Eigen::Vector2d start(.55,.75), goal(1.55,.75);
  auto small = inflateGrid(grid.geometry,cells,{.15,0,35,true});
  auto large = inflateGrid(grid.geometry,cells,{.3,0,35,true});
  EXPECT_TRUE(astar(small,start,goal).success);
  EXPECT_EQ(astar(large,start,goal).status, "unreachable");
}

TEST(Astar, KnownFourEightCostsExactEndpointsAndSameCell)
{
  const auto grid = blank();
  auto four = astar(grid,{1.5,1.5},{4.5,4.5},false);
  auto eight = astar(grid,{1.5,1.5},{4.5,4.5},true);
  ASSERT_TRUE(four.success); ASSERT_TRUE(eight.success);
  EXPECT_NEAR(four.grid_cost,6.,1e-12); EXPECT_NEAR(eight.grid_cost,3*std::sqrt(2.),1e-12);
  const auto local = astar(grid,{2.2,2.3},{2.7,2.8});
  ASSERT_EQ(local.path.size(),2U); EXPECT_EQ(local.path.front(),Eigen::Vector2d(2.2,2.3));
  EXPECT_EQ(local.path.back(),Eigen::Vector2d(2.7,2.8));
  EXPECT_EQ(astar(grid,{2.2,2.3},{2.2,2.3}).path.size(),1U);
}

TEST(Astar, CannotCutDiagonalCornerAndFailureClearsPath)
{
  auto grid = blank(3,3); grid.blocked[1] = grid.blocked[3] = 1;
  auto result = astar(grid,{.5,.5},{1.5,1.5});
  EXPECT_EQ(result.status,"unreachable"); EXPECT_TRUE(result.path.empty());
  EXPECT_EQ(astar(grid,{-1,.5},{1.5,1.5}).status,"invalid_start");
  EXPECT_EQ(astar(grid,{.5,.5},{1.5,.5}).status,"invalid_goal");
  EXPECT_EQ(astar(grid,{1.,0.5},{1.5,1.5}).status,"invalid_start");
}

TEST(Astar, SeededCasesMatchIndependentUniformCostSearch)
{
  std::mt19937 random(1111);
  for (int trial = 0; trial < 30; ++trial) {
    auto grid = blank(12,10);
    for (auto & cell : grid.blocked) {cell = random()%5 == 0;}
    grid.blocked[0] = grid.blocked.back() = 0;
    const double reference = dijkstra(grid,0,119);
    const auto result = astar(grid,grid.cellCenter(0),grid.cellCenter(119));
    EXPECT_EQ(result.success,std::isfinite(reference));
    if (result.success) {
      EXPECT_NEAR(result.grid_cost,reference,1e-11);
      for (std::size_t i = 1; i < result.path.size(); ++i) {EXPECT_TRUE(segmentIsFree(grid,result.path[i-1],result.path[i]));}
    }
  }
}

TEST(PathSimplifier, AnalyticSegmentChecksCornerEdgesAndThinObstacles)
{
  auto grid = blank(6,6); grid.blocked[2*6+2] = 1;
  EXPECT_FALSE(segmentIsFree(grid,{.5,.5},{4.5,4.5}));
  EXPECT_FALSE(segmentIsFree(grid,{1.,3.},{4.,3.})); // Lies on the occupied square's upper edge.
  EXPECT_FALSE(segmentIsFree(grid,{1.,4.},{4.,1.})); // Cuts through the square.
  EXPECT_FALSE(segmentIsFree(grid,{1.,2.},{3.,4.})); // Touches only corner (2,3).
  EXPECT_TRUE(segmentIsFree(grid,{1.,3.},{3.,5.}));
}

TEST(PathSimplifier, PreservesSafeEndpointsAndRejectsUnsafeInput)
{
  auto grid = blank(); grid.blocked[4*9+4] = 1;
  const auto path = astar(grid,{1.5,1.5},{7.5,7.5});
  ASSERT_TRUE(path.success);
  const auto simplified = simplifyPath(grid,path.path);
  EXPECT_EQ(simplified.front(),path.path.front()); EXPECT_EQ(simplified.back(),path.path.back());
  EXPECT_LT(simplified.size(),path.path.size()); EXPECT_GT(simplified.size(),2U);
  for (std::size_t i = 1; i < simplified.size(); ++i) {EXPECT_TRUE(segmentIsFree(grid,simplified[i-1],simplified[i]));}
  EXPECT_THROW(simplifyPath(grid,{{1.5,1.5},{7.5,7.5}}),std::invalid_argument);
}
