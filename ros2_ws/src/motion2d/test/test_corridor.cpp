#include <gtest/gtest.h>
#include "motion2d/planning/corridor.hpp"
#include "motion2d/planning/astar.hpp"

using namespace motion2d;
namespace
{
PlanningGrid openGrid(int size = 12)
{
  GridConfig g; g.width = g.height = size; g.resolution = 1; g.origin.setZero();
  PlanningGrid grid{g,std::vector<std::uint8_t>(size*size,0),0};
  for (int i = 0; i < size; ++i) {
    grid.blocked[i] = grid.blocked[(size-1)*size+i] = grid.blocked[i*size] = grid.blocked[i*size+size-1] = 1;
  }
  return grid;
}
ConvexRegion box(double x0, double y0, double x1, double y1)
{
  return makeRegion({{{x0,y0},{x1,y0},{x1,y1},{x0,y1}}});
}
}

TEST(Corridor, HalfspacesAndOverlapHaveGeometricUnits)
{
  const auto a = box(1,2,4,5), b = box(3,4,6,7);
  EXPECT_TRUE(a.contains({1,2})); EXPECT_FALSE(a.contains({.999,2}));
  for (int i = 0; i < a.A.rows(); ++i) {EXPECT_NEAR(a.A.row(i).norm(),1,1e-12);}
  const auto overlap = makeRegion(intersectRegions(a,b));
  EXPECT_TRUE(overlap.contains({3.5,4.5})); EXPECT_FALSE(overlap.contains({2.5,4.5}));
  EXPECT_EQ(overlap.polygon.vertices.size(),4u);
  EXPECT_TRUE(intersectRegions(a,box(4,2,6,5)).vertices.empty()); // Only shared edge.
  EXPECT_TRUE(intersectRegions(a,box(8,8,9,9)).vertices.empty());
  EXPECT_THROW(makeRegion({{{0,0},{0,1},{1,0}}}),std::invalid_argument); // Clockwise.
}

TEST(Corridor, SafeVerticesDoNotCertifyInteriorOrEdgeContact)
{
  auto grid = openGrid(); grid.blocked[5*12+5] = 1;
  const auto enclosing = box(2.1,2.1,8.9,8.9);
  for (const auto & p : enclosing.polygon.vertices) {EXPECT_TRUE(segmentIsFree(grid,p,p));}
  EXPECT_FALSE(regionIsFree(grid,enclosing));
  EXPECT_FALSE(regionIsFree(grid,box(2.1,2.1,5,5.9))); // Edge contact.
  EXPECT_FALSE(regionIsFree(grid,makeRegion({{{4,4},{6,4},{4,6}}}))); // Corner contact (5,5).
  EXPECT_TRUE(regionIsFree(grid,box(2.1,2.1,4.9,4.9)));
  EXPECT_FALSE(regionIsFree(grid,box(-.1,2,3,4)));
}

TEST(Corridor, SafeLocalRectanglesMergeToNonrectangularConvexHull)
{
  const auto grid = openGrid();
  const std::vector<Eigen::Vector2d> path{{2.5,2.5},{3.5,3.5},{4.5,4.5},{5.5,5.5}};
  const auto rectangles = buildCorridor(grid,path,false,0);
  const auto convex = buildCorridor(grid,path,true,0);
  ASSERT_TRUE(rectangles.success); ASSERT_TRUE(convex.success);
  EXPECT_GT(rectangles.regions.size(),1u); ASSERT_EQ(convex.regions.size(),1u);
  EXPECT_EQ(convex.regions[0].polygon.vertices.size(),6u);
  EXPECT_TRUE(regionIsFree(grid,convex.regions[0]));
  EXPECT_EQ(convex.waypoints.front(),path.front()); EXPECT_EQ(convex.waypoints.back(),path.back());
}

TEST(Corridor, BentRouteRejectsUnsafeHullsAndAssignsEverySegment)
{
  for (const double resolution : {1., .1}) {
  auto grid = openGrid(14); grid.geometry.resolution = resolution;
  for (int y = 1; y <= 9; ++y) {grid.blocked[y*14+7] = 1;}
  const auto path = astar(grid,resolution*Eigen::Vector2d(3.5,3.5),resolution*Eigen::Vector2d(10.5,3.5)); ASSERT_TRUE(path.success);
  const auto corridors = buildCorridor(grid,path.path,true,2*resolution);
  ASSERT_TRUE(corridors.success) << corridors.status;
  EXPECT_GT(corridors.regions.size(),1u);
  ASSERT_EQ(corridors.waypoints.size(),corridors.regions.size()+1);
  for (std::size_t i = 0; i < corridors.regions.size(); ++i) {
    const auto & r = corridors.regions[i];
    EXPECT_TRUE(regionIsFree(grid,r));
    EXPECT_TRUE(r.contains(corridors.waypoints[i])); EXPECT_TRUE(r.contains(corridors.waypoints[i+1]));
    EXPECT_TRUE(segmentIsFree(grid,corridors.waypoints[i],corridors.waypoints[i+1]));
    if (i) {EXPECT_FALSE(intersectRegions(corridors.regions[i-1],r).vertices.empty());}
  }
  }
}

TEST(Corridor, FailuresClearResultsAndGridBoundaryEndpointsWork)
{
  auto grid = openGrid();
  const auto valid = buildCorridor(grid,{{3,3},{4.5,4.5}},true,.5); ASSERT_TRUE(valid.success);
  EXPECT_TRUE(valid.regions.front().contains({3,3}));
  EXPECT_TRUE(buildCorridor(grid,{{3,3}}).success);
  EXPECT_EQ(buildCorridor(grid,{}).status,"empty_path");
  EXPECT_EQ(buildCorridor(grid,{{3,3}},true,-1).status,"invalid_extension");
  const auto failure = buildCorridor(grid,{{3,3},{0,0}});
  EXPECT_FALSE(failure.success); EXPECT_TRUE(failure.regions.empty()); EXPECT_TRUE(failure.waypoints.empty());
  grid.blocked[3*12+7] = 1; // Off a diagonal, but inside its seed bounding box.
  EXPECT_TRUE(segmentIsFree(grid,{2.5,2.5},{9.5,9.5}));
  EXPECT_EQ(buildCorridor(grid,{{2.5,2.5},{9.5,9.5}}).status,"blocked_seed_box");
}
