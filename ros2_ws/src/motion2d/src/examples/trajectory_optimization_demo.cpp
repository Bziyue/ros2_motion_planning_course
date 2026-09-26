#include "motion2d/trajectory/minco_optimizer.hpp"
#include "motion2d/planning/astar.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace motion2d;
/** @brief Fixed supplied-map experiment, not an unknown-world navigation claim. */
int main(int argc,char ** argv) {
  const std::filesystem::path dir=argc>1 ? argv[1] : "tmp/ch15_optimization";
  std::filesystem::create_directories(dir);
  GridConfig g;g.width=61;g.height=41;g.resolution=.1;g.origin.setZero();
  std::vector<int8_t> raw(g.width*g.height,0);for(int y=0;y<29;++y) raw[y*g.width+30]=100;
  const auto grid=inflateGrid(g,raw,{.15,0,35,true});
  const auto route=astar(grid,{.75,.75},{5.35,.75});
  const auto corridor=buildCorridor(grid,route.path,true,.6);
  if(!corridor.success) return 1;
  Esdf2D field(g,raw);
  TranslationState start,finish;start.position=corridor.waypoints.front();finish.position=corridor.waypoints.back();
  std::vector<Eigen::Vector2d> q(corridor.waypoints.begin()+1,corridor.waypoints.end()-1);
  std::vector<double> T;for(std::size_t i=1;i<corridor.waypoints.size();++i) T.push_back((corridor.waypoints[i]-corridor.waypoints[i-1]).norm()/.6);
  std::ofstream summary(dir/"summary.csv"),samples(dir/"samples.csv");
  summary<<"mode,status,cost,duration,speed,acceleration,corridor_residual,clearance,sampled_feasible,iterations,seconds\n";
  samples<<"mode,t,x,y,speed,acceleration\n";
  for(int mode=0;mode<3;++mode) {
    MincoOptimizationConfig c;c.optimize_waypoints=mode>0;c.optimize_times=mode>1;
    c.limits.clearance=.15;c.solver.max_iterations=400;c.solver.gradient_tolerance=1e-5;
    auto r=optimizeMinco(start,finish,q,T,c,corridor.regions,&field);
    if(!r.curve) {std::cerr<<r.solver.status<<'\n';return 2;}
    summary<<mode<<','<<r.solver.status<<','<<r.solver.objective.value<<','<<r.curve->duration()<<','<<r.samples.peak_speed<<','<<r.samples.peak_acceleration<<','<<r.samples.max_corridor_residual<<','<<r.samples.min_clearance_lower_bound<<','<<r.samples.samples_feasible<<','<<r.solver.iterations<<','<<r.solver.seconds<<'\n';
    for(int j=0;j<=1500;++j) {const double t=r.curve->duration()*j/1500;const auto p=r.curve->sample(t);samples<<mode<<','<<t<<','<<p.position.x()<<','<<p.position.y()<<','<<p.velocity.norm()<<','<<p.acceleration.norm()<<'\n';}
    std::cout<<"mode="<<mode<<" "<<r.solver.status<<" cost="<<r.solver.objective.value<<" "<<r.samples.status<<" residual="<<r.samples.max_corridor_residual<<" clearance="<<r.samples.min_clearance_lower_bound<<'\n';
  }
}
