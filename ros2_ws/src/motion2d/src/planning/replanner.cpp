#include "motion2d/planning/replanner.hpp"
#include "motion2d/trajectory/quintic.hpp"
#include <algorithm>
#include <chrono>
#include <deque>
#include <limits>
#include <stdexcept>
namespace motion2d {
LocalRoute observedLocalRoute(const PlanningGrid & grid,const std::vector<std::int8_t> & observed,
  const Eigen::Vector2d & start,const Eigen::Vector2d & goal,double max_length) {
  if(observed.size()!=grid.blocked.size() || !start.allFinite() || !goal.allFinite() || !std::isfinite(max_length) || max_length<=0)
    throw std::invalid_argument("Invalid local route input");
  LocalRoute out;const auto first=grid.cellIndex(start),last=grid.cellIndex(goal);
  if(!first || grid.blocked[*first]) {out.status="invalid_start";return out;}
  if(!last) {out.status="goal_outside_map";return out;}
  if(observed[*last]>=50) {out.status="goal_occupied";return out;}
  const int width=grid.geometry.width,height=grid.geometry.height;
  std::vector<std::uint8_t> reached(grid.blocked.size(),0);std::deque<int> queue{*first};reached[*first]=1;
  while(!queue.empty()) {
    const int index=queue.front();queue.pop_front();const int x=index%width,y=index/width;
    for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) {
      if((dx==0 && dy==0) || !grid.freeCell(x+dx,y+dy)) continue;
      if(dx && dy && (!grid.freeCell(x+dx,y) || !grid.freeCell(x,y+dy))) continue;
      const int next=(y+dy)*width+x+dx;if(!reached[next]) {reached[next]=1;queue.push_back(next);}
    }
  }
  Eigen::Vector2d target=goal;bool global=reached[*last]!=0;
  if(!global) {
    int best=-1;double distance=std::numeric_limits<double>::infinity();
    const int radius=int(std::ceil(grid.clearance_radius/grid.geometry.resolution))+2;
    // frontier_selection_begin
    for(int i=0;i<int(reached.size());++i) if(reached[i]) {
      bool near_unknown=false;const int x=i%width,y=i/width;
      for(int dy=-radius;dy<=radius && !near_unknown;++dy)
        for(int dx=-radius;dx<=radius;++dx) {
          const int nx=x+dx,ny=y+dy;
          if(nx>=0 && nx<width && ny>=0 && ny<height && observed[ny*width+nx]<0) {near_unknown=true;break;}
        }
      const double candidate=(grid.cellCenter(i)-goal).squaredNorm();
      if(near_unknown && candidate<distance) {distance=candidate;best=i;}
    }
    if(best<0) {out.status="no_reachable_frontier";return out;}
    target=grid.cellCenter(best);
    if((start-goal).norm()-(target-goal).norm()<.1) {out.status="no_progress_frontier";return out;}
    // frontier_selection_end
  }
  const auto searched=astar(grid,start,target,true);
  if(!searched.success) {out.status=searched.status;return out;}
  out.path.push_back(searched.path.front());double length=0;bool truncated=false;
  for(std::size_t j=1;j<searched.path.size();++j) {
    const Eigen::Vector2d delta=searched.path[j]-out.path.back();const double segment=delta.norm();
    if(segment<1e-10) continue;
    if(max_length-length<1e-9) {truncated=true;break;}
    if(length+segment>max_length) {
      out.path.push_back(out.path.back()+delta*((max_length-length)/segment));truncated=true;break;
    }
    out.path.push_back(searched.path[j]);length+=segment;
  }
  if(out.path.size()<2) {out.status="already_at_local_goal";return out;}
  out.success=true;out.reaches_global=global && !truncated;
  out.status=out.reaches_global ? "global_goal" : global ? "route_prefix" : "frontier_prefix";return out;
}
ReplanConfig::ReplanConfig() {
  optimization.bezier_penalties=true;optimization.cost.corridor_weight=50000;optimization.cost.corridor_margin=.04;
  optimization.cost.speed_max=.45;optimization.cost.acceleration_max=.5;
  optimization.limits.speed=.7;optimization.limits.acceleration=.8;
  optimization.solver.max_iterations=120;optimization.solver.max_wall_seconds=.04;
}
ReplanResult replanObserved(const PlanningGrid & grid,const std::vector<std::int8_t> & observed,
  const Esdf2D & field,const TranslationState & start,const Eigen::Vector2d & goal,const ReplanConfig & config) {
  const auto begin=std::chrono::steady_clock::now();ReplanResult result;
  const auto finish=[&]{result.seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();return result;};
  for(double v:{config.nominal_speed,config.min_duration,config.max_route_length})
    if(!std::isfinite(v) || v<=0) throw std::invalid_argument("Invalid local trajectory settings");
  if(!start.position.allFinite() || !start.velocity.allFinite() || !start.acceleration.allFinite()) throw std::invalid_argument("Invalid future boundary");
  const auto route=observedLocalRoute(grid,observed,start.position,goal,config.max_route_length);
  result.path=route.path;result.reaches_global=route.reaches_global;
  if(!route.success) {result.status=route.status;return finish();}
  const auto corridor=buildCorridor(grid,route.path,true,config.corridor_extension);
  if(!corridor.success) {result.status="corridor:"+corridor.status;return finish();}
  result.regions=corridor.regions;
  std::vector<double> times;std::vector<Eigen::Vector2d> q(corridor.waypoints.begin()+1,corridor.waypoints.end()-1);
  for(std::size_t i=1;i<corridor.waypoints.size();++i)
    times.push_back(std::max(config.min_duration,(corridor.waypoints[i]-corridor.waypoints[i-1]).norm()/config.nominal_speed));
  auto optimization=config.optimization;optimization.limits.clearance=grid.clearance_radius;
  optimization.cost.esdf_distance=grid.clearance_radius+.05;
  TranslationState end;end.position=corridor.waypoints.back();
  result.optimization_status="disabled";
  if(config.optimize) {
    try {
      auto solution=optimizeSpline(start,end,q,times,optimization,corridor.regions,&field);
      result.optimization_status=solution.solver.status;
      if(solution.curve && solution.solver.converged() && certifyBezier(*solution.curve,corridor.regions,optimization.limits).certified) {
        result.curve=std::move(solution.curve);result.success=result.optimized=true;result.status="optimized:"+route.status;return finish();
      }
    } catch(const std::exception & e) {result.optimization_status=std::string("failed:")+e.what();}
  }
  // replanner_fallback_begin
  for(double scale:{1.,1.5,2.,3.}) {
    std::vector<QuinticPiece> pieces;TranslationState left=start;
    for(std::size_t i=0;i<times.size();++i) {
      TranslationState right;right.position=corridor.waypoints[i+1];
      pieces.push_back(interpolateQuintic(left,right,times[i]*scale));left=right;
    }
    PolynomialTrajectory candidate(std::move(pieces));
    if(certifyBezier(candidate,corridor.regions,optimization.limits).certified) {
      result.curve=std::move(candidate);result.success=true;
      result.status="fallback_stop_segments:"+route.status;return finish();
    }
  }
  // replanner_fallback_end
  result.status="no_certified_local_trajectory";return finish();
}
}
