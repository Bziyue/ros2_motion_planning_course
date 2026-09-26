#include "motion2d/planning/replanner.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace motion2d;
/** @brief Two explicit observed-grid fixtures, not an oracle navigation benchmark. */
int main(int argc,char ** argv) {
  const std::filesystem::path out=argc>1 ? argv[1] : "tmp/ch19_replanner";std::filesystem::create_directories(out);
  std::ofstream points(out/"curves.csv"),summary(out/"summary.csv"),maps(out/"maps.csv");
  points<<"scene,mode,t,x,y,vx,vy\n";summary<<"scene,mode,status,optimization,seconds,duration,pieces,certified\n";maps<<"scene,x,y,occupancy,blocked\n";
  for(int scene=0;scene<2;++scene) {
    GridConfig g;g.width=60;g.height=40;g.resolution=.1;g.origin={0,0};std::vector<std::int8_t> raw(g.width*g.height,0);
    for(int y=0;y<g.height;++y) for(int x=0;x<g.width;++x)
      if(scene==0 && x>=30) raw[y*g.width+x]=-1;
      else if(scene==1 && x==30 && y<28) raw[y*g.width+x]=100;
    InflationConfig inflation;inflation.radius=.1;inflation.margin=.05;const auto grid=inflateGrid(g,raw,inflation);const Esdf2D field(g,raw);
    for(int i=0;i<int(raw.size());++i) {const auto p=grid.cellCenter(i);maps<<scene<<','<<p.x()<<','<<p.y()<<','<<int(raw[i])<<','<<int(grid.blocked[i])<<'\n';}
    TranslationState start;start.position={.75,scene ? .75 : 2.};
    const Eigen::Vector2d goal(5.35,start.position.y());
    for(int mode=0;mode<2;++mode) {
      ReplanConfig config;config.optimize=mode==1;config.max_route_length=scene ? 5. : 2.;config.optimization.solver.max_wall_seconds=0;
      const auto result=replanObserved(grid,raw,field,start,goal,config);
      if(!result.success) {std::cerr<<result.status<<' '<<result.optimization_status<<'\n';return 1;}
      const auto & curve=*result.curve;const auto certificate=certifyBezier(curve,result.regions,config.optimization.limits);
      summary<<scene<<','<<mode<<','<<result.status<<','<<result.optimization_status<<','<<result.seconds<<','<<curve.duration()<<','<<curve.pieces().size()<<','<<certificate.certified<<'\n';
      for(int k=0;k<=300;++k) {double t=(double(k)/300)*curve.duration();const auto p=curve.sample(t);points<<scene<<','<<mode<<','<<t<<','<<p.position.x()<<','<<p.position.y()<<','<<p.velocity.x()<<','<<p.velocity.y()<<'\n';}
      std::cout<<scene<<' '<<mode<<' '<<result.status<<" optimizer="<<result.optimization_status<<" T="<<curve.duration()<<" certified="<<certificate.certified<<'\n';
    }
  }
}
