#include "motion2d/trajectory/minco2d.hpp"
#include "motion2d/trajectory/quintic.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

/** @brief Same knots/times: stop at each knot versus the exact minimum-jerk interpolant. */
int main(int argc,char ** argv)
{
  using namespace motion2d;
  const std::filesystem::path out=argc>1 ? argv[1] : "tmp/ch15_minco";std::filesystem::create_directories(out);
  TranslationState a,b;b.position={4,0};std::vector<Eigen::Vector2d>q{{1,1},{2,-.2},{3,.8}};
  const std::vector<double>T{1.5,1.5,1.5,1.5};Minco2D minco(a,b,q,T);const auto smooth=minco.trajectory();
  std::vector<QuinticPiece>stops;TranslationState from=a;
  auto points=q;points.push_back(b.position);
  for(std::size_t i=0;i<T.size();++i){TranslationState to;to.position=points[i];stops.push_back(interpolateQuintic(from,to,T[i]));from=to;}
  const PolynomialTrajectory stopped(stops);
  std::ofstream samples(out/"samples.csv"),summary(out/"summary.csv");
  samples<<std::setprecision(17)<<"mode,t,x,y,vx,vy,ax,ay,jx,jy\n";
  summary<<"mode,energy,peak_speed,peak_acceleration\n";
  for(int mode=0;mode<2;++mode){
    const auto & curve=mode ? smooth : stopped;double energy=0,speed=0,acceleration=0;
    for(const auto & piece:curve.pieces()){
      const Eigen::Matrix<double,6,2>C=piece.coefficients.transpose();energy+=(C.array()*(jerkGram(piece.duration)*C).array()).sum();
    }
    for(int i=0;i<=3000;++i){const double t=(i/3000.)*curve.duration();const auto s=curve.sample(t);const auto j=curve.evaluate(t,3);
      speed=std::max(speed,s.velocity.norm());acceleration=std::max(acceleration,s.acceleration.norm());
      samples<<mode<<','<<t<<','<<s.position.x()<<','<<s.position.y()<<','<<s.velocity.x()<<','<<s.velocity.y()<<','
        <<s.acceleration.x()<<','<<s.acceleration.y()<<','<<j.x()<<','<<j.y()<<'\n';
    }
    summary<<mode<<','<<energy<<','<<speed<<','<<acceleration<<'\n';
    std::cout<<mode<<" energy="<<energy<<" speed="<<speed<<" acceleration="<<acceleration<<'\n';
  }
  const auto partials=minco.energyPartials();const auto g=minco.propagate(partials.coefficients,partials.times);
  std::ofstream gradients(out/"gradients.csv");gradients<<"kind,index,x,y\n";
  for(int i=0;i<g.waypoints.rows();++i)gradients<<"waypoint,"<<i<<','<<g.waypoints(i,0)<<','<<g.waypoints(i,1)<<'\n';
  for(int i=0;i<g.times.size();++i)gradients<<"time,"<<i<<','<<g.times(i)<<",0\n";
}
