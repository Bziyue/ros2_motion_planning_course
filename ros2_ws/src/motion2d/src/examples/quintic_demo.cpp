#include "motion2d/trajectory/quintic.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

/** @brief Two durations for one rest-to-rest move, plus a curved C2 joined example. */
int main(int argc,char ** argv)
{
  const std::filesystem::path out=argc>1 ? argv[1] : "tmp/ch14_quintic";
  std::filesystem::create_directories(out);
  std::ofstream data(out/"samples.csv"), summary(out/"summary.csv"), coefficients(out/"coefficients.csv");
  data<<std::setprecision(17)<<"case,t,x,y,vx,vy,ax,ay,jx,jy\n";
  coefficients<<std::setprecision(17)<<"case,piece,T,axis,c0,c1,c2,c3,c4,c5\n";
  summary<<"case,duration,peak_speed,peak_acceleration,peak_jerk\n";
  for(int scenario=0;scenario<3;++scenario) {
    motion2d::TranslationState a,b,c; b.position={1,0}; c.position={2,0};
    if(scenario==2) {b.position={1,1}; b.velocity={.8,0}; b.acceleration={0,-.5};}
    std::vector<motion2d::QuinticPiece> pieces;
    const double T=scenario==0 ? 2 : 4;
    pieces.push_back(motion2d::interpolateQuintic(a,b,T));
    if(scenario==2) pieces.push_back(motion2d::interpolateQuintic(b,c,T));
    motion2d::PolynomialTrajectory trajectory(pieces);
    double speed=0,accel=0,jerk=0;
    for(int n=0;n<=2000;++n) {
      const double t=(n/2000.)*trajectory.duration(); const auto s=trajectory.sample(t);
      const auto j=trajectory.evaluate(t,3);
      speed=std::max(speed,s.velocity.norm()); accel=std::max(accel,s.acceleration.norm()); jerk=std::max(jerk,j.norm());
      data<<scenario<<','<<t<<','<<s.position.x()<<','<<s.position.y()<<','<<s.velocity.x()<<','<<s.velocity.y()<<','
          <<s.acceleration.x()<<','<<s.acceleration.y()<<','<<j.x()<<','<<j.y()<<'\n';
    }
    for(std::size_t i=0;i<pieces.size();++i) for(int axis=0;axis<2;++axis) {
      coefficients<<scenario<<','<<i<<','<<pieces[i].duration<<','<<axis;
      for(int k=0;k<6;++k) coefficients<<','<<pieces[i].coefficients(axis,k);
      coefficients<<'\n';
    }
    summary<<scenario<<','<<trajectory.duration()<<','<<speed<<','<<accel<<','<<jerk<<'\n';
    std::cout<<scenario<<" T="<<trajectory.duration()<<" speed="<<speed<<" accel="<<accel<<" jerk="<<jerk<<'\n';
  }
}
