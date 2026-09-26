#include "motion2d/trajectory/spline2d.hpp"
#include "motion2d/trajectory/minco2d.hpp"
#ifdef MOTION2D_UPSTREAM
#include "SplineTrajectory.hpp"
#endif
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace motion2d;
namespace {
volatile double sink=0;
template<class F> double medianMicros(F f) {
  for(int j=0;j<10;++j) sink=f();
  std::vector<double> batches;
  for(int k=0;k<21;++k) {
    auto before=std::chrono::steady_clock::now();
    for(int j=0;j<20;++j) sink=f();
    batches.push_back(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-before).count()/20);
  }
  std::sort(batches.begin(),batches.end());return batches[batches.size()/2];
}
}
/** @brief Identical clamped problem; time construction + exact energy + full energy gradient. */
int main(int argc,char ** argv) {
  std::filesystem::path dir=argc>1 ? argv[1] : "tmp/ch16_spline";std::filesystem::create_directories(dir);
  std::ofstream out(dir/"comparison.csv");
  out<<"segments,energy,coefficient_relative,gradient_relative,minco_us,spline_us,upstream_us,upstream_coefficient_relative,upstream_gradient_relative\n";
  for(int n:{4,16,64}) {
    TranslationState a,b;b.position={double(n),.2};a.velocity={.1,.2};b.acceleration={.2,-.1};
    std::vector<Eigen::Vector2d> q;std::vector<double> T;
    for(int i=0;i<n;++i) {T.push_back(1+.2*std::cos(i));if(i<n-1) q.push_back({double(i+1),std::sin(i*.7)});}
    Minco2D m(a,b,q,T);Spline2D s(a,b,q,T);auto p=m.energyPartials();auto mg=m.propagate(p.coefficients,p.times),sg=s.energyGradient();
    const double ce=(m.coefficients()-s.coefficients()).norm()/std::max(1.,m.coefficients().norm());
    const double ge=std::max((mg.times-sg.times).norm()/std::max(1.,mg.times.norm()),(mg.waypoints-sg.waypoints).norm()/std::max(1.,mg.waypoints.norm()));
    double mt=medianMicros([&]{Minco2D v(a,b,q,T);auto p=v.energyPartials();return p.cost+v.propagate(p.coefficients,p.times).times.sum();});
    double st=medianMicros([&]{Spline2D v(a,b,q,T);return v.energy()+v.energyGradient().times.sum();});
    double ut=std::numeric_limits<double>::quiet_NaN(),uce=ut,uge=ut;
#ifdef MOTION2D_UPSTREAM
    using U=SplineTrajectory::QuinticSplineND<2>;U::CoefficientMatrix points(n+1,2);
    points.row(0)=a.position;points.row(n)=b.position;for(int i=0;i<n-1;++i) points.row(i+1)=q[i];
    SplineTrajectory::BoundaryConditions<2> bc(a.velocity,a.acceleration,b.velocity,b.acceleration);
    U u(T,points,0.,bc);if(!u.isValid()) return 2;
    const auto ug=u.energyGradient();
    uce=(u.polynomial().coefficients()-s.coefficients()).norm()/std::max(1.,s.coefficients().norm());
    uge=std::max((ug.durations-sg.times).norm()/std::max(1.,sg.times.norm()),(ug.inner_points-sg.waypoints).norm()/std::max(1.,sg.waypoints.norm()));
    ut=medianMicros([&]{U v(T,points,0.,bc);return v.energy()+v.energyGradient().durations.sum();});
    if(uce>1e-8 || uge>1e-8 || std::abs(u.energy()-s.energy())>1e-8*std::max(1.,s.energy())) return 3;
#endif
    if(ce>1e-8 || ge>1e-8) return 4;
    out<<n<<','<<s.energy()<<','<<ce<<','<<ge<<','<<mt<<','<<st<<','<<ut<<','<<uce<<','<<uge<<'\n';
    std::cout<<"N="<<n<<" errors="<<ce<<','<<ge<<" minco/spline/upstream us="<<mt<<'/'<<st<<'/'<<ut<<'\n';
  }
}
