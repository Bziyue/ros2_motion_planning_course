#include "motion2d/sim/bias_walk.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <iostream>
/** @brief Reproduce one gyro path and independent endpoint ensembles at two rates. */
int main(int argc,char ** argv) {
  const std::filesystem::path out=argc>1?argv[1]:"tmp/appC_bias";std::filesystem::create_directories(out);
  std::ofstream path(out/"path.csv"),stats(out/"summary.csv");
  if(!path || !stats) {std::cerr<<"Cannot open output files\n";return 1;}
  path<<"t,bias,white,measurement\n"<<std::setprecision(17);
  stats<<"rate,trials,T,density,mean,variance,expected_variance\n"<<std::setprecision(17);
  constexpr double density=.002,T=10,white_std=.004;
  std::mt19937 bias_rng(20260926),white_rng(20260927);
  std::normal_distribution<double> bias_normal,white_normal;
  double b=0;
  for(int k=0;k<=1000;++k) {
    if(k) b=motion2d::stepBiasWalk(b,density,.01,bias_normal(bias_rng));
    const double white=white_std*white_normal(white_rng);path<<k*.01<<','<<b<<','<<white<<','<<b+white<<'\n';
  }
  // Separate distributions: cached variates must not couple the two RNG streams.
  for(int rate:{100,200}) {
    std::mt19937 rng(20260926+rate);std::normal_distribution<double> noise;
    double sum=0,square=0;constexpr int trials=5000;
    for(int j=0;j<trials;++j) {
      double bias=0;
      for(int k=0;k<static_cast<int>(T*rate);++k) bias=motion2d::stepBiasWalk(bias,density,1./rate,noise(rng));
      sum+=bias;square+=bias*bias;
    }
    const double mean=sum/trials,variance=(square-trials*mean*mean)/(trials-1);
    stats<<rate<<','<<trials<<','<<T<<','<<density<<','<<mean<<','<<variance<<','<<density*density*T<<'\n';
  }
}
