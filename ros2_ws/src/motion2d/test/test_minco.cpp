#include <gtest/gtest.h>
#include <Eigen/LU>
#include <random>
#include "motion2d/trajectory/minco2d.hpp"
#include "motion2d/trajectory/quintic.hpp"

using namespace motion2d;
namespace
{
// Independent dense constrained quadratic reference: no jerk/snap continuity rows.
Eigen::MatrixX2d denseQp(const TranslationState & start,const TranslationState & finish,
  const std::vector<Eigen::Vector2d> & q,const std::vector<double> & times)
{
  const int n=times.size(),constraints=6+4*(n-1),size=6*n;
  Eigen::MatrixXd H=Eigen::MatrixXd::Zero(size,size),A=Eigen::MatrixXd::Zero(constraints,size);
  Eigen::MatrixX2d b=Eigen::MatrixX2d::Zero(constraints,2);
  auto row=[&](int r,int i,double t,int d,double sign=1.){
    for(int k=d;k<6;++k) {
      double coefficient=std::pow(t,k-d);
      for(int j=0;j<d;++j) coefficient*=k-j;
      A(r,6*i+k)+=sign*coefficient;
    }
  };
  for(int i=0;i<n;++i) for(int k=3;k<6;++k) for(int l=3;l<6;++l) {
    // Three-point Gauss-Legendre is exact for the quartic jerk-square integrand.
    for(int sample=0;sample<3;++sample) {
      const double s[3]={-std::sqrt(3./5),0,std::sqrt(3./5)},w[3]={5./9,8./9,5./9};
      const double t=(1+s[sample])*times[i]/2;
      H(6*i+k,6*i+l)+=w[sample]*times[i]*k*(k-1)*(k-2)*l*(l-1)*(l-2)*std::pow(t,k+l-6);
    }
  }
  int r=0;
  for(int d=0;d<3;++d) {row(r++,0,0,d);}
  b.row(0)=start.position.transpose();b.row(1)=start.velocity.transpose();b.row(2)=start.acceleration.transpose();
  for(int i=0;i<n-1;++i) {
    row(r,i,times[i],0);b.row(r++)=q[i].transpose();
    row(r,i+1,0,0);b.row(r++)=q[i].transpose();
    for(int d=1;d<3;++d) {row(r,i,times[i],d);row(r,i+1,0,d,-1);++r;}
  }
  b.row(r)=finish.position.transpose();b.row(r+1)=finish.velocity.transpose();b.row(r+2)=finish.acceleration.transpose();
  for(int d=0;d<3;++d) {row(r++,n-1,times.back(),d);}
  Eigen::MatrixXd K=Eigen::MatrixXd::Zero(size+constraints,size+constraints);
  K.topLeftCorner(size,size)=H;K.topRightCorner(size,constraints)=A.transpose();K.bottomLeftCorner(constraints,size)=A;
  Eigen::MatrixX2d rhs=Eigen::MatrixX2d::Zero(size+constraints,2);rhs.bottomRows(constraints)=b;
  auto lu=K.fullPivLu(); EXPECT_EQ(lu.rank(),size+constraints);
  const Eigen::MatrixX2d solution=lu.solve(rhs);
  EXPECT_LT((K*solution-rhs).norm(),1e-7);
  return solution.topRows(size);
}
}
TEST(Minco, IndependentDenseQuadraticAndC4)
{
  std::mt19937 gen(1515);std::uniform_real_distribution<double> random(-1,1);
  for(int n=1;n<=6;++n) {
    TranslationState a,b;b.position={double(n),.4};a.velocity={.2,-.1};b.acceleration={0,.2};
    std::vector<double>T;std::vector<Eigen::Vector2d>q;
    for(int i=0;i<n;++i){T.push_back(1.2+.3*random(gen));if(i<n-1)q.push_back({i+1.,random(gen)});}
    const Minco2D solver(a,b,q,T);const auto curve=solver.trajectory();
    const auto reference=denseQp(a,b,q,T);
    EXPECT_LT((solver.coefficients()-reference).norm()/std::max(1.,reference.norm()),2e-9);
    for(int i=0;i<n-1;++i) for(int d=0;d<=4;++d) {
      EXPECT_LT((derivative(curve.pieces()[i],T[i],d)-derivative(curve.pieces()[i+1],0,d)).norm(),1e-8);
    }
  }
}
TEST(Minco, EnergyAndGeneralAdjointFiniteDifferences)
{
  TranslationState a,b;b.position={3,0};a.velocity={.1,0};
  std::vector<Eigen::Vector2d>q{{1,.8},{2,-.2}};std::vector<double>T{1.2,.9,1.4};
  Minco2D solver(a,b,q,T);auto direct=solver.energyPartials();
  // Extra coefficient and explicit-time terms exercise the full adjoint API.
  direct.coefficients+=.2*solver.coefficients();
  for(int i=0;i<3;++i)direct.times(i)+=.4*T[i];
  const auto gradient=solver.propagate(direct.coefficients,direct.times);
  auto cost=[&](const std::vector<Eigen::Vector2d> & p,const std::vector<double>& t){
    Minco2D s(a,b,p,t);double value=s.energyPartials().cost+.1*s.coefficients().squaredNorm();
    for(double d:t)value+=.2*d*d;return value;
  };
  const double h=1e-5;
  for(int i=0;i<2;++i)for(int d=0;d<2;++d){
    auto plus=q,minus=q;plus[i](d)+=h;minus[i](d)-=h;
    const double fd=(cost(plus,T)-cost(minus,T))/(2*h);
    EXPECT_NEAR(gradient.waypoints(i,d),fd,2e-6*std::max(1.,std::abs(fd)));
  }
  for(int i=0;i<3;++i){
    auto plus=T,minus=T;plus[i]+=h;minus[i]-=h;
    const double fd=(cost(q,plus)-cost(q,minus))/(2*h);
    EXPECT_NEAR(gradient.times(i),fd,2e-6*std::max(1.,std::abs(fd)));
  }
}
TEST(Minco, AnalyticSingleSegmentAndEnergyImprovement)
{
  TranslationState a,b;b.position={1,0};Minco2D one(a,b,{}, {2});
  const auto partials=one.energyPartials();const auto g=one.propagate(partials.coefficients,partials.times);
  EXPECT_NEAR(partials.cost,720./32,1e-9);EXPECT_NEAR(g.times(0),-5*720./64,1e-8);EXPECT_EQ(g.waypoints.rows(),0);
  b.position={3,0};std::vector<Eigen::Vector2d>q{{1,1},{2,-.2}};std::vector<double>T{1.,1.3,1.1};
  Minco2D smooth(a,b,q,T);double stops=0;TranslationState left=a;
  q.push_back(b.position);
  for(int i=0;i<3;++i){TranslationState right;right.position=q[i];const auto piece=interpolateQuintic(left,right,T[i]);
    const Eigen::Matrix<double,6,2>C=piece.coefficients.transpose();stops+=(C.array()*(jerkGram(T[i])*C).array()).sum();left=right;}
  EXPECT_LT(smooth.energyPartials().cost,stops*.9);
  EXPECT_THROW(Minco2D(a,b,{},{}),std::invalid_argument);
  EXPECT_THROW(Minco2D(a,b,{}, {0}),std::invalid_argument);
  EXPECT_THROW(Minco2D(a,b,{}, {1,1}),std::invalid_argument);
}
TEST(Minco, BandedTransposeAndNumericalFailure)
{
  const int n=15,w=3;BandedLu band(n,w);Eigen::MatrixXd A=Eigen::MatrixXd::Zero(n,n);
  for(int i=0;i<n;++i)for(int j=std::max(0,i-w);j<std::min(n,i+w+1);++j){
    A(i,j)=i==j ? 10 : .1*(i-j);band.at(i,j)=A(i,j);
  }
  const Eigen::MatrixXd B=Eigen::MatrixXd::Random(n,2);band.factor();
  EXPECT_LT((A*band.solve(B)-B).norm(),1e-12);
  EXPECT_LT((A.transpose()*band.solve(B,true)-B).norm(),1e-12);
  BandedLu singular(2,1);EXPECT_THROW(singular.factor(),std::runtime_error);
}
