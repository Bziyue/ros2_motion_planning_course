#include "motion2d/trajectory/banded_lu.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>

namespace motion2d
{
BandedLu::BandedLu(int size,int bandwidth) : size_(size),bandwidth_(bandwidth)
{
  if(size<=0 || bandwidth<0) {throw std::invalid_argument("Positive banded size required");}
  bands_=Eigen::MatrixXd::Zero(size,2*bandwidth+1);
}

double & BandedLu::at(int row,int column)
{
  assert(row>=0 && row<size_ && column>=0 && column<size_ && std::abs(column-row)<=bandwidth_);
  return bands_(row,column-row+bandwidth_);
}

void BandedLu::factor()
{
  // band_factor_begin
  for(int k=0;k<size_;++k) {
    const double pivot=get(k,k);
    if(!std::isfinite(pivot) || std::abs(pivot)<1e-14)
    {throw std::runtime_error("Unsafe MINCO pivot; check time/coordinate scales");}
    const int last=std::min(size_-1,k+bandwidth_);
    for(int i=k+1;i<=last;++i) {
      at(i,k)/=pivot;
      const double multiplier=get(i,k);
      for(int j=k+1;j<=last;++j) {at(i,j)-=multiplier*get(k,j);}
    }
  }
  // band_factor_end
}

Eigen::MatrixXd BandedLu::solve(Eigen::MatrixXd b,bool transpose) const
{
  if(b.rows()!=size_) {throw std::invalid_argument("Banded RHS has wrong row count");}
  if(!transpose) {
    for(int i=0;i<size_;++i) for(int j=std::max(0,i-bandwidth_);j<i;++j)
      {b.row(i)-=get(i,j)*b.row(j);}
    for(int i=size_-1;i>=0;--i) {
      for(int j=i+1;j<std::min(size_,i+bandwidth_+1);++j) {b.row(i)-=get(i,j)*b.row(j);}
      b.row(i)/=get(i,i);
    }
  } else {
    // A^T=U^T L^T: first the diagonal lower solve, then a unit-upper solve.
    for(int i=0;i<size_;++i) {
      for(int j=std::max(0,i-bandwidth_);j<i;++j) {b.row(i)-=get(j,i)*b.row(j);}
      b.row(i)/=get(i,i);
    }
    for(int i=size_-1;i>=0;--i) for(int j=i+1;j<std::min(size_,i+bandwidth_+1);++j)
      {b.row(i)-=get(j,i)*b.row(j);}
  }
  if(!b.allFinite()) {throw std::runtime_error("Nonfinite banded solution");}
  return b;
}
}  // namespace motion2d
