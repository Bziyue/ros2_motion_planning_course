#include "motion2d/trajectory/polynomial.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion2d
{
Eigen::Vector2d derivative(const QuinticPiece & piece, double time, int order)
{
  if (order < 0 || order > 5 || !std::isfinite(time) || time < 0 || time > piece.duration ||
    !std::isfinite(piece.duration) || piece.duration <= 0 || !piece.coefficients.allFinite())
  {throw std::invalid_argument("Invalid quintic sample");}
  // polynomial_derivative_begin
  Eigen::Vector2d value = Eigen::Vector2d::Zero();
  for (int k = 5; k >= order; --k) {
    double factor = 1;
    for (int j = 0; j < order; ++j) {factor *= k-j;}
    value = value*time + factor*piece.coefficients.col(k);
  }
  return value;
  // polynomial_derivative_end
}

PolynomialTrajectory::PolynomialTrajectory(std::vector<QuinticPiece> pieces)
: pieces_(std::move(pieces))
{
  if (pieces_.empty()) {throw std::invalid_argument("Trajectory needs at least one piece");}
  for (std::size_t i = 0; i < pieces_.size(); ++i) {
    const auto & p = pieces_[i];
    // Sampling at the far endpoint also catches coefficient overflow in evaluation.
    for (int d = 0; d <= 2; ++d) {
      const auto end = derivative(p,p.duration,d);
      if (!end.allFinite()) {throw std::invalid_argument("Nonfinite endpoint");}
      if (i && (derivative(p,0,d)-derivative(pieces_[i-1],pieces_[i-1].duration,d)).norm()>1e-7)
      {throw std::invalid_argument("Trajectory is not C2 at a join");}
    }
    duration_ += p.duration; ends_.push_back(duration_);
    if (!std::isfinite(duration_)) {throw std::invalid_argument("Nonfinite total duration");}
  }
}

Eigen::Vector2d PolynomialTrajectory::evaluate(double time, int order) const
{
  if (!std::isfinite(time) || time < 0 || time > duration_)
  {throw std::invalid_argument("Trajectory time outside its closed interval");}
  const auto it = std::upper_bound(ends_.begin(),ends_.end(),time);
  const auto i = std::min<std::size_t>(it-ends_.begin(),pieces_.size()-1);
  const double begin = i ? ends_[i-1] : 0;
  return derivative(pieces_[i],std::clamp(time-begin,0.0,pieces_[i].duration),order);
}

TranslationState PolynomialTrajectory::sample(double time) const
{
  return {evaluate(time,0),evaluate(time,1),evaluate(time,2)};
}
}  // namespace motion2d
