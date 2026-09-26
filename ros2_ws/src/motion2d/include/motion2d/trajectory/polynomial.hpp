#pragma once
#include <Eigen/Core>
#include <vector>

namespace motion2d
{
/** @brief Translation boundary data in one fixed frame: m, m/s and m/s². */
struct TranslationState
{
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();
};

/** @brief Quintic p(t)=sum c_k t^k, local time in seconds; columns ascend from k=0.
 * @details Row 0 is x, row 1 is y. Column k has units m/s^k. No yaw is implied.
 */
struct QuinticPiece
{
  double duration = 1.0;
  Eigen::Matrix<double, 2, 6> coefficients = Eigen::Matrix<double, 2, 6>::Zero();
};

/** @brief Evaluate derivative order 0..5 by Horner's method; no extrapolation.
 * @throws std::invalid_argument for invalid order, local time or piece data.
 */
Eigen::Vector2d derivative(const QuinticPiece & piece, double time, int order);

/** @brief A C² piecewise quintic; validates finite coefficients, durations and joins once.
 * @details Join tolerance is an absolute 1e-7 in m, m/s and m/s². This verifies
 * algebra, not collision safety or actuator feasibility. Sampling rejects times
 * outside [0,duration]; an exact interior knot uses the piece to its right.
 */
class PolynomialTrajectory
{
public:
  explicit PolynomialTrajectory(std::vector<QuinticPiece> pieces);
  /** @brief Total positive duration, seconds. */
  double duration() const {return duration_;}
  /** @brief Immutable local-time coefficients, suitable for serialization. */
  const std::vector<QuinticPiece> & pieces() const {return pieces_;}
  /** @brief Sample position/velocity/acceleration in the trajectory frame. */
  TranslationState sample(double time) const;
  /** @brief Derivative at a global trajectory time; order 0..5. */
  Eigen::Vector2d evaluate(double time, int order) const;
private:
  std::vector<QuinticPiece> pieces_;
  std::vector<double> ends_;
  double duration_ = 0;
};
}  // namespace motion2d
