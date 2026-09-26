#pragma once
#include <limits>
#include <optional>
#include "motion2d/dynamics/ackermann_flatness.hpp"
#include "motion2d/trajectory/polynomial.hpp"
#include "motion2d/planning/corridor.hpp"
#include "motion2d/optimization/bfgs.hpp"
namespace motion2d {
/** @brief A regular geometric quintic q(u), u in [0,1], traversed in duration seconds.
 * @details geometry.duration is exactly 1, not the physical time. A quintic
 * progress law stops with zero speed and acceleration at every piece boundary.
 */
struct AckermannPiece {QuinticPiece geometry;double duration=1;};
/** @brief Progress and its physical time derivatives: unitless, 1/s, 1/s^2. */
struct ProgressSample {double u=0,rate=0,acceleration=0;};
/** @brief h(r)=10r^3-15r^4+6r^5, with exact resting endpoints. */
ProgressSample stopProgress(double time,double duration);
/** @brief Convert geometric flat-map quantities into physical quantities, including stops.
 * @pre Geometry has a nonzero tangent, progress.rate>=0; parameters are fixed.
 */
AckermannFlatOutput warpAckermann(const AckermannFlatOutput & geometry,
  const ProgressSample & progress,const AckermannParameters & parameters);
/** @brief Analytic reverse of warpAckermann; u itself affects evaluation of q(u). */
struct WarpGradient {AckermannFlatOutput geometry;double rate=0,acceleration=0;};
WarpGradient warpAckermannBackward(const AckermannFlatOutput & geometry,const ProgressSample & progress,
  const AckermannFlatOutput & gradient,const AckermannParameters & parameters);
/** @brief State and feedforward recovered from a retained geometric path. */
struct AckermannReference {State2D state;AckermannFlatOutput dynamics;};
/** @brief Stop-to-stop curve; shared tangents give continuous yaw and zero end steering.
 * @details No reverse planning and no nonzero-speed handover. Boundary checks
 * do not replace geometric regularity/collision/dynamic certification.
 */
class AckermannTrajectory {
public:
  explicit AckermannTrajectory(std::vector<AckermannPiece> pieces);
  const std::vector<AckermannPiece> & pieces() const {return pieces_;}
  double duration() const {return duration_;}
  AckermannReference sample(double time,const AckermannParameters & parameters) const;
private:
  std::vector<AckermannPiece> pieces_;double duration_=0;
};
/** @brief Physical planning limits; actuator limits may be larger for feedback reserve. */
struct AckermannLimits {
  AckermannParameters vehicle;
  double speed=.7,force=1,steering=.5,steering_rate=.7,lateral_acceleration=.8;
};
void validateAckermannLimits(const AckermannLimits & limits);
/** @brief One direct cost/gradient evaluation, before endpoint tangent elimination. */
struct AckermannPieceGradient {
  double cost=0,time=0;
  Eigen::Matrix<double,6,2> coefficients=Eigen::Matrix<double,6,2>::Zero();
};
/** @brief Fixed-normalized-time quadrature with analytic geometric/physical backpropagation.
 * @details Cubic normalized penalties at 85% of limits; optional verified corridor.
 * Caller supplies valid constants. No finite-difference gradients in optimization.
 */
AckermannPieceGradient ackermannPieceCost(const AckermannPiece & piece,const AckermannLimits & limits,
  const ConvexRegion * region=nullptr,int quadrature_steps=32);
/** @brief Sufficient continuous bounds; geometry and timing failures remain distinct. */
struct AckermannCertificate {
  bool geometry_valid=false,certified=false;
  double min_tangent=std::numeric_limits<double>::infinity();
  double corridor_residual=-std::numeric_limits<double>::infinity();
  double speed=0,force=0,steering=0,steering_rate=0,lateral_acceleration=0;
};
/** @brief Subdivided Bézier hulls bound regularity, corridor and rational flat-map quantities.
 * @pre Exactly one independently verified configuration-space region per piece.
 * @details Uses global bounds on progress derivatives; conservative, never a
 * finite-sample safety claim. Geometry is independent of physical duration.
 */
AckermannCertificate certifyAckermann(const AckermannTrajectory & trajectory,
  const std::vector<ConvexRegion> & regions,const AckermannLimits & limits,int depth=6);
/** @brief Small optimizer: fixed knot poses, positive tangent lengths and durations. */
struct AckermannPlanningConfig {AckermannLimits limits;BfgsConfig solver;int quadrature_steps=32;};
struct AckermannPlan {
  std::optional<AckermannTrajectory> curve;
  BfgsResult solver;AckermannCertificate certificate;
  int retiming_steps=0;std::string status;
};
/** @brief Optimize geometric tangent lengths and timing using flatness VJPs.
 * @details Knots include endpoints and specify heading; no Hybrid A-star or reverse
 * search or completeness claim. Only a converged, certified result is returned.
 * Conservative dynamic bounds may require explicit post-optimization retiming;
 * that step cannot repair a geometry/steering violation.
 */
AckermannPlan planAckermann(const std::vector<Pose2D> & knots,const std::vector<double> & durations,
  const std::vector<ConvexRegion> & regions,const AckermannPlanningConfig & config={});
}
