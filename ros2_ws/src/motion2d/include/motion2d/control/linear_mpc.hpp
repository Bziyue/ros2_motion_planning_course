#pragma once
#include "motion2d/control/pd_tracker.hpp"
#include "motion2d/optimization/box_qp.hpp"
#include "motion2d/planning/corridor.hpp"
namespace motion2d {
/** @brief Exact ZOH translation, x=[px,py,vx,vy], u=[Fx,Fy], in odom SI units. */
struct LinearModel {Eigen::Matrix4d A;Eigen::Matrix<double,4,2> B;};
/** @brief Reuse the ch04 exact integrator on basis states/inputs; no duplicate dynamics. */
LinearModel linearInertialModel(const InertialParameters & model,double dt);
/** @brief Condensed prediction X=Sx*x0+Su*U for steps 1..N. */
struct PredictionMatrices {Eigen::MatrixXd Sx,Su;};
/** @brief Lift a one-step linear model into N future states. */
PredictionMatrices predictionMatrices(const LinearModel & model,int horizon);
/** @brief All velocity/force/slew limits are componentwise, not Euclidean norms. */
struct MpcConfig {
  int horizon=20;
  double dt=.02,position_weight=40,velocity_weight=2,force_weight=.02,increment_weight=.005;
  double velocity_max=1,force_rate_max=5;
  QpSettings solver;
};
/** @brief Current solve diagnostics and newly chosen force; failure force is bounded damping. */
struct MpcResult {
  QpResult solver;
  Eigen::Vector2d force=Eigen::Vector2d::Zero();
  Eigen::VectorXd predicted_states;
  double violation=0;
};
/** @brief Small dense linear MPC; yaw stays in the independent PD controller.
 * @details Optimizes force with exact ZOH dynamics and linear node constraints.
 * Position corridor constraints apply only at prediction nodes, not between them.
 * No nonlinear ESDF term or unproved continuous collision guarantee is implied.
 */
class LinearMpc {
public:
  LinearMpc(const InertialParameters & model,const MpcConfig & config={});
  /** @brief Form the actual QP, exposed for teaching and independent verification.
   * @param reference N future p/v/a states at t+h,...,t+Nh, in odom.
   * @param previous_force Last APPLIED force, including a fallback command.
   * @param regions Either empty, or N verified configuration-space regions in odom.
   */
  BoxQp problem(const State2D & state,const std::vector<State2D> & reference,
    const Eigen::Vector2d & previous_force,const std::vector<ConvexRegion> & regions={}) const;
  /** @brief Shift successful primal sequence for warm start; discard it on any failure. */
  MpcResult step(const State2D & state,const std::vector<State2D> & reference,
    const Eigen::Vector2d & previous_force,const std::vector<ConvexRegion> & regions={});
  /** @brief Clear history after time reset or reference discontinuity. */
  void reset() {warm_.resize(0);}
private:
  InertialParameters model_;MpcConfig config_;PredictionMatrices prediction_;
  Eigen::MatrixXd Q_,D_,P_;
  Eigen::VectorXd warm_;
};
}
