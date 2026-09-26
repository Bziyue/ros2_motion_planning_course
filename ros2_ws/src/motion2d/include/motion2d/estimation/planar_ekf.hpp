#pragma once
#include "motion2d/geometry/se2.hpp"

namespace motion2d
{
using Vector8d = Eigen::Matrix<double, 8, 1>;
using Matrix8d = Eigen::Matrix<double, 8, 8>;

/** @brief Horizontal raw IMU; roll=pitch=0, sensor at centre, aligned with body.
 * @details Body x/y specific force (m/s^2) equals body x/y acceleration because
 * gravity is vertical. Variance order is ax, ay, gz, per sample (not density).
 */
struct PlanarImu
{
  Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();
  double yaw_rate = 0;
  Eigen::Vector3d variance{.04 * .04, .05 * .05, .004 * .004};
};

/** @brief One held-IMU step and its derivatives; state [px,py,vx,vy,yaw,bax,bay,bg]. */
struct ImuPrediction
{
  Vector8d state;
  Matrix8d f;
  Eigen::Matrix<double, 8, 3> g;
};

/** @brief Discrete planar inertial step, F=d(next)/d(state), G=d(next)/d(imu).
 * @param dt Held sample interval in seconds, (0,.1].
 * @details Rotation is held at interval start; this is an explicit approximation.
 * Acceleration biases are in body axes, gyro bias in rad/s. No truth attitude.
 */
ImuPrediction predictImu(const Vector8d & state, const PlanarImu & imu, double dt);

/** @brief EKF settings. Bias diffusion is stddev per sqrt(s), unlike IMU samples. */
struct EkfConfig
{
  bool estimate_bias = false;
  Eigen::Vector3d bias_initial_stddev{.05, .05, .01};
  Eigen::Vector3d bias_random_walk{.0005, .0005, .0001};
  Eigen::Vector3d pose_stddev{.03, .03, .01}; ///< m, m, rad; not calibrated ICP uncertainty.
  double innovation_gate = 16.3; ///< Squared Mahalanobis threshold for 3 residuals.
};

/** @brief Additive planar EKF with gated pose correction and Joseph covariance.
 * @details Pedagogical loose coupling: the local lidar map and filter reuse
 * history, so this is not a claim of statistically independent measurements.
 * Call reset at a chosen sensor-derived origin; default velocity prior is zero.
 */
class PlanarEkf
{
public:
  explicit PlanarEkf(EkfConfig config = {});
  /** @brief Start a new gauge, clearing biases; initial velocity uncertainty is 1 m/s. */
  void reset(const Pose2D & origin = {}, const Eigen::Vector2d & velocity = Eigen::Vector2d::Zero());
  /** @brief Advance using a held raw sample; covariance uses per-sample variance. */
  void predict(const PlanarImu & imu, double dt);
  /** @brief Correct with pose in the same odom gauge; false means innovation rejected. */
  bool correct(const Pose2D & measured);
  /** @brief Current estimated pose, in odom. */
  Pose2D pose() const {return {state_.head<2>(), state_[4]};}
  const Vector8d & state() const {return state_;}
  const Matrix8d & covariance() const {return covariance_;}
  double lastInnovationSquared() const {return last_innovation_;}

private:
  EkfConfig config_;
  Vector8d state_;
  Matrix8d covariance_;
  double last_innovation_ = 0;
};
}  // namespace motion2d
