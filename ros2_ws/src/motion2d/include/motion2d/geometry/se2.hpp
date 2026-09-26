#pragma once
#include <Eigen/Core>

namespace motion2d
{
inline constexpr double kPi = 3.14159265358979323846;

/** @brief Pose T_A_B: B's origin (m) and counterclockwise yaw (rad) expressed in A. */
struct Pose2D
{
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  double yaw = 0.0;
};

/** @brief Map a finite angle (rad) into [-pi, pi). */
double wrapAngle(double angle);

/** @brief Counterclockwise 2D rotation, with yaw measured in radians. */
Eigen::Matrix2d rotation(double yaw);

/**
 * @brief Convert a B-frame point to frame A using T_A_B; inputs/outputs in metres.
 * @details \f$p_A=R(\psi)p_B+t\f$. See chapter 02.
 */
Eigen::Vector2d transformPoint(const Pose2D & pose, const Eigen::Vector2d & point);

/** @brief Compose T_A_B and T_B_C, returning T_A_C. */
Pose2D compose(const Pose2D & lhs, const Pose2D & rhs);

/** @brief Return T_B_A given T_A_B. Position is -R^T t, not simply -t. */
Pose2D inverse(const Pose2D & pose);
}  // namespace motion2d
