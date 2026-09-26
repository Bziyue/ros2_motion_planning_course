#include <cmath>
#include <iostream>
#include "point_jacobian.hpp"
int main()
{
  for (double yaw : {-.9, 0.0, 1.3}) {
    Eigen::Vector2d p{2.1, -.7};
    const auto project = [&](const Eigen::Vector3d & q) {
        return Eigen::Vector2d{q.x() + std::cos(q.z()) * p.x() - std::sin(q.z()) * p.y(),
          q.y() + std::sin(q.z()) * p.x() + std::cos(q.z()) * p.y()};
      };
    const Eigen::Vector3d state{.8, -1.2, yaw};
    const auto j = exerciseJacobian(project(state) - state.head<2>());
    for (int axis = 0; axis < 3; ++axis) {
      auto a = state, b = state;
      a[axis] += 1e-6; b[axis] -= 1e-6;
      if ((j.col(axis) - (project(a) - project(b)) / 2e-6).norm() > 1e-8) {
        std::cout << "FAIL yaw=" << yaw << " axis=" << axis << '\n'; return 1;
      }
    }
  }
  std::cout << "PASS ch08 Jacobian: central differences at three orientations\n";
}
