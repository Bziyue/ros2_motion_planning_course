#!/usr/bin/env bash
# Optional pinned upstream comparison; not a build dependency of the ROS course.
set -euo pipefail
cd "$(dirname "$0")/.."
upstream=tmp/SplineTrajectory
pin=126525e49a43b0548bc6960e4979dd6b6258f289
if [ ! -d "$upstream/.git" ]; then
  git clone https://github.com/Bziyue/SplineTrajectory.git "$upstream"
fi
if [ "$(git -C "$upstream" rev-parse HEAD)" != "$pin" ]; then
  echo "Expected pinned upstream $pin; use an isolated checkout at tmp/SplineTrajectory." >&2
  exit 1
fi
mkdir -p tmp/ch16_upstream
src=ros2_ws/src/motion2d
g++ -O3 -fno-fast-math -DNDEBUG -std=c++17 -DMOTION2D_UPSTREAM \
  -I"$src/include" -I"$upstream/include" -I/usr/include/eigen3 \
  "$src/src/examples/spline_demo.cpp" "$src/src/trajectory/"{polynomial,quintic,banded_lu,minco2d,spline2d}.cpp \
  -o tmp/ch16_upstream/benchmark
tmp/ch16_upstream/benchmark tmp/ch16_upstream
