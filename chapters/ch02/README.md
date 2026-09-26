# 第 02 章：二维坐标变换与时间

先修：第 01 章。当前首先提供独立 SE(2) 数学库和实验；时钟与 TF 在同章的后续功能加入。

## 一、用可以手算的点检查坐标方向

机器人圆心在 map 的 `(1,2) m`，朝向逆时针 90 度。机器人前方 2 m 的点在机体系为 `(2,0)`，在 map 中应为 `(1,4)`。

~~~bash
source /opt/ros/lyrical/setup.bash
cd /home/zdp/ForCodex/ros2_motion_planning_course/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 run motion2d se2_demo
colcon test --packages-select motion2d --event-handlers console_direct+
colcon test-result --verbose
~~~

预期输出 body `(2,0)`、map `(1,4)`、recovered `(2,0)`。这个程序只调用数学库，不创建 ROS 节点，也不需要 RViz2。

## 二、函数和公式

源码：`motion2d/geometry/se2.hpp` 与 `src/geometry/se2.cpp`。

- `Pose2D` 表示 `T_A_B`：B 的原点和朝向用 A 的坐标表达，单位 m/rad。
- `transformPoint`：`p_A = R(yaw) p_B + t`。
- `inverse`：位置为 `-Rᵀt`，角度为 `-yaw`，不能简单把位置取负。
- `compose`：从右向左应用变换，`T_A_C = T_A_B T_B_C`。
- `wrapAngle` 统一到 `[-pi,pi)`；不要直接相减跨越 ±pi 的角度。

测试包括手算 90 度旋转、带外参的坐标链、200 个固定种子的往返变换、角度边界。练习见 [ch02](../../exercises/ch02/README.md)。
