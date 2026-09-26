# ch02：求逆变换

1. 理解题：为什么逆平移是 `-Rᵀt`，不是 `-t`？
2. 代码题：完成 starter 中 `EXERCISE(ch02-1)`，把世界坐标的点变回机体坐标。直接使用课程的 Pose2D，不复制类型。
3. 实验题：在 se2_demo 中把 yaw 从 +90 度改为 -90 度，先计算预期 map 坐标，再运行对照；完成后恢复演示。

从仓库根目录编译：

~~~bash
g++ -std=c++17 -I/usr/include/eigen3 \
  -Iros2_ws/src/motion2d/include -Iexercises/ch02/starter \
  exercises/ch02/check.cpp ros2_ws/src/motion2d/src/geometry/se2.cpp \
  -o /tmp/ch02_exercise
/tmp/ch02_exercise
~~~

starter 默认未完成。把 starter 换为 solutions 可检查参考解。提示在 [hints.md](hints.md)，参考解在 [solutions/inverse_point.hpp](solutions/inverse_point.hpp)。

4. 推导：把单位x/y轴分别旋转30度，写出旋转矩阵的两列，计算RᵀR。
5. 手算：车体在(1,2)、yaw=90度，雷达在车体前方0.5m，观测点在雷达前方2m。先分两次变换，再用compose一次变换，核对结果。
6. 表示：yaw=0/90/180度分别对应哪些(qz,qw)？代回教材的Rq检查，而不是背诵半角公式。
