# 第 04 章练习

1. 理解题：构造起终点都安全、移动过程却碰撞的例子；说明为什么“理想到点”没有可用于 IMU 的物理加速度。
2. 代码题 ch04-1：补全圆盘与圆障碍的整段扫掠检查。
3. 实验题：先在暂停状态发送目标再单步，核对只执行一次。改变半径，比较同一条贴近障碍的运动是否通过；记录 seed、半径、起终点与结果。

从仓库根目录：

~~~bash
g++ -std=c++17 -I/usr/include/eigen3 -Iros2_ws/src/motion2d/include \
  -Iexercises/ch04/starter exercises/ch04/check_sweep.cpp \
  ros2_ws/src/motion2d/src/geometry/obstacles.cpp -o /tmp/ch04_sweep
/tmp/ch04_sweep
~~~

用 solutions 替换 starter 检查参考解。评分要求：穿越、相切、刚好分离、原地不动四类都正确，不只检查端点。提示见 [hints.md](hints.md)。

4. 代码题 ch04-2：完成 velocity_step.hpp 的 odom 系速度积分。检查位置、时间单位与跨越 pi 的 yaw。
5. 实验题：速度模式下分别发 odom 与 base_link 的同一速度。转向后两者的轨迹为什么不同？停止发送，观察 command_timeout；暂停 2 秒后单步，超时依据墙钟还是模拟时间？

~~~bash
g++ -std=c++17 -I/usr/include/eigen3 -Iros2_ws/src/motion2d/include \
  -Iexercises/ch04/starter exercises/ch04/check_velocity.cpp \
  ros2_ws/src/motion2d/src/geometry/se2.cpp -o /tmp/ch04_velocity
/tmp/ch04_velocity
~~~
