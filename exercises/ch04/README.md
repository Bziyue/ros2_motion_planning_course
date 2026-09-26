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

6. 推导题：同一个 1 N 力分别作用于 1 kg 和 2 kg 机器人 1 s，初始静止，求位移和速度。再从相同 1 m/s 初速出发，用 -1 N 制动，比较制动时间和距离。
7. 代码题 ch04-3：实现无阻尼恒力/恒力矩的一步积分；输入已经限幅。更新位置必须使用旧速度。
8. 实验题：运行 inertia_demo，比对 dt=0.02 与 0.005 s；解释为何恒定输入对齐步边界时结果一致。再改变阻尼，预测无输入衰减趋势；在 ROS 模式中停止发力，验证它不会立即停下。

~~~bash
g++ -std=c++17 -I/usr/include/eigen3 -Iros2_ws/src/motion2d/include \
  -Iexercises/ch04/starter exercises/ch04/check_force.cpp \
  ros2_ws/src/motion2d/src/geometry/se2.cpp -o /tmp/ch04_force
/tmp/ch04_force
~~~

参考解仍用 solutions 替换 starter。评分要求包括平移与偏航，不能只让位置通过。自动惯量取 m*r²/2；r=0 时请显式给正惯量，不能拿零做除数。

9. 代码题 ch04-4：对连续理想圆轨迹的速度求导，补齐 reference_acceleration.hpp。注意链式法则中的第二个 omega；这是初始朝向轴下的分量，旋转到 odom 后才发布。
10. 实验题：用 reference 模式跑一圈，验证初末 p/v/a 一致。把 reference_omega 翻倍，速度与加速度分别怎样变化？为什么初始速度不为零，却仍可称为连续运动？

~~~bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch04/starter \
  exercises/ch04/check_reference.cpp -o /tmp/ch04_reference
/tmp/ch04_reference
~~~

## ch04-5 微分平坦与力梯度

推导 `J=0.5*|m*a+c*v|²` 的加速度偏导，完成 `starter/flat_force.hpp`。
从仓库根目录运行：

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch04/starter exercises/ch04/check_flatness.cpp -o /tmp/ch04_flatness
/tmp/ch04_flatness
```

参考解将 `starter` 改为 `solutions`。评分：两轴中央差分误差均小于 1e-8；解释质量与阻尼的作用。
实验：运行 `ros2 run motion2d flatness_demo`，比较相同轨迹在两种质量下的力。提示与答案仍分开。

## ch04-6 阿克曼曲率导数

完成 `starter/curvature_rate.hpp`，对非零 v 推导 kappa=cross(v,a)/|v|³ 的时间导数。

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch04/starter exercises/ch04/check_ackermann.cpp -o /tmp/ch04_ackermann
/tmp/ch04_ackermann
```

参考目录 solutions；误差需 <1e-8。实验运行 `ros2 run motion2d ackermann_demo`，推导其转角与力。
解释：为何零速处不应用 epsilon 偷换分母？为何减速不能修正过小的转弯半径？

## 逐步推导与图解练习

1. 对机体系恒速 u=(1,0) m/s、角速度 1 rad/s，从朝向0开始积分 π/2 秒，先按两个分量积分，再用 sinc 公式核对位移。
2. 对阻尼方程 v'=u−λv 使用积分因子，推导 A、B、C，写出 B 到 λ²、C 到 λ² 的展开。说明 λ→0 时为什么恢复恒加速度更新。
3. 用曲线与弦的偏差积分式重新推出 a_max*h²/8。把步长减半，包络增加量如何变化？
4. m=2 kg、c=0.1 kg/s、v=(0.3,0)、a=(0.4,0) 时，计算 J=|F|²/2 及其速度、加速度梯度，再用中心差分检查。
5. 从 v=n*e、a=n_dot*e+n*psi_dot*e_perp 推导曲率与侧向加速度。把 n=0 的位置画在一条已知路径上，说明路径切向保存了什么信息。
6. 对阿克曼反向映射先只设置 force 的输出梯度为1，沿 F→a_parallel→b,n→v,a 回传；然后单独设置 steering_rate 的梯度，按正文表格逐行回传。

正文推导和这些手算练习是现有六个代码练习的阅读入口。参考手算值见 hints.md。
