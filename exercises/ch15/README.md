# 第15章练习：从最小 jerk 到路点和时间梯度

先完成手算，再把同一组数代入代码。教材第15章给出了各步推导，提示放在 hints.md。

## 1. 系数与最优曲线

1. 推导 Q(3,5) 和 Q(4,4)。对 p(t)=t³+2t⁴、T=1，分别用平方展开积分和 cᵀQc 算 J。
2. 从 p+εη 展开代价，做三次分部积分。写出内部连接的 η′、η″ 系数，解释固定位置为何产生 jerk/snap 连续条件。
3. 对 0→1→2 m、每段1 s、首尾静止且加速度为0，使用教材的两段局部系数，核对 p/v/a/jerk/snap 和总 J。与中间停靠比较。
4. **代码 EXERCISE(ch15-1)**：补全 `starter/jerk_gram.hpp` 的积分条目。

```bash
g++ -std=c++17 -Iexercises/ch15/starter exercises/ch15/check_jerk_gram.cpp -o /tmp/ch15_gram
/tmp/ch15_gram
```

把 include 路径的 `starter` 换为 `solutions` 可核对参考实现。完成后的程序应报告检查通过。

## 2. 直接偏导与完整梯度

5. 对 Tc=q、K=c²，取 q=3、T=2，先直接代入，再通过伴随 Tλ=2c 求两个输入梯度。
6. 对单段位移1 m、T=2 s，算固定系数的 ∂J/∂T、保持端点条件的 dJ/dT，以及二者之差。
7. **代码 EXERCISE(ch15-2)**：补全正时长变量的链式梯度。编译命令与上面相同，将源文件换为 `check_time_gradient.cpp`，输出名换为 `/tmp/ch15_time`。
8. 从 K(T)=720w_J L²/T⁵+w_T T 求最优时长。取 w_J=w_T=L=1，计算数值并检查二阶导数符号。最短时长比它更大时，目标会希望把 T 调向哪里？

## 3. 物理量与移动积分节点

9. 取 p(t)=t⁴/4、P=j²，一段只用1个梯形区间。分别计算近似积分、其时长导数、权重变化项和节点移动项。增加区间数，观察近似积分趋向哪个解析值。
10. **代码 EXERCISE(ch15-3)**：完成 `starter/flat_time.hpp` 中的移动节点时长偏导。

```bash
g++ -std=c++17 -Iexercises/ch15/starter exercises/ch15/check_flat_time.cpp -o /tmp/ch15_flat_time
/tmp/ch15_flat_time
```

11. 一维模型 m=2 kg、c_v=0.5 kg/s、v=1 m/s、a=0.75 m/s²、j=0.2 m/s³。只启用 F*=1 N、w_F=1 的力惩罚，求 F、Ḟ、h_F 和代价对 v/a/j 的偏导。
12. 二维优化变量取 s=(1,0)ᵀ、y=(2,0)ᵀ、H=I。手算 BFGS 更新，并验证 H_new y=s。

## 4. 对照实验

```bash
ros2 run motion2d minco_demo tmp/ch15_minco
/usr/bin/python3 scripts/plot_ch15_minco.py
ros2 run motion2d corridor_demo tmp/ch13_corridor
ros2 run motion2d trajectory_optimization_demo tmp/ch15_optimization
/usr/bin/python3 scripts/plot_ch15_optimization.py
ros2 run motion2d force_planning_demo
```

- 在固定路点与时长下比较停靠和 MINCO 的 J、速度及加速度采样峰值。
- 扰动一个路点或时长，每次重新求系数，用 10⁻⁴、10⁻⁵、10⁻⁶ 三种差分步长检查完整梯度。
- 固定初值，逐次增大走廊权重，记录时长、最大残差、迭代数和终止状态；解释代价下降和残差下降的关系。
- 力约束实验比较1 kg和3 kg的时长与连续力界，结合 v/a/jerk 的时间缩放解释结果。

## 5. ROS 预览

启动 `ch15.launch.py` 并运行 `scripts/check_ch15.py`。在 RViz 观察紫色优化预览、青色路径和走廊。
默认起点 (-8,-8) m、目标 (-7.5,-8) m，核对预览端点和 `preview_only` 状态。预览期间机器人保持当前状态。
将目标坐标系改为 `bad`，观察旧曲线清空；重置时间后再次设置目标。
阅读 `planner_node.cpp`，画出一次回调中栅格、走廊、距离场和曲线的生成顺序，说明共用地图快照的作用。

完成标准：手算结果含中间步骤；三个代码练习通过各自检查；实验记录包含固定参数、梯度比较和约束残差解释。
