# 第16章练习：块消元、连续界与小车规划

## 1. 从边界求样条

1. 在 T=1 时，用单位端点输入构造左、右速度两列，核对 `(0,1,0,-6,8,-3)` 和 `(0,0,0,-4,7,-3)`。将它们代入 Q，求 R 中对应的条目。
2. 对 0→1→2 m、每段1 s、首尾 v/a=0，组装内部节点的2×2方程。求出 v₁、a₁，再还原两段系数，与第15章手算对照。
3. **EXERCISE(ch16-1)**：补全 Schur 块。先取 H₀=H₁=2I、B₁=−I、r₀=(1,0)ᵀ、r₁=0，手算两次代入，再运行检查。

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch16/starter exercises/ch16/check_block.cpp -o /tmp/ch16_block
/tmp/ch16_block
```

4. 从六个端点约束对时间求导，解释 E_T=−E M_T E。指出 M_T 的哪三行非零，以及它们各对应哪一阶导数。
5. 对静止到静止1 m、2 s曲线，用起点与终点分别计算完整能量时长梯度，再与 −3600/T⁶ 对照。

## 2. Bézier 表示与整段界

6. **EXERCISE(ch16-2)**：补全幂次到 Bézier 转换的二项式比例。使用上述编译命令，将输入改成 `check_bezier.cpp`，输出改成 `/tmp/ch16_bezier`。
7. 将 p(t)=1+2t+3t²、T=2 转为五次控制点。用 Bernstein 权重求 s=0、1/2、1 的值，与 t=0、1、2 的原多项式比较。
8. 对二次控制点 (0,0)、(1,2)、(2,0)，手算二分三角形，写出左右两段控制点。取左半段参数 r=1/2，分别由新控制点和原曲线 s=1/4 计算位置。
9. 对1 m、2 s静止到静止曲线，算未细分速度上界与解析峰值。若速度限制1 m/s，两者分别说明什么？细分后重新求界。
10. 一维 p=t²/2、T=2、m=2 kg、c_v=0.5 kg/s，算力和力变化率的五次控制点。说明为何这个例子的控制点范数上界恰好等于真实峰值。

## 3. 阿克曼几何与时间

11. 固定几何路径和 r，推导 F=m(a_g d²+n_g e)+c n_g d 对 T 的偏导，其中 d=h′/T、e=h″/T²。**EXERCISE(ch16-3)**：完成 `starter/warp_force.hpp`。

```bash
g++ -std=c++17 -Iexercises/ch16/starter exercises/ch16/check_warp.cpp -o /tmp/ch16_warp
/tmp/ch16_warp
```

将 include 路径中的 `starter` 换为 `solutions` 可核对参考实现。

12. 几何保持不变、时长翻倍，求速度、惯性力、阻尼力、转角、转向速率、侧向加速度的比例。解释为何转角超限要先改变路径形状。
13. 导数控制点 V₀=(2,0)、V₁=(2,1)、V₂=(3,0)，弦单位方向 e_c=(1,0)。求 n_min 和 n_max，证明这个凸包中的切向都非零。若把 V₀ 换成 (−1,0)，这次投影检查会怎样变化？
14. 由 q′、q″ 的凸组合展开点积/叉积，说明为何枚举所有控制点对能得到整体上界。对曲率求导，找出 n_min 的三次方和五次方从何而来。

## 4. 运行与观察

```bash
ros2 run motion2d spline_demo tmp/ch16_spline
bash scripts/benchmark_ch16.sh
/usr/bin/python3 scripts/plot_ch16_spline.py
ros2 run motion2d bezier_demo tmp/ch16_bezier
/usr/bin/python3 scripts/plot_ch16_bezier.py
ros2 run motion2d ackermann_planning_demo tmp/ackermann_planning
```

- 比较 MINCO/Spline2D 时固定边界、内部路点和时长，记录系数误差与构造加能量梯度的耗时。改变段数，观察增长比例。
- 固定同一条曲线，把细分深度从0增到5，记录界与耗时。重新采样曲线，检查表示转换和细分保持原值。
- 固定阿克曼几何，延长时长，比较转角界与动力学界。再减小允许转角，观察几何检查的结果。

ROS：启动 `ch16.launch.py`，先观察候选，再调用 `/trajectory/execute`。运行 `scripts/check_ch16.py`，核对接收系数在每个 tick 的 p/v/a。
让候选过期、发送非法坐标系目标、重置时间，观察服务和候选状态。画出候选局部时间变为绝对执行时间的过程。

完成标准：三个代码练习通过检查；手算包含中间步骤；实验表保留固定参数、误差、约束界和计时范围。
