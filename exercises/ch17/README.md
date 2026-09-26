# 第17章：从输入解释跟踪误差

先运行教材的圆形/八字参考和阿克曼换道演示，再完成以下练习。每次只改变一个参数，保留同一参考和被控模型。

1. 单轴计算：真实 m=1 kg、c=0.15 kg/s，e=0.1 m、e_dot=-0.2 m/s、a_ref=0.3 m/s²、v=0.5 m/s，kp=4、kd=3。求模型匹配时的请求力、实际加速度、e_ddot；再分别关闭前馈、把估计质量改为0.6 kg。
2. 推导完整的模型失配误差方程。若输入限幅，写出额外项与 requested/applied 的关系。
3. 取 kp=4、kd=5，e(0)=1、e_dot(0)=0，解出两根与两个积分常数。再代入教材临界阻尼解，核对初始条件。
4. **EXERCISE(ch17-1)**：补全 `starter/force_limit.hpp` 的每轴独立限幅。请求 (3,1) N、每轴上限2 N时输出什么？若改为向量范数上限2 N，又会输出什么？画出两种可用输入集合。
5. 从四个边界条件推导时钟速率 q(s)=3s²-2s³，然后积分求 tau。R=2 s 时，求 t=1 s 的 tau、tau_dot、tau_ddot。写出任一路线的位置、速度和加速度。
6. 从阻尼制动力推导指数速度、总滑行距离与速度阈值时间。使用教材参数，计算速度从0.5降至0.01 m/s的时间。检查初始请求力是否饱和后，再使用解析式。
7. **EXERCISE(ch17-2)**：补全 `starter/body_velocity.hpp`。用 yaw=pi/2、机体系速度(0.2,0.3) m/s核对；再用转置旋转还原原向量。
8. 画出阿克曼当前前向、左向与位置误差，说明每一项的符号。给定 s=0.5 m/s、L=0.3 m、s0=0.3 m/s、k_yaw=2、k_lateral=1.5，求教材横向小误差方程中的两个系数。解释停车后横向偏差的修正方式。
9. 固定圆形/八字参考、24 s、50 Hz控制/200 Hz动力学，比较正常前馈、关闭前馈、估计质量0.6 kg、每轴力限0.2 N。报告位置RMS/最大误差、yaw误差、输入峰值、饱和次数及最小净空，分别用误差方程解释。
10. 对同一阿克曼参考只把控制器质量改成真实值的1.3倍，记录误差和输入。再恢复质量，只改变转向反馈增益，观察转角和横向误差。保持规划参考一致。
11. ROS时间实验：运行 `scripts/check_ch17.py`。暂停后等待墙钟1 s，观察参考时间戳；使用 `scripts/check_tracker_boundary.py` 中的独立时钟实验，仅推进 /clock制造断流，比较新的制动力与旧跟踪力。

从仓库根目录编译两个代码练习：

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch17/starter exercises/ch17/check_force_limit.cpp -o /tmp/ch17_limit
/tmp/ch17_limit
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch17/starter exercises/ch17/check_body_velocity.cpp -o /tmp/ch17_velocity
/tmp/ch17_velocity
```

完成后对照 `hints.md` 检查推导，再阅读 `solutions/`。评价重点是能把公式、单位和输入输出对应起来，并通过固定条件的比较解释误差。
