# 第14章练习：位置之外还有速度和加速度

1. 推导：起点0、终点1、两端v=a=0，设s=t/T，求10s³−15s⁴+6s⁵。
   求s=.5处速度；解释T翻倍时速度、加速度和jerk分别缩放多少。
2. 代码：完成 EXERCISE(ch14-1)，升幂系数计算一阶、二阶导数。
3. 实验：quintic_demo输出T=2/4s及C²连接案例；比较采样峰值，绘图确认连接处p/v/a连续。
   进一步指定非零中间速度，说明为何“经过安全路点”无法证明曲线安全。

```bash
g++ -std=c++17 -Iexercises/ch14/starter exercises/ch14/check_derivatives.cpp -o /tmp/ch14_derivatives
/tmp/ch14_derivatives
```

评分：幂次与因子50%，单位/时间缩放30%，实验和安全解释20%。starter不影响完整演示。

4. ROS时间实验：使用ch14_time_lab.yaml运行check_ch14_time.py。
   起始.203s、连接.661s、终止1.298s均不对齐5ms tick；传感器137/7Hz。
   解释“把IMU时间戳写对、但使用tick末状态”为什么仍错；比较解析加速度与每条消息。
   再发送穿墙曲线，验证时间和所有传感器停在碰撞前，而不是跳过障碍继续执行。
