# 第17章：先知道机器人为什么跟不上

1. 推导：理想模型、无饱和且质量阻尼匹配时，将F=m(a_ref+kp e+kd e_dot)+c v代入动力学，得到误差方程。
   去掉a_ref会多出哪个强迫项？增大kp是否总能补偿力限幅？
2. EXERCISE(ch17-1)：补全每轴独立限幅；不能使用向量范数缩放替代本章的执行器约束。
3. 实验：相同圆形/八字、24s、50Hz控制/200Hz动力学，比较前馈、关闭前馈、质量估计.6kg、每轴力限.2N。
   报告RMS/最大位置误差、yaw误差、输入峰值和最小墙面净空。理想模式无物理力输出，不把NaN当0N。

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch17/starter exercises/ch17/check_force_limit.cpp -o /tmp/ch17_limit
/tmp/ch17_limit
```

评分：误差方程40%，限幅和物理单位30%，固定同参考的实验与解释30%。练习不影响完整演示。
