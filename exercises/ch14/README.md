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
