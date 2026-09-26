# 第18章：约束也要预测

1. 推导：从m dv/dt=F-c v积分得到零阶保持A/B；说明c趋于0时如何回到双积分模型。
2. EXERCISE(ch18-1)：补齐input_difference.hpp，构成输入增量矩阵D。为什么第一块还要减上一次实际输入？
3. 固定八字参考与物理模型，比较N=10/20/40、50/20Hz；区分预测步数N与实际前瞻时间N*h。
4. 把每轴力限制到.35N，观察速度/输入变化率。解释为什么PD对比组只保证力上限，而不自动满足MPC的其他约束。
5. 求解器max_iterations=1与max_wall_seconds极小值分别产生什么？记录新制动力，不能继续旧U[0]。

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch18/starter exercises/ch18/check_difference.cpp -o /tmp/ch18_difference
/tmp/ch18_difference
```

评分：离散模型/单位30%，增量矩阵30%，同条件数据与失败解释40%。完整演示不依赖习题答案。
