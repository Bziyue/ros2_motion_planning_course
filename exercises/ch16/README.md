# 第16章练习

1. 推导：内部速度/加速度为何只耦合相邻两段？两种消元顺序为何给出同一最小jerk曲线？
2. EXERCISE(ch16-1)：补全2×2 Schur块更新。用独立4×4密集解检查，避免只对照实现公式。
3. 实验：固定时长、端点p/v/a和内部位置，运行本地与锁定上游对照；分别记录误差和构造+能量梯度耗时。

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch16/starter exercises/ch16/check_block.cpp -o /tmp/ch16_block
/tmp/ch16_block
```

评分：块消元40%，相同问题/误差检查30%，清楚解释计时范围和局限30%。演示不依赖未完成练习。
