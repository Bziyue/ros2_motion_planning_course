# 第19章：接续旧参考，而不是每次从零开始

1. 理解：定位误差、规划可行性与跟踪误差分别属于哪一层？地图回环校正为什么不应重写正在执行的odom参考？
2. EXERCISE(ch19-1)：补全matching_boundary.hpp，验证同一未来交接时刻的p/v/a。只比较位置会遗漏什么？
3. 实验：运行reference_handover_demo，1.5s交接处应保留非零速度；把新轨迹首速度清零，必须拒绝且旧曲线继续有效。
4. 在MPC预测中先读取未来，再读取当前：未来查询不能提前激活待执行计划。

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iros2_ws/src/motion2d/include -Iexercises/ch19/starter exercises/ch19/check_boundary.cpp -o /tmp/ch19_boundary
/tmp/ch19_boundary
```

评分：分层误差解释30%，p/v/a与坐标变换40%，实验和失败行为30%。
