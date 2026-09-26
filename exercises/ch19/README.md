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

5. 局部规划实验：把目标放入未知区、占据墙、地图外和已知不连通区，逐项记录状态；“没有当前可达前沿”为什么不等于证明未知世界无路？
6. 将未来首速度从.15改成2m/s；所有候选必须因证书上限被拒绝。关闭优化时，备用轨迹仍需连续认证，不能绕过验收。

5. ROS实验：运行 `scripts/check_navigation_boundary.py`（启动命令见章节说明），删去候选的一个区域，验证不会收到确认且原参考继续执行。为什么“publish返回”不能当作“接收成功”？
6. 启动容差和运动中的C2交接分别解决什么问题？把0.03m启动容差用于每次接续会产生什么后果？

7. EXERCISE(ch19-2)：补完 `starter/replan_due.hpp` 的周期触发条件。用仿真时间而非墙钟；等待交接时禁止第二候选。
```bash
c++ -std=c++17 -I exercises/ch19/starter exercises/ch19/check_trigger.cpp -o tmp/ch19-trigger
./tmp/ch19-trigger
```
参考答案在solutions，判据包含首次、暂停、到周期前1ns、恰好到周期、待确认和回退。
8. 用相同种子运行A/B/C。分别报告初始对齐ATE、1s平移RPE、跟踪误差、真实圆盘净空、碰撞尝试、规划/MPC耗时、停车次数。不能把真值注入规划或SLAM来提高分数。
