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

9. ch19-3 配置与闭环实验（代码练习仍为ch19-1/2）：复制 `flat_vehicle.yaml`，运行两种模型×两种定位，参数和命令见 [入口](../../docs/FLATNESS.md)。解释为何同一个物理目标在truth与SLAM下坐标不同；只把评价参数改成slam而不重启launch不能算切换成功。
10. 分别把 vehicle.mass 改为3kg、planning.force_max改为0.5N、planning.steering_max改为0.005rad，每次只改一个量。记录求解状态、认证界、时长、实际指令和跟踪误差，包括所有失败。哪些约束可以通过延时缓解，哪些必须改变几何？
11. 把阿克曼的 `planning.backend` 从spline改成minco，解释为何实际仍选择专用规划器；再对全向模型验证后端变化。查看 `/flat/plan_status` 与节点订阅图，证明选模改变了动力学与控制通道。
12. 运动中发 `/flat/stop`，观察清空参考后使用新状态制动；讨论参考轨迹证书为什么不能直接覆盖制动过程。评分：物理单位/模型对应30%，可复现四组记录40%，失败与零速边界解释30%。
