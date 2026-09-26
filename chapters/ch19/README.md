# 第19章：在线接续与综合导航

先完成参考接续内核，再接地图与ROS导航。
ReferenceSchedule只保存活动曲线和一个未来候选；MPC的sample查询为const，不会因预读未来而提前切换当前参考。
只有实际观测时间advance才激活候选；拒绝积压的第二个候选、过去起点、不匹配p/v/a/yaw与非静止终点。
首个参考接静止锚点；后续接旧参考在未来时刻的p/v/a，不把运动机器人每次当成静止。

```bash
ros2 run motion2d reference_handover_demo tmp/ch19_handover
/usr/bin/python3 scripts/plot_ch19_handover.py
```

演示1.5s交接处p=(.550415,0)m、v=(.823975,0)m/s、a=(.439453,0)m/s²，前缀不变，后段转向(3,1)。
冻结变换同时作用于曲线和每段配置空间区域，yaw随坐标旋转；仅c0受平移。
后续map->odom校正不再输入已冻结的执行曲线。样条接续不是碰撞证书，也不是跟踪稳定性证明。
reset(anchor)会清空两条曲线并保持位置；紧急处理不宣称保持C2。时间回退由ROS边界显式reset。

## 观测快照上的局部规划内核

```bash
ros2 run motion2d replanner_demo tmp/ch19_replanner
/usr/bin/python3 scripts/plot_ch19_replanner.py
```

observedLocalRoute从未来交接点在配置栅格上搜索可达集；目标可达就用A*，否则选邻近未知且更接近目标的自由候选。
未知始终阻塞，前沿窗口覆盖膨胀半径；默认路线按弧长截到2m。贪心探索无完整性保证，no_progress_frontier/no_reachable_frontier不表示整个未知世界必然无路。
replanObserved对同一快照构建走廊、ESDF软项与Spline优化，必须收敛+独立Bézier认证。
优化失败可尝试保持首p/v/a的停点五次段，1/1.5/2/3倍时长也都逐一认证；不合格返回失败，不复用旧优化结果。
结果标明optimized或fallback_stop_segments，并保留optimization_status；控制点惩罚默认开启，速度/加速度证书.7/.8。
手工网格夹具（不是导航成绩）：未知目标局部段，备用/Spline T=7.258/11.424s；绕墙5m前缀T=19.466/12.994s，四组均认证。

## 原子参考接收（ROS）

`NavigationReference` 把 odom 曲线、逐段 float64 顶点区域、地图时刻绑定发布。
`reference.source:=navigation` 接收后用 `/control/accepted_reference` 回传起点纳秒；不连续/非法替换保留原曲线。
`/navigation/stop` 清空曲线/显示/热启动并从当前状态重新计算阻尼制动力。
启动允许 0.03m / 0.02rad 的静止测量容差，运动中仍严格 p/v/a/yaw 接续。

两个终端 source 后运行（不启动仿真器）：
```bash
ros2 run motion2d tracker_node --ros-args -p use_sim_time:=true -p reference.source:=navigation -p control.odometry_timeout:=10.0
python3 scripts/check_navigation_boundary.py
```
检查未来确认、C2 切换、非法替换保留、显式新制动与移动重启拒绝。

## 在线导航与A/B/C切换

```bash
ros2 launch motion2d_bringup ch19.launch.py route:=B seed:=42
ros2 service call /sim/pause std_srvs/srv/SetBool "{data: false}"
# RViz 2D Goal Pose or:
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: -4.0, y: -4.0}, orientation: {w: 1.0}}}"
```

- A：truth + ideal，IMU关闭，200Hz点位置命令；不解释为力驱动机器人。
- B：truth + inertial + MPC，50Hz控制。
- C：lidar/IMU + EKF + SLAM + inertial + MPC，唯一map→odom来自SLAM；无truth adapter。C初始map位置为(0,0)，同一物理目标在map中约(4,4)。

默认三路线均观测建图、720束/10Hz、range6m、地图0.1m；未知格阻塞。
导航5Hz、未来0.15s交接、优化0.04s预算、局部前缀2m、物理半径.2m加跟踪余量.15m。
节点使用采样软惩罚以减轻单段控制点过保守的时长，但仍独立通过同一连续Bézier证书后才执行。
地图更新验证剩余配置区域；控制/规划失败、激光过期.35s、里程计.12s、地图3s或ACK超时均显式制动。
`/navigation/status` 含state、planner_status、optimization_status、planning_seconds、accepted_plans、stop_count。

独立故障检查（不启动simulator、tracker）：
```bash
ros2 run motion2d navigation_node --ros-args -p use_sim_time:=true
python3 scripts/check_navigation_failure.py
```
完整批量对照：`python3 scripts/run_ch19.py --seeds 42 43 44`。
脚本为每次试验单独创建和关闭进程组，默认domain78；不要同时在该domain运行自己的仿真。
详细样本、状态、QP与净空在tmp/ch19_batch；真值仅进入离线评价。
RViz应显示观测地图、注册点云、参考与预测、凸区域；本主机当前GUI限制仍未解除，已完成的ROS检查不替代GUI验收。
