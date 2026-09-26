# 第 11 章：配置空间与 A*

先修第03、07章。已完成独立规划内核与观测地图、真值定位、RViz目标接口。
手工给定栅格用于隔离算法；在线演示仅消费观测地图。

```bash
ros2 run motion2d astar_demo tmp/ch11_astar
/usr/bin/python3 scripts/plot_ch11_astar.py
```

规划只把概率编码<=35且已观测的格子作为原始自由格；不确定和未知默认阻塞，地图外部阻塞。
可配置physical radius与margin，相加一次；unknown_blocked=false仅用于显式乐观实验。
原始障碍是完整方格。输出的每一个自由方格都保证其整个闭区域与原始阻塞方格/外部间距大于radius+margin。
使用精确方格到方格距离构造膨胀模板，接触算阻塞；因此比只对格子中心规划更保守。
半径0时仍会排除与障碍接触的相邻方格，细分网格能减小这种整格证书的代价。

A*选择Manhattan或octile启发式；对角必须同时通过两侧轴向自由格，不能穿角。
路径保留请求的真实起终点，grid_cost仅表示中心图上的最短距离；失败代价为inf，路径为空。
状态有found、invalid_start、invalid_goal、unreachable。

简化使用解析线段/闭方格相交，检查端点、角点接触和沿边情况；不能用建图的普通DDA作为碰撞证明。
实现遍历线段包围框，虽不如supercover DDA快，却容易核对；未来需要性能再保留此基准优化。
输出是折线，不带时间，也不保证曲线化后安全。

演示给定41×31、.1 m栅格，一堵带.7 m门的隔墙，起点(.75,.75)、终点(3.25,2.25) m，margin=0。
半径0/.15/.30与四/八邻接六组：比较可达性、网格代价、扩展数、简化点数。
单元测试另用独立Dijkstra核对30个seed=1111随机栅格，检查未知、边界、窄门、穿角和线段相切。
练习ch11-1补完邻居合法性；常见误区是把障碍格当成无面积点、把简化折线直接称为可执行轨迹。


## 在线观测地图与 RViz 目标

```bash
ros2 launch motion2d_bringup ch11.launch.py
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: -7.5, y: -8.0}, orientation: {w: 1.0}}}'
ros2 topic echo /plan/status
```

默认 ideal 静止、IMU 关闭，等待数帧扫描后目标才有足够自由证据。
RViz 2D Goal Pose 发送到 `/goal_pose`，位置须在 map、z=0；yaw 忽略。
几何路径不会触发机器人运动，第14章才增加连续轨迹执行。
`/planning/grid` 是0/100的配置空间掩码，`/plan/path` 是包含精确起终点的map折线。
规划器只读 `/map`、`/odometry`、`/goal_pose` 与 TF；本配置定位来自真值，地图来自观测。
只支持轴对齐地图；按里程计时间查询 map←odom。失败时发空路径，时间回退清除目标。
5 Hz 合并输入，新地图、新目标或起点移动0.05 m时重算。参数见 `config/ch11.yaml`。

```bash
# Separate terminal, same ROS_DOMAIN_ID; launch with rviz:=false if desired.
/usr/bin/python3 scripts/check_ch11.py
/usr/bin/python3 scripts/plot_ch11_observed.py
# A separate ROS domain with only planner_node:
/usr/bin/python3 scripts/check_ch11_frames.py
```

前一个检查固定reset后0.5 s、100步，保存实际地图与路径到tmp。
后一个独立检查90°旋转及(1,2)m平移、旋转地图拒绝和尺寸错误；勿与完整启动同时向相同话题发布地图。
本次主机完成无界面ROS验收，RViz配置已提供，GUI画面未验收。
ch11-2路径长度练习补充图代价与实际折线长度的区别。
