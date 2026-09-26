# 第 07 章：真值里程计、激光点云与观测建图

先修 01-06。当前完成真值里程计、扫描点云与观测占据栅格；分辨率/噪声独立评估在下一功能提交中完成。

## 先明确本章已知什么

本章已知机器人真值位姿，用传感器观测建图；它不是 SLAM。建图节点只读取 /scan 与统一的 /odometry，不能读取世界几何或 /sim 调试数据。完整几何仅用于独立评估。

| 输出/边 | 发布者 | 含义 |
| --- | --- | --- |
| /ground_truth/odometry | simulator | world / ground_truth_base，原子真值消息，不广播这条真值 TF |
| /odometry | truth_odometry | odom / base_link，本章明确选择真值 |
| map → odom | truth_odometry | 固定单位变换 |
| odom → base_link | truth_odometry | 每个真值状态/采样时刻的动态变换 |
| base_link → laser、imu_link | simulator | 固定圆心外参 |
| /clock | simulator | 唯一模拟时间 |

world、map、odom 在本章使用同一原点与方向。起点 (-8,-8) 不改成 (0,0)，以后要重新定原点必须显式做坐标变换。没有把 world/ground_truth_base 加进 TF 树，防止估计器意外读取真值定位。

Odometry 的 pose 用 header.frame_id，twist 用 child_frame_id。内部世界速度先乘 R(yaw)^T，再写入消息；位置、速度和时间来自同一个 State2D，不能拼接两个不同回调时刻的 /sim/pose 与 /sim/velocity。真值协方差为零代表这个确定性仿真状态，不是实际定位精度承诺。

## 运行与验收

从 ros2_ws 构建并加载 install 后：

~~~bash
ros2 launch motion2d_bringup ch07.launch.py rviz:=false
# 另一终端，同 ROS 环境
/usr/bin/python3 ../scripts/check_ch07_odometry.py
~~~

默认 reference 圆轨迹，IMU/雷达继承第 06 章。去掉 rviz:=false 可显示里程计箭头、机器人与扫描；主机 RViz 的 reset/关闭异常仍见 docs/VALIDATION.md，自动重置验收建议先不启动 GUI。

ch07.launch.py 强制 simulator.publish_truth_tf=false，适配器独占运动 TF。直接运行模拟器时默认 true，以兼容第 04-06 章；不要手动同时启动旧场景与适配器。当前仅实现 truth 选源，SLAM/estimated 尚未实现。

## 代码与练习

- src/ros/odometry_messages.cpp：truthOdometry，世界位姿与机体速度。
- src/nodes/truth_odometry_node.cpp：显式 truth 适配与 TF 所有权。
- src/nodes/simulator_node.cpp：在 tick 末端和传感器采样时刻发布原子真值。
- test/test_odometry.cpp：手算旋转、时间、帧和协方差。
- [练习 ch07-1](../../exercises/ch07/README.md)：机体速度转换。

暂停时 /odometry 与 TF 有同一时间戳的心跳，观测处理不能把它当成新运动或重复地图证据。reset 从 t=0 开始新试验，下游消费者需要清理时序历史。

## 扫描点云

核心为 mapping/scan_projection.cpp，消息转换为 ros/mapping_messages.cpp，消费者为 nodes/mapping_node.cpp。LaserScan 只保留有限且量程内的回波；两个 PointCloud2 输出分别使用 laser 与 odom，XYZ float32、z=0、12 字节/点，保持原始采样时间。外参固定为圆心单位变换。registered 指位姿变换，尚不是扫描匹配。

从 ros2_ws 运行 `/usr/bin/python3 ../scripts/check_ch07_cloud.py`，检查扫描、里程计和点云逐点一致。另在独立域的两个终端分别运行 `ROS_DOMAIN_ID=47 ros2 run motion2d mapping_node` 与 `ROS_DOMAIN_ID=47 /usr/bin/python3 ../scripts/check_ch07_join.py`，验证两种到达顺序和拒绝陈旧位姿。不要在这个测试域同时启动模拟器。

每个扫描按整数纳秒精确配对；缓存上限 1000 条位姿、20 帧扫描，缺少位姿时等待，扫描溢出告警丢弃。正时间重复帧不重复处理；t=0 扫描专用于启动/reset，连续在零时刻 reset 也刷新首帧。reset 后先保持暂停，等待新 t=0 扫描和里程计，再恢复；任意乱序回放/无屏障的多试验混流不在本章范围内。雷达和里程计时间回退会清理本地历史。

RViz 的 Registered scan 显示当前帧观测点，不累积全局点云。Simulation world 是可关闭的真值视觉对照，不是 mapping 输入。/map/cloud 留给后续关键帧重建，不在本章发布。练习 ch07-2 与投影公式对应。

## 观测占据栅格

核心 mapping/occupancy_grid.cpp 不接收 World2D。默认地图 220×220 格、0.1 m/格，左下角 (-11,-11)；这是存储范围，不是提前知道世界障碍。有限回波：DDA 经过格给自由证据，端点格给占据证据；+inf 清到 range_max 但不造占据端点；NaN/-inf/越界有限值整束跳过。图外端点先裁剪，不能把裁剪位置当成障碍。起点在图外则拒绝地图更新并告警。

默认 log-odds 命中概率 .7、穿过 .4，先验 .5，限制 ±4；每帧每格至多更新一次、命中优先。unknown=-1，其余为四舍五入的概率×100；评估用 <=35 free、>=65 occupied，中间 uncertain。未来规划默认将未知与不确定阻塞，地图概率不是 ESDF 距离。

/map 使用 map 帧、最后扫描时间、可靠 transient-local 深度 1；map_load_time 为本轮首次成功插入时间。地图随扫描时间回退清空，再插入 t=0 首帧。运行 `/usr/bin/python3 ../scripts/check_ch07_map.py` 验证消息、未知/自由/占据、暂停、晚加入与重播。改变 mapping.resolution 时同时调整 width_cells/height_cells，才能维持相同物理范围。

RViz 默认关闭 Simulation world (truth comparison)，显示灰色未知区、已观测自由域、占据轮廓和橙色当前回波。起点附近的一米圆周只能看到世界一部分；未知目标尚不能直接用全图路径规划。DDA 与 log-odds 公式、对应源码和练习 ch07-3 见教材。
