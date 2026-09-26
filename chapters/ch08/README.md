# 第 08 章：二维扫描匹配与激光里程计

先修 01–07。已实现配准内核、局部里程计、ROS 接入与六组传感器回放实验。

~~~bash
source /opt/ros/lyrical/setup.bash
cd /home/zdp/ForCodex/ros2_motion_planning_course/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 run motion2d scan_matcher_demo
colcon test --packages-select motion2d --event-handlers console_direct+
colcon test-result --verbose
~~~

演示把两面墙上的点用已知变换反投影，再只交给 ICP 两组点。
目标为 x=.12 m、y=-.08 m、yaw=.04 rad；分别报告点到点、点到线的状态、
RMSE、关联数量与迭代次数；同时比较零初值与 (.11,-.075,.038) 的邻近初值。
零初值下点到点会落入错误局部极小值，见实际 CSV；这是初值实验的一部分。
算法不接收已知答案、世界几何或 TF。

核心位于 estimation/scan_matcher.cpp。MatchConfig 控制关联门限、Huber 尺度、
最少关联数、收敛容差、最大迭代次数与残差门限，单位见头文件。
点到线法向量来自参考点的邻域 PCA，邻域不足或不呈线状时跳过。

常见错误：变换方向写反；把平移加入旋转雅可比；用到迭代上限当作收敛；
只凭残差小忽略走廊退化。最近邻搜索暂用清楚的双循环，不承诺大点云实时性。
练习、提示和参考解见 exercises/ch08。

## 扫描到局部地图的在线里程计

加载工作区后运行 `ros2 launch motion2d_bringup ch08.launch.py`。无界面验收使用
`rviz:=false`，另一个终端运行 `/usr/bin/python3 ../scripts/check_ch08.py`。
默认点到线配准，参数在 `config/ch08.yaml`；可选 `estimation.metric: point` 做对比。

数据流：`/scan → lidar_odometry → /odometry/estimated → estimated_odometry → /odometry`。
`mapping` 只消费 `/scan` 与采样时刻完全相同的 `/odometry`。配准失败不发布该帧位姿，
不会把过期位姿冒充新测量。下一帧成功即可继续建图；未配对的旧扫描被丢弃。
`/estimation/status` 报告原因，`/cloud/local_map` 显示最近关键帧的参考表面。

第一帧定义 odom 原点，map 暂与 odom 重合。估计器从来不读真值 TF、世界障碍物或真值里程计。
RViz 默认关闭真值世界/机器人，显示观测栅格、当前注册点云、估计轨迹和机体坐标轴。
其圆轨迹从 (0,0) 开始，不能直接叠在世界起点 (-8,-8) 上。真值只在验收脚本中用于首帧对齐。
速度是两次成功扫描之间的割线速度；协方差是固定教学测量尺度，未经统计标定。

局部地图最多 5 个关键帧；平移 .2 m 或转角 .15 rad 时插入，.06 m 方格取质心，PCA 半径 .4 m。
用前两次成功估计预测初值。拒绝帧不更新历史；超过 1 s 没有成功跟踪则 tracking_lost，需 reset。
这不是全局重定位器；重复场景、长走廊和过快运动都可能失败或产生错误的局部极小值。

离线实验（仓库根目录，加载 ROS 环境后）：

```bash
ros2 run motion2d lidar_odometry_demo tmp/ch08_results
/usr/bin/python3 scripts/plot_ch08.py
```

固定世界种子 42、噪声种子 4242、160 帧、10 Hz、1 m 圆参考和 .4 rad/s；
比较 90/360/720 束以及 0/.03 m 噪声。CSV 同时记录接受数、拒绝数、有效帧位置 RMSE 与末帧误差。
绘图在拒绝处画叉，并画出保持旧估计的误差；不能只展示通过的帧。
修改法向量邻域、体素大小与关键帧数，预测耗时和误差变化后复跑。

主机 RViz 已知时间回拨可能触发 TF 缓存异常。自动 reset 验收先关闭 RViz；
完成 reset 并恢复运动后重新打开窗口。不能把 GUI 重启当作估计器重置。

本次已完成无界面 ROS 验收；桌面截图中的 OpenGL 视口未成功捕获，未将 RViz 画面列为通过。
