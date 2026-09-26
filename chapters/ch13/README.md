# 第13章：二维凸安全走廊

先修11–12章。完成走廊内核、给定栅格实验及 ROS 观测地图接入。

```bash
ros2 run motion2d corridor_demo tmp/ch13_corridor
/usr/bin/python3 scripts/plot_ch13_corridor.py
```

输入为已经膨胀的PlanningGrid及原始A*邻格折线。每个线段先取完整种子格包围框；整框无阻塞才扩张。
每侧最多扩展max_extension/resolution的整数格，轮流检查左/右/下/上新增边；不宣称最大体积。
边界向内缩1e-6格，避免浮点接触；边界上的输入点种子包含两侧格子，不能被缩到走廊外。
长对角简化路径可能安全而包围框有障碍，应保留原始A*路径；失败状态blocked_seed_box不返回部分走廊。

每个CCW凸多边形转换为单位外法向A*p<=b，b单位m。不能只检查顶点或边界：内部也可能包住障碍。
regionIsFree按闭半平面逐个裁剪候选阻塞方格，非空包括擦边/擦角就拒绝，同时拒绝地图外部。
可选merge_convex合并相邻区域的凸包，只有整个凸包验证通过才替换原区域；近共线数值候选无效时保留安全旧区域。
这是从局部矩形到一般凸多边形的简单方法，不是全局最大体积走廊分解。

相邻走廊必须有正面积交叠；取交集多边形顶点的平均值作为连接点。
返回regions.size()+1个waypoints，段i的两个端点属于region[i]，直线因此整段包含于该凸区域。
此结论不自动扩展到经过同样路点的五次曲线，之后必须验证Bezier控制点等连续条件。
半径已在PlanningGrid处理，不再扣除。

实验：61×41、.1m、带顶端绕行空间的给定隔墙，半径.15m、裕量0，
起点(.75,.75)终点(5.35,.75)，max_extension=.6m。
不合并/合并分别得到10/3区域、40/19顶点、11/4路点，最小相邻交叠约.06m²。
测试覆盖单位法向、退化交叠、内部障碍、接触、非矩形凸包、转弯路线、1/.1m尺度及失败清空。
练习ch13-1实现边到半空间。常见错误：重复膨胀、仅检查顶点、允许共点交叠、将路径安全当成曲线安全。


## ROS 观测地图

```bash
ros2 launch motion2d_bringup ch13.launch.py
# 同一 ROS_DOMAIN_ID、已 source 的另一个终端
/usr/bin/python3 scripts/check_ch13.py
/usr/bin/python3 scripts/plot_ch13_observed.py
```

planner 在同一配置图快照上完成 A* 与走廊；原始邻格路径直接传入构造器。
/plan/path_raw 是原始路径，/plan/path 是简化路径，/plan/corridor_path 是区域连接点；
/plan/corridors 是闭合 LINE_STRIP，先 DELETEALL 再画新区域。Marker 仅显示，算法使用 ConvexRegion。
/plan/status 和 /plan/corridor_status 分别描述两个阶段。失败/reset 清空；新目标不能直接移动机器人。
配置 corridor.enabled=true、merge_convex=true、max_extension=.6m。
路径/区域共用起点里程计时间和 map frame；地图掩码仍保留地图采样时间，不是轨迹执行时间。
RViz 预期显示配置图、走廊边框和连接点。主机已通过无界面 ROS 检查，GUI 限制见验收记录。
固定 .5s 的真实观测中，(-8,-8)→(-4,-1) 得到73原始点、4走廊；脚本以独立 SAT 逐格检查安全。
