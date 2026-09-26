# 第14章：从路径到五次轨迹

先修04、11、13章。已完成纯算法、消息、观测路线转换与理想连续执行。

```bash
ros2 run motion2d quintic_demo tmp/ch14_quintic
/usr/bin/python3 scripts/plot_ch14_quintic.py
```

TranslationState是同一固定坐标系的p/v/a。QuinticPiece保存正duration及2×6升幂系数，时间为段内秒。
interpolateQuintic用归一化时间求六个边界条件，最终转换为秒制系数。
PolynomialTrajectory检查C²连接，采样闭区间[0,total]；内部恰好连接时刻取右段，终点取末段。
超出时间范围拒绝；等待和终点保持是执行器的策略，不在数学采样器里偷偷截断。
所有平移量与yaw参考分离。系数CSV可以完整重建平移，不隐含朝向。

stopAtWaypoints给每个路点零v/a，T=max(min_duration,段长/nominal_speed)。
这是沿已知线段的单调运动，峰值速度为1.875倍平均速度；重复点为正时长停留。
任意非零连接导数可能让曲线偏离线段，必须另查避障/动力学。

一米静止到静止，T=2/4s的采样速度峰值.9375/.46875m/s，加速度1.44337/.360844m/s²，
jerk7.5/.9375m/s³。第三例有非零连接v/a，两段各4s，C²连续，不要求jerk连续。
测试使用解析缩放、100组seed=1414边界、导数差分、连接单侧、非法时长及重复点。
教材给公式与真实源码；ch14-1补完导数。常见错误：升降幂混淆、全局时间当局部时间、对秒制系数再除T。


## ROS执行

```bash
ros2 launch motion2d_bringup ch14.launch.py
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: -7.5, y: -8.0}, orientation: {w: 1.0}}}'
ros2 service call /trajectory/execute std_srvs/srv/Trigger '{}'
ros2 topic echo /sim/trajectory_status
```

先等待建图/规划成功。默认 model=trajectory，IMU开启，只有/plan/trajectory控制输入。
服务成功表示发布，执行器的accepted/waiting/executing/completed/rejected状态单独报告。
起点须与静止当前状态相同，末端静止，yaw不跳变，执行时刻非过去；当前曲线未结束不替换。
碰撞冻结和reset语义与旧模型相同；以加速度三角界扩大每tick扫掠圆盘，不遗漏曲线鼓出。
trajectory_node只读/plan/corridor_path、/odometry和TF；发布时将map路点固定到odom。
trajectory.nominal_speed=.4m/s、start_lead=.25s，段时长至少.5s。
消息包motion2d_interfaces的Trajectory2D存header/start_time/pieces/yaw，QuinticPiece2D存duration/x[6]/y[6]。
/plan/trajectory_path仅为采样预览，不能代替系数执行。RViz预期洋红色曲线与实际轨迹重合；本机GUI未验收。

默认launch下运行scripts/check_ch14.py，生成tmp/ch14_execution.csv；scripts/plot_ch14_execution.py绘图。
独立时间实验：停止旧launch后，启动相同launch加rviz:=false与config:=$PWD/ros2_ws/src/motion2d_bringup/config/ch14_time_lab.yaml，
运行scripts/check_ch14_time.py。137/7Hz、非整步起终/连接时刻验证解析IMU/激光，包含拒绝替换与碰撞冻结。
