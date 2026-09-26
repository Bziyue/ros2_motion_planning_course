# 微分平坦与模型切换实验

本扩展对应第04、06、15、16、17、19章。先运行无ROS回调的三个核心例子，再运行观测地图闭环。所有命令均从仓库根目录执行，并先加载主机 ROS 与工作空间：

```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
ros2 run motion2d flatness_demo
ros2 run motion2d force_planning_demo
ros2 run motion2d ackermann_planning_demo tmp/ackermann
```

## 模型判断与控制量

| 模型 | 平坦输出与适用范围 | 实际输入 |
| --- | --- | --- |
| 全向速度 | `(px,py,yaw)`，速度由一阶导数恢复 | odom/body 速度与角速度 |
| 全向惯性 | `(px,py,yaw)`；独立 yaw 不能从二维位置唯一恢复 | odom 力 Fx/Fy（N）与力矩（N·m） |
| 阿克曼自行车 | `(px,py)`；前进、非零速度的局部支路 | 车体纵向力（N）、前轮转向速率（rad/s） |
| 理想瞬移 | 位置赋值没有连续状态方程，不能以此断言微分平坦 | 位姿 |

全向模型的 `F=m*a+c*v`、`Fdot=m*j+c*a`、`tau=I*yaw_ddot+c_omega*yaw_dot` 都在限幅之前计算。受限输入是轨迹可行域，不改变无约束模型的平坦性。教材参考四旋翼平坦映射和 GCOPTER 的 forward/backward 分工；这里是地面水平受力，没有实现三维推力方向、倾角或旋翼分配。

阿克曼状态点选在后轴，碰撞圆盘和共址激光/IMU也以此为中心；radius 是用户选择的保守包络，默认0.4m，wheelbase=0.3m。没有模拟轮胎侧滑或完整侧向轮胎动力学。规划器保留正则几何切向，再安排停车进度，避免在零速处直接计算 `atan2(vy,vx)` 与 `1/|v|`；这不消除任意零速状态的平坦奇异性。

## 一份 YAML 选择整套模型

```bash
mkdir -p tmp
cp ros2_ws/src/motion2d_bringup/config/flat_vehicle.yaml tmp/vehicle.yaml
# Edit tmp/vehicle.yaml, then restart the launch.
ros2 launch motion2d_bringup flat_vehicle.launch.py \
  rviz:=false config:=/home/zdp/ForCodex/ros2_motion_planning_course/tmp/vehicle.yaml
```

编辑顶层 `vehicle`：

```yaml
vehicle:
  model: ackermann       # ackermann or inertial
  localization: truth    # truth or slam
  radius: 0.4            # m, collision envelope
  mass: 1.0              # kg
  linear_drag: 0.15      # kg/s
  force_max: 2.0         # N, physical actuator limit
  wheelbase: 0.3         # m, Ackermann only
  steering_max: 0.6      # rad, Ackermann only
  steering_rate_max: 1.0 # rad/s, Ackermann only
```

这些物理量由 launch 同时传给模拟器和规划/控制节点，显示世界也使用同一个 radius。惯性转动惯量默认 `m*r²/2`，可在 vehicle 下显式写正的 `inertia_z`；radius=0 时必须这样做。参数只在启动读取，改变模型或质量后重启。此文件包含 launch 自己读取的 vehicle 段，**不能直接作为 `ros2 run ... --params-file` 使用**。

`flat_vehicle.ros__parameters` 中的 `planning.*` 是更小的参考轨迹上限：默认速度0.7m/s、力1N；惯性模型还有力变化率2N/s，阿克曼还有转角0.5rad、转向速率0.7rad/s和侧向加速度0.8m/s²。软惩罚采用上限的85%，后验连续界使用完整规划上限。反馈输入最终按物理执行器上限裁剪，不能把参考证书冒充为实际误差、制动轨迹或反馈力变化率的证明。

| vehicle.model | 使用的轨迹与跟踪 |
| --- | --- |
| inertial | `planning.backend: minco/spline`；位置系数/时长的力与力变化率梯度；前馈PD保持起步yaw |
| ackermann | 专用正则几何五次段 + 五次停车进度；切向长度/时长优化；平坦前馈及纵向/航向/横向误差反馈 |

`planning.backend` 只决定全向后端，阿克曼固定用专用规划器。全向目标的朝向不参与位置优化，保持起步yaw；阿克曼使用目标朝向。原第18/19章的全向线性MPC与运动中接续仍通过原章节入口运行，本入口是便于读懂的停止到停止实验，不把线性全向MPC套给小车。

## 发目标、观察、停车

启动默认暂停，在第二个已加载环境的终端：

```bash
ros2 service call /sim/pause std_srvs/srv/SetBool '{data: false}'
# truth localization: move from (-8,-8) to (-6,-7.7), end yaw=0
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: -6.0, y: -7.7}, orientation: {w: 1.0}}}'
ros2 topic echo /flat/status
```

SLAM 初始 map 原点接近首帧后轴位置，同一相对目标改成 `(2.0,0.3)`；当前入口要求初始 yaw=0，以保持估计 odom 与模拟全向施力轴一致。`truth` 使用真值适配器定位，再以扫描建图；`slam` 启动激光里程计、IMU融合、估计适配器和SLAM，无真值适配器。

```bash
ros2 topic pub --once /flat/stop std_msgs/msg/Empty '{}'
ros2 service call /sim/reset std_srvs/srv/Trigger '{}'
```

地图默认未知阻塞；SLAM 第一次自由更新概率0.4还未达到0.35自由阈值，需要后续关键帧。状态 `waiting_observed_free_start` 会等待新地图，不降低阈值。目标须位于当前已观测可达空间；没有前沿探索、倒车或 Hybrid A*，狭窄拐角可能明确失败。

求解只在静止时进行，预算1s；成功后等新里程计，取其时刻+0.3s作为起点，避免把计算期间积压的观测当作现在。轨迹和对应区域冻结到 odom；新地图使区域失效时清空参考并制动。暂停不推进轨迹；reset、非法目标和显式停车清空旧参考。激光/地图/转角编码器失效使用新鲜状态制动；里程计也过期时归零输入，不能反复用旧速度制动导致反向加速。

RViz 可设 `rviz:=true`：预期显示 `/map`、`/cloud/registered`、`/odometry`、`/plan/path`、`/plan/corridors` 与 `/flat/reference`。完整世界/仿真圆盘显示默认关闭，勾选时明确属于真值对照；圆盘含前轮转角标记。已完成无界面ROS验收，未将配置文件检查称作GUI验收。

## 复现实验与代码入口

启动上面匹配的模型/定位后，检查器会主动 reset、推进和暂停仿真：

```bash
/usr/bin/python3 scripts/check_flat_vehicle.py \
  --model ackermann --localization truth --output tmp/flat_ack_truth
```

分别切换两种模型/定位，脚本参数也相应修改。惯性truth用spline、惯性slam用minco。固定seed42，720束/10Hz/测距噪声0.01m，200Hz IMU沿用第06章各轴白噪声；同一相对目标(2,0.3)m，质量1kg。四组均到达，并通过暂停、非法目标、reset、传感器时间/字段、唯一时钟和实际订阅图隔离检查。峰值误差比较同stamp的选定里程计与参考，是跟踪指标，不是SLAM ATE。

实测汇总见 [数据表](../textbook/data/flat_vehicle_validation.csv)。把规划转角上限改为0.005rad，再运行检查器加 `--expect-failure`，该换道案例拒绝发布曲线。失败不是“所有可能路线不存在”的证明，保守证书也可能拒绝真实可行的轨迹。

| 代码 | 教材内容 |
| --- | --- |
| `dynamics/omni_flatness.*`、`ackermann_flatness.*` | 第04章正向物理量与解析VJP |
| `trajectory/trajectory_cost.cpp`、`bezier_bounds.cpp` | 第15/16章系数/时长梯度及全向力界 |
| `trajectory/ackermann_trajectory.cpp` | 第16章几何/进度复合、优化、连续界 |
| `control/ackermann_tracker.cpp` | 第17章局部跟踪 |
| `nodes/simulator_node.cpp` | 第04/06章保持输入、碰撞、采样、理想转角编码器 |
| `nodes/flat_vehicle_node.cpp`、`launch/flat_vehicle.launch.py` | 第19章选模、观测地图、未来起点与执行 |

公开接口位于同路径 include/motion2d 下；解析梯度由中央差分验证，差分不用于实际优化。练习见 ch04-5/6、ch15-3、ch16-3 以及 ch19-3。来源与固定版本见 [REFERENCES](REFERENCES.md)。
