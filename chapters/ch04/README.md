# 第 04 章：机器人模型

先修 01-03。机器人中心状态包含位置、速度、加速度、yaw、角速度和角加速度，平移导数在 odom 系，全部使用 SI 单位。已实现理想到点与速度模型；受力惯性另作独立功能。

## 运行理想到点

工作空间重新构建、加载 install 后：

~~~bash
ros2 launch motion2d_bringup ch04.launch.py model:=ideal
~~~

RViz2 的 2D Goal Pose 工具向 `/command/pose` 发目标。在空地点击并拖动指定 yaw；也可以在已加载 ROS 环境的另一终端：

~~~bash
ros2 topic pub --once /command/pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: -7.0, y: -8.0}, orientation: {w: 1.0}}}"
~~~

机器人从 (-8,-8) 到 (-7,-8)，留下实际接受的运动轨迹。这是经过几何检查的位置赋值，不是沿参考轨迹连续飞行。理想到点的速度/加速度字段为未使用的零值，后续 IMU 模拟不能使用此模式。

`rviz:=false` 为无界面模式，`config:=/绝对路径/ch04.yaml` 更换配置；同一时间只运行一章。世界参数放在 YAML 的通配节点段，由仿真与显示共用。节点的 map、odom 在本章重合，TF 来自真值，尚未引入定位算法或 Odometry 消息。

## 时间、碰撞与输入约定

- 唯一 `/clock` 来自 simulator，默认 dt=0.005 s。复用 `/sim/pause`、`/sim/step`、`/sim/reset` 服务。
- 命令在下一个 tick 执行；暂停时接收命令不会偷偷移动。reset 清空命令与轨迹，回到初始状态、t=0、暂停。
- 位姿只接受 odom/map、有限二维坐标、平面单位四元数。时间戳 0 表示下一个 tick；非零时间戳必须位于当前仿真时刻的前 command_timeout 秒到后一个 dt 之内。
- 圆盘从旧位置到新位置形成胶囊形，与圆、凸多边形、边界进行整段检查；相切也是碰撞，不用稀疏采样代替检查。
- 检查失败时状态和时间停在上一个接受的 tick，状态为 `collision_predicted`，圆盘变红；本次试验结束，reset 后继续。没有把撞入障碍的状态推回去，也不假装模拟了接触后的运动。
- `radius=0` 是几何质点。RViz2 用直径 0.04 m 的小点保持可见，该点不参与碰撞。
- `/sim/pose`、`/sim/velocity`、`/sim/acceleration` 为调试用真值；后续 SLAM 不订阅它们。姿态跳变不能产生物理有效的 IMU。

## 速度模式

~~~bash
ros2 launch motion2d_bringup ch04.launch.py model:=velocity
ros2 topic pub -r 20 /command/velocity geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.5}, angular: {z: 0.3}}}"
~~~

两个命令在分别加载环境的终端执行。该模式只订阅 /command/velocity，RViz2 的位姿目标不驱动它。odom 表示固定方向的平移速度；base_link 表示随圆盘朝向旋转的速度。默认平移速度模长不超过 1 m/s，角速度绝对值不超过 1 rad/s。

命令在模拟时间中保持，默认 0.5 s 未刷新则切为零速度（tick 边界判定），报告 command_timeout；暂停不会消耗有效期。非零时间戳的有效期从消息时间算起，零时间戳从接收的模拟时刻算起。重置会清空命令。惯性模型以后超时归零的是力，不能把此处的瞬时停下照搬过去。

odom 速度的平移是直线；机体系恒速度和角速度用精确圆弧积分。碰撞将圆弧到弦的最大偏差上界 |omega|*|v|*dt²/8 加到半径上，再检查胶囊；这是保守检查，可能拒绝本来安全的窄缝。命令切换时速度仍可跳变，因此不用于 IMU 融合教学。

运行 `/usr/bin/python3 ../scripts/check_ch04.py --model velocity` 核对输入隔离、限速、机体系转换、超时和重置。

## 读代码与验收

| 入口 | 职责 |
| --- | --- |
| sim/robot_model.hpp（include 下） | 普通状态值类型 |
| sim/ideal_model.cpp | 理想位姿赋值，清除未使用的导数 |
| sim/swept_collision.cpp | 连续直线扫掠几何 |
| nodes/simulator_node.cpp | 命令、唯一时钟、TF、状态与轨迹显示 |
| ros/world_parameters.hpp（include 下） | 显示与碰撞共用参数读取 |

启动默认场景后，在工作空间执行 `/usr/bin/python3 ../scripts/check_ch04.py`。脚本检查暂停后命令不提前执行、单步时间、错误帧、碰撞冻结、reset 和唯一时钟。纯算法测试覆盖跨越圆、薄墙、接触、边界、原地与质点情况。

运行画面应显示世界、圆盘及移动后的轨迹，Fixed Frame 为 map。若目标没有执行，先看 `/sim/status`，确认未暂停、无碰撞锁存，并检查输入模型和 frame。后续激光、IMU 在第 05/06 章实现。
