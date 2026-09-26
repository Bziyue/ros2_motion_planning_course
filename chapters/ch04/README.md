# 第 04 章：机器人模型

先修 01-03。机器人中心状态包含位置、速度、加速度、yaw、角速度和角加速度，平移导数在 odom 系，全部使用 SI 单位。本章已实现理想到点、速度控制与受力惯性，三个功能分别提交。

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

## 受力惯性模式

~~~bash
ros2 launch motion2d_bringup ch04.launch.py model:=inertial
# 在第二个已加载环境的终端施加 odom 系力；停止发布后观察滑行。
ros2 topic pub -r 20 /command/wrench geometry_msgs/msg/WrenchStamped \
  "{header: {frame_id: odom}, wrench: {force: {x: 1.0}, torque: {z: 0.02}}}"
~~~

该模式只订阅 /command/wrench，输入是 N 和 N·m。力固定在 odom 系，与圆盘朝向无关；机器人可以横向受力。这是全向平面模型，可用于后来线性 MPC，未模拟差速轮或四旋翼姿态/推力耦合。推荐主线保持此模型；差速扫地机器人作为附录再引入非完整约束。

运动方程为 m*v_dot=F-c_v*v、I_z*omega_dot=tau-c_omega*omega。每个 dt 内保持输入，用零阶保持解析离散；零阻尼时退化为 p+=v*dt+a*dt²/2、v+=a*dt。加速度由施加的力和当前速度计算，未来可交给 IMU 模拟器。

参数在启动时读取；修改 ch04.yaml 并重启，不用运行中的 param set 改动物理模型。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| mass | 1.0 | kg，正值 |
| inertia_z | -1.0 | -1 表示自动 m*r²/2；也可显式给正惯量 |
| linear_drag | 0.0 | kg/s，非负 |
| angular_drag | 0.0 | kg·m²/s，非负 |
| force_max | 2.0 | 每个平面轴的对称力上限，N |
| torque_max | 0.2 | 偏航力矩绝对值上限，N·m |

radius=0 时惯性模式需要显式的正 inertia_z，不能从零半径推得零惯量。速度模式的 speed_max/yaw_rate_max 不用于惯性模式；直接截断惯性速度会悄悄破坏物理方程。

超时归零的是力与力矩；无阻尼时继续匀速滑行，有阻尼时按指数衰减。制动要给反向力，持续太久会反向加速。碰撞仍冻结试验，保留最后接受的状态；曲线检查增加 |a(0)|*dt²/8 余量，避免只看端点或直线弦。

~~~bash
/usr/bin/python3 ../scripts/check_ch04.py --model inertial
ros2 run motion2d inertia_demo
~~~

第一项检查默认零阻尼配置的输入隔离、力/力矩限幅、自动惯量、暂停、超时滑行、反向力制动和 reset。仅把 mass 改为 2.0 时可加 `--mass 2`；其他参数变更需按实验的解析结果调整验收。

第二项是不依赖 ROS 回调的核心模型实验。1/2 kg、dt=0.02/0.005 s、1 N 力作用 1 s，并从相同的 1 m/s 初速制动。1 kg 的位移/末速为 0.5 m、1 m/s；2 kg 为 0.25 m、0.5 m/s。同速制动距离分别 0.5/1.0 m。

复现教材图表：

~~~bash
# 从仓库根目录，先加载 ROS 和 install 环境
ros2 run motion2d inertia_demo > textbook/data/ch04_inertia.csv
make -C textbook
~~~

LaTeX 的 pgfplots 直接读取这份 C++ 实验 CSV 绘图，不需要 Python 绘图依赖。

## 读代码与验收

| 入口 | 职责 |
| --- | --- |
| sim/robot_model.hpp（include 下） | 普通状态值类型 |
| sim/ideal_model.cpp | 理想位姿赋值，清除未使用的导数 |
| sim/velocity_model.cpp | 恒速度直线/圆弧、速度限制 |
| sim/inertial_model.cpp | 力/力矩限幅、质量与阻尼的解析离散 |
| sim/swept_collision.cpp | 连续直线扫掠几何 |
| nodes/simulator_node.cpp | 命令、唯一时钟、TF、状态与轨迹显示 |
| ros/world_parameters.hpp（include 下） | 显示与碰撞共用参数读取 |

启动默认场景后，在工作空间执行 `/usr/bin/python3 ../scripts/check_ch04.py`。脚本检查暂停后命令不提前执行、单步时间、错误帧、碰撞冻结、reset 和唯一时钟。纯算法测试覆盖跨越圆、薄墙、接触、边界、原地与质点情况。

运行画面应显示世界、圆盘及移动后的轨迹，Fixed Frame 为 map。若目标没有执行，先看 `/sim/status`，确认未暂停、无碰撞锁存，并检查输入模型和 frame。后续激光、IMU 在第 05/06 章实现。

本机 RViz2 的时间回退后退出崩溃在本章也有观察；运行显示与 ROS 检查正常，根因仍未确定。可按第 03 章的方法独立启动 RViz2 并分开关闭；详见 [验收记录](../../docs/VALIDATION.md)。
