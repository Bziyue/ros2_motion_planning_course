# 接口与建模草案

状态：v0.1，随批准大纲进入实施。这里规定课程将使用的语义，实际已实现范围见 [PROGRESS.md](PROGRESS.md)。优先标准消息；实际需要第二种实现或跨节点传输时才增加抽象。

第 04 章落地约定：simulator 独占时钟，发布 /sim/pose、/sim/velocity、/sim/acceleration 真值调试消息。此章 map 与 odom 重合，真值 TF 暂由 simulator 发布；第 07 章才拆出里程计选源适配器。理想到点在下一 tick 执行，失败扫掠冻结上一个有效状态和时间，/sim/status=collision_predicted，必须 reset 开新试验。不把这个调试入口供给后续 SLAM。

## 1. 模块与数据边界

```text
世界几何 → 机器人仿真 → 激光/IMU → 定位 → 栅格/ESDF → 路径/轨迹 → 控制 → 仿真
                 └→ 真值里程计 → 真值模式选源或独立评估
```

世界生成、CPU 雷达、IMU、机器人积分可以先在一个 simulator 节点里按仿真时刻调用，减少初学者要理解的异步行为。估计、地图、规划和控制按课程进度逐步加入。算法是独立 C++ 函数/小类，节点只处理参数、消息和调度。

几何真值由仿真与独立评估持有。SLAM 和默认规划器不接收世界障碍物列表。显式 `oracle_map` 演示使用完整地图时，在 launch 名称、RViz2 标题和实验记录中注明。

计划中的 C++ 值类型：`Pose2D`、`State2D`、`World2D`、`Scan2D`、`Grid2D`、`Esdf2D`、`Corridor2D`、`PolynomialTrajectory2D`。不建立全系统的抽象基类树；雷达 CPU/CUDA 只共享一个清楚的输入/输出约定。

## 2. 状态、外形和控制输入

状态为 `p=(x,y)`、`v=(vx,vy)`、`yaw`、`omega`；`p/v` 在世界或所声明的惯性坐标系中，`yaw` 逆时针为正。SI 单位为 m、s、rad、kg、N、N·m。

- **圆盘**：中心是状态点，`radius` 是碰撞半径。雷达与 IMU 默认在圆心，与 `base_link` 对齐。
- **理想到点**：目标位姿只用于几何规划演示，检查两位置之间的扫掠圆盘；不是任意穿障碍的瞬移。IMU 停用，不能与融合 SLAM 组合。
- **理想连续执行**：输入含时间的参考 `p/v/a`，从轨迹解析求导得到 IMU 的运动真值；位置、速度与加速度来自同一个参考，不对跳变位置做差分。
- **速度模式**：输入 `TwistStamped`，帧明确为 `odom` 或 `base_link` 并转换。瞬时速度跳变不适合 IMU 教学；传感器融合实验使用光滑参考或惯性模式。
- **惯性模式**：输入 `WrenchStamped`，平面力在连续的 `odom` 系表达，由仿真适配器转入内部世界系；`torque.z` 为偏航力矩。仿真已知的坐标变换只用于 plant，不反馈给估计器。

惯性方程：

```text
p_dot     = v
v_dot     = (F - c_v v) / m
yaw_dot   = omega
omega_dot = (tau_z - c_omega omega) / I_z
```

参数 `m > 0`、`I_z > 0`；平面阻尼 `c_v` 单位 kg/s，转动阻尼 `c_omega` 单位 kg·m²/s。采用明确的离散方法，教材推导与仿真、MPC 的离散模型保持一致；第 04 章先从零阻尼恒力解析解验证。

理想命令、速度和力使用不同话题，启动时仅启用对应输入；不把同一个 `cmd_vel` 偷换成力。期望加速度是控制器内部量，转换为力时说明质量、阻尼补偿与饱和。

## 3. 随机世界与碰撞

基础世界为固定边界内的静态圆与凸多边形。多边形以逆时针顶点记录，由随机点凸包生成，检查面积、边长与退化。参数包括数量、大小区间、边数范围、边界、种子、起终点净空和可选连通性筛选。

固定种子与参数可重复，几何、激光噪声、IMU 噪声用分别派生的随机流，改变束数不应同时改变世界。记录最终参数和种子，不因生成失败而偷偷重新抽取。

碰撞依据实际圆盘与障碍几何。仿真报告接触并结束当前试验或按明确策略制动；不悄悄把位置推回安全点。轨迹检查考虑圆盘扫掠和连续时间误差；离散检查必须给出步长/速度导致的保守余量。

## 4. 激光雷达契约

主线模拟单平面、均匀角度的 N 束扫描，“束数”不是三维雷达的垂直通道数。完整 360 度采用 `Delta_alpha=2*pi/N`，`i=0..N-1`，`angle_max=angle_min+(N-1)*Delta_alpha`，首末射线不重合。非整周视场使用端点包含的 `FOV/(N-1)`，并明确需要 `N>=2`。

单束真值是射线与全部障碍/边界的最近非负交点，雷达不与机器人自身碰撞。默认视场 360 度，可配置束数、视场、角起点、频率、量程、距离噪声与后端。

消息为 `sensor_msgs/LaserScan`，frame 为 `laser`：

- `ranges[i]` 单位 m；有效回波加入可选独立高斯距离噪声。
- 量程内没有命中记 `+inf`；近于 `range_min` 的真实命中或噪声后越界的读数记 `NaN` 并跳过建图，避免把近障碍或异常回波当自由空间。
- 不使用强度模型时 `intensities` 为空。
- **基础 snapshot 模式**：同一仿真时刻发出全部束，`time_increment=0`，`scan_time=1/rate`，首束时间也是整帧时刻。
- **进阶 rolling 模式**：逐束采样，`time_increment=scan_time/N`，header 为首束采集时刻，整帧采完再发布。每束使用自己的 pose，去畸变方法与扫描模型同时启用。
- CPU/CUDA 输入一致；确定性对比先关噪声。加噪对比使用同一组事先生成的样本，不能把不同随机序列误认成几何误差。

## 5. IMU 契约

`sensor_msgs/Imu`，frame 为 `imu_link`，水平面 yaw-only 姿态，roll/pitch 固定为零。真实姿态只在模拟器内部用于生成传感器读数。

```text
gyro_m = [0, 0, omega] + b_g + n_g
acc_m  = R_world_body^T (a_world - g_world) + b_a + n_a
g_world = [0, 0, -9.81] m/s²
```

每个采样点、每个轴独立生成零均值高斯噪声；基础配置给每轴每采样标准差，分别为 rad/s 与 m/s²。消息协方差对角线填对应方差，不能把 sigma 填成 sigma² 的替代。它不是噪声密度；如果进阶使用连续时间谱密度，须先约定频谱定义和采样转换。

不输出融合姿态，`orientation_covariance[0]=-1`。水平静止时 z 比力约为 +g；估计器只积分 x/y，并在已知水平假设下解释重力。基础 `bias=0`，常值偏置和随机游走在理解白噪声后加入，偏置不可从真值直接传给估计器。

静止、恒加速度、恒角速度实验分别验证噪声、坐标方向、积分与时间戳。理想到点/瞬移不参与 IMU 实验；reset 清空状态历史、估计缓存与传感器队列。

## 6. ROS 坐标、时间和发布责任

标准链：`map → odom → base_link → {laser, imu_link}`。

| 模式 | `map → odom` | `odom → base_link` | 真值用途 |
| --- | --- | --- | --- |
| truth | 唯一定位适配节点发布单位变换，起点/世界约定固定 | 同一适配节点由选中的真值生成 | 建图、控制、评估 |
| slam | SLAM 后端发布全局修正，前端阶段为单位变换 | 唯一定位适配节点由连续估计生成 | 只用于评估 |

两个模式启动互斥。估计器与真值发布器先只发布原始里程计消息，由一个简单选源/适配节点给下游统一 `/odometry` 并负责连续 TF，不引入通用路由框架。

`/ground_truth/odometry` 使用 `world` / `ground_truth_base` 的 frame 标识，默认不广播真值 TF。显示对照轨迹时，评估工具明确做一次坐标对齐并发布 Marker，不让真值抢占 `base_link` 或给 SLAM 提供隐藏的绝对定位。

`Odometry.pose` 在 `header.frame_id`；`twist` 在 `child_frame_id`，由内部世界速度显式旋转到机体系。所有传感器使用采样时间，禁止用回调到达时间替代。

唯一 `/clock` 来自模拟器，所有课程节点 `use_sim_time=true`。默认仿真 200 Hz，IMU 200 Hz，激光 10 Hz，控制 50 Hz；若需要非整数频率，用采样时刻调度与状态插值，不能靠多个 wall timer 累积相位误差。暂停/单步以仿真 tick 为单位；reset 开始新试验时协调清空所有时序状态。

传感器使用与订阅端匹配的 SensorDataQoS；控制/轨迹采用小队列和明确可靠性；静态/低频地图使用适合晚加入 RViz2 的 transient-local 配置。具体值在第 02 章验证，不复制未经检验的 QoS 模板。

## 7. 话题草案

| 话题 | 消息 | frame / 语义 |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/Clock` | 唯一仿真时间 |
| `/scan` | `sensor_msgs/LaserScan` | laser；均匀角度测距 |
| `/imu/data_raw` | `sensor_msgs/Imu` | imu_link；无姿态输出 |
| `/ground_truth/odometry` | `nav_msgs/Odometry` | world / ground_truth_base；独立真值 |
| `/odometry/estimated` | `nav_msgs/Odometry` | odom / base_link；连续局部估计 |
| `/odometry` | `nav_msgs/Odometry` | 已选源的连续里程计；下游唯一入口 |
| `/cloud/scan` | `sensor_msgs/PointCloud2` | laser；当前有效扫描点 |
| `/cloud/registered` | `sensor_msgs/PointCloud2` | odom；局部配准点云 |
| `/map/cloud` | `sensor_msgs/PointCloud2` | map；按关键帧全局位姿累积/重建 |
| `/map` | `nav_msgs/OccupancyGrid` | map；未知/自由/占据 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | map；RViz2 点击的目标 |
| `/plan/path` | `nav_msgs/Path` | map；几何参考路径 |
| `/plan/trajectory` | 必要时自定义 `Trajectory2D` | odom；执行起点时间、分段时长、系数 |
| `/command/pose` | `geometry_msgs/PoseStamped` | odom；仅理想到点模式 |
| `/command/velocity` | `geometry_msgs/TwistStamped` | 声明的 odom/base_link；仅运动学模式 |
| `/command/wrench` | `geometry_msgs/WrenchStamped` | odom；力和偏航力矩 |
| `/visualization/*` | `visualization_msgs/MarkerArray` | 对应帧；机器人、走廊、ESDF、轨迹 |

`Trajectory2D` 计划固定二维五次表示：Header 与执行起始时刻、每段正 duration、x/y 各 6 个系数；系数按升幂，时间变量为每段起点开始的局部秒。yaw 参考单独明确，不从平移轨迹默默推断。具体 `.msg` 在第 14 章按实际需求确定。

栅格原点/分辨率和 ESDF 符号属于数据契约。ESDF 初版作为普通值对象传给规划器并用 Marker 可视化；确实跨节点共享时再引入包含 Header、origin、resolution、width/height、有效性掩码和距离数组的消息。不使用 OccupancyGrid 的 0-100 值域偷装米制距离。

运行状态以少量明确状态表示：就绪、运行、到达、不可达、估计失效、求解失败、碰撞；不得把空轨迹解释为成功。reset/暂停/单步接口先用标准 service，可表达不了的输入再自定义。

## 8. SLAM、规划、轨迹与控制的连接

SLAM 输出连续 odom、全局地图和 `map → odom`；关键帧位置图只在静态小场景内实现。局部配准失败要影响状态与地图更新；不静默插入未经验证的扫描。

地图读取使用一个版本的快照。默认 unknown 为阻塞，几何路径位于 map；优化在固定地图/走廊上完成，再在发布时转换到连续 odom。控制器执行 odom 中的参考，回环只触发重规划与平滑衔接，不能把一条执行中的轨迹随 TF 突然跳变。

全局目标未被观测时，基础演示先使用已观测自由域内目标；第 19 章可加入简单可达前沿局部目标，逐步扩展观测。没有可达前沿或目标不可达时明确停止，不承诺任意环境下的完备探索。

ESDF 与走廊采用一致的配置空间语义。以原始障碍 ESDF 计算净空时扣除圆盘半径/安全裕量；用已膨胀图生成走廊时不再重复扣一次半径。栅格单元有面积，误差余量需计入分辨率与插值误差。

轨迹优化区分软代价与硬可行性。Bezier 控制点都在已验证凸走廊内部是整段包含性的充分条件；在任意非凸 ESDF 上仅查询控制点或有限采样没有同等保证。对速度/加速度约束也说明采样、凸包界或解析极值各自的含义。

控制器通过同一 trajectory 采样 API 获取 p/v/a。PD 与 MPC 共享质量、输入限幅和时间定义。规划失败、轨迹过期或定位失效时进入有界制动，并报告剩余制动空间；有限观测或过小净空下仍可能无法避免碰撞，实验如实记录。

## 9. 初始参数建议（非已验证性能指标）

| 参数 | 建议初值 | 单位/说明 |
| --- | --- | --- |
| `world.seed` | 42 | 世界随机种子 |
| `world.size` | `[20,20]` | m；包含边界 |
| `world.circle_count` / `polygon_count` | 8 / 8 | 先少障碍，再增加 |
| `robot.radius` | 0.20 | m；独立于 RViz 标记尺度 |
| `robot.mass` | 1.0 | kg；教学默认值 |
| `robot.drag` | 0.0 | kg/s；先验证无阻尼 |
| `robot.inertia_z` | 0.02 | kg·m²；对应默认均匀圆盘 |
| `robot.force_max` | 2.0 | N；初版每平面轴对称上限 |
| `robot.torque_max` | 0.2 | N·m |
| `lidar.beams` / `rate` | 720 / 10 | 束 / Hz |
| `lidar.fov` | `2*pi` | rad |
| `lidar.range_min` / `range_max` | 0.05 / 10.0 | m |
| `lidar.range_stddev` | 0.01 | m；可设为零 |
| `lidar.scan_model` | snapshot | rolling 为进阶 |
| `lidar.backend` | cpu | cuda 独立选学 |
| `imu.rate` | 200 | Hz |
| `imu.accel_stddev` | 0.05 | m/s²；每轴每采样 |
| `imu.gyro_stddev` | 0.005 | rad/s；每轴每采样 |
| `imu.bias` | 0 | 基础不注入偏置 |
| `sim.dt` | 0.005 | s |
| `mapping.resolution` | 0.05 | m/格 |
| `planning.safety_margin` | 0.05 | m；与半径分开 |
| `control.rate` | 50 | Hz |

传感器各自持有可配置种子。接口支持真值/估计里程计、理想/惯性模型和 CPU/CUDA 后端独立选择，但提供经过验证的 A/B/C 完整预设；不支持的组合在启动时给出一条直接、可理解的错误。
