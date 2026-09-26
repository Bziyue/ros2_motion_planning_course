# 接口与建模约定

状态：第01–20章与所选附录实验已实施。这里记录实际接口及明确标注的扩展语义，范围见 [PROGRESS.md](PROGRESS.md)。优先标准消息；实际需要第二种实现或跨节点传输时才增加抽象。

第 04 章落地约定：simulator 独占时钟，发布 /sim/pose、/sim/velocity、/sim/acceleration 真值调试消息。此章 map 与 odom 重合，真值 TF 暂由 simulator 发布；第 07 章已拆出 truth_odometry 适配器。理想到点在下一 tick 执行，失败扫掠冻结上一个有效状态和时间，/sim/status=collision_predicted，必须 reset 开新试验。不把这个调试入口供给后续 SLAM。

已实现三个互斥模型 ideal/velocity/inertial，只订阅对应命令。速度支持 odom/base_link，力仅支持 odom。有限二维输入、命令 frame/时间、物理参数均在边界校验。速度限模长；力每轴独立限幅。保持输入在模拟时间中超时，速度模式切零速度，惯性模式撤力并继续积分。惯性核心用线性阻尼系统的零阶保持解析离散，碰撞增加曲线与弦的偏差上界；质量与阻尼配置可直接供后续 MPC 使用。

另外已实现 reference 模式，自动采样解析圆轨迹，不订阅外部命令。sampleCircle(initial,radius,omega,time) 返回同一 State2D 中一致的 p/v/a 与 yaw；初始速度明确非零。参考轨迹半径与机器人半径分开，力可行性不在理想模式保证范围内。第14章trajectory模式提供同一p/v/a采样语义，使用Trajectory2D消息；具体接入见文末。

## 1. 模块与数据边界

```text
世界几何 → 机器人仿真 → 激光/IMU → 定位 → 栅格/ESDF → 路径/轨迹 → 控制 → 仿真
                 └→ 真值里程计 → 真值模式选源或独立评估
```

世界生成、CPU 雷达、IMU、机器人积分可以先在一个 simulator 节点里按仿真时刻调用，减少初学者要理解的异步行为。估计、地图、规划和控制按课程进度逐步加入。算法是独立 C++ 函数/小类，节点只处理参数、消息和调度。

几何真值由仿真与独立评估持有。SLAM 和默认规划器不接收世界障碍物列表。显式 `oracle_map` 演示使用完整地图时，在 launch 名称、RViz2 标题和实验记录中注明。

已实现的 C++ 值类型包括 `Pose2D`、`State2D`、`World2D`、`PlanningGrid`、`Esdf2D`、`ConvexRegion`、`TimedTrajectory`，以include中的实际定义为准。不建立全系统的抽象基类树；雷达 CPU/CUDA 只共享一个清楚的输入/输出约定。

## 2. 状态、外形和控制输入

状态为 `p=(x,y)`、`v=(vx,vy)`、`yaw`、`omega`；`p/v` 在世界或所声明的惯性坐标系中，`yaw` 逆时针为正。SI 单位为 m、s、rad、kg、N、N·m。

- **圆盘**：中心是状态点，`radius` 是碰撞半径。雷达与 IMU 默认在圆心，与 `base_link` 对齐。
- **理想到点**：目标位姿只用于几何规划演示，检查两位置之间的扫掠圆盘；不是任意穿障碍的瞬移。IMU 停用，不能与融合 SLAM 组合。
- **理想连续执行**：输入含时间的参考 `p/v/a`，从轨迹解析求导得到 IMU 的运动真值；位置、速度与加速度来自同一个参考，不对跳变位置做差分。
- **速度模式**：输入 `TwistStamped`，帧明确为 `odom` 或 `base_link` 并转换。瞬时速度跳变不适合 IMU 教学；传感器融合实验使用光滑参考或惯性模式。
- **惯性模式**：输入 `WrenchStamped`，平面力以 `odom` 标记，当前实现要求odom轴与物理世界轴平行；`torque.z` 为偏航力矩。A/B显式真值对齐，C强制初始yaw=0；非零初始朝向的力坐标适配未实现，不能只改frame名称。

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
- **进阶 rolling 设计（未实现）**：逐束采样，`time_increment=scan_time/N`，header 为首束采集时刻，整帧采完再发布。每束使用自己的 pose，去畸变方法与扫描模型同时启用。
- CPU/CUDA 输入一致；确定性对比先关噪声。加噪对比使用同一组事先生成的样本，不能把不同随机序列误认成几何误差。

第 05 章已落地 CPU snapshot：/scan 使用 SensorDataQoS，所有束共享同一采样位姿与时间。第 06 章已将整数 tick 基线扩展为按采样序号调度，频率范围 [0.1, 1/dt] Hz，支持 7 Hz 等非对齐频率。暂停/碰撞冻结不重采样，reset 在 t=0 发布新试验的首帧。光束 Marker 以 odom 表达采样时刻的实测射线，不随当前机器人移动；这些显示辅助量不供 SLAM 使用。CPU 单帧束数范围为 2-100000，参数改动后重启。

近距 NaN 是本课程的已批准无效读数约定。REP-117 对“过近”另有 -inf 定义；不能把课程的 NaN 约定说成完整实现该 REP。实际设备适配时需区分处理。

测距噪声已实现：lidar.range_stddev 为每束每采样标准差，默认配置 0.01 m，0 关闭；lidar.noise_seed 默认 4242，与世界 seed 独立。只对有限真回波加噪，越出量程变 NaN。reset 同步重新设定雷达随机流；重复性限定在相同参数、采样顺序和标准库实现。scanCpu 保持纯几何输出，addRangeNoise 单独调用，perturbRange 支持用给定误差做后端成对比较。

## 5. IMU 契约

第 06 章已落地：imu.hpp/imu.cpp 为纯 C++ 核心，imu_messages.cpp 转成 /imu/data_raw（SensorDataQoS）。第06章允许 reference/inertial 启用 IMU，第14章增加连续trajectory支持；旧章节默认关闭。六轴每采样白噪声、独立种子与 reset 重放已实现。无姿态估计，orientation=单位四元数只是占位，orientation_covariance[0]=-1；协方差对角填 sigma²。全部 sigma=0 的无噪声实验会产生全零测量协方差，按 ROS 约定表示未知，后续估计器必须显式配置处理。显示 Marker 不供 SLAM 使用。

`sensor_msgs/Imu`，frame 为 `imu_link`，水平面 yaw-only 姿态，roll/pitch 固定为零。真实姿态只在模拟器内部用于生成传感器读数。

```text
gyro_m = [0, 0, omega] + b_g + n_g
acc_m  = R_world_body^T (a_world - g_world) + b_a + n_a
g_world = [0, 0, -9.81] m/s²
```

每个采样点、每个轴独立生成零均值高斯噪声；基础配置给每轴每采样标准差，分别为 rad/s 与 m/s²。消息协方差对角线填对应方差，不能把 sigma 填成 sigma² 的替代。它不是噪声密度；如果进阶使用连续时间谱密度，须先约定频谱定义和采样转换。

不输出融合姿态，`orientation_covariance[0]=-1`。水平静止时 z 比力约为 +g；估计器只积分 x/y，并在已知水平假设下解释重力。基础 `bias=0`，常值偏置和随机游走在理解白噪声后加入，偏置不可从真值直接传给估计器。

静止、恒加速度、恒角速度实验分别验证噪声、坐标方向、积分与时间戳。理想到点/瞬移不参与 IMU 实验。reset 已清空模拟器状态历史、采样序号与噪声序列；外部订阅者需自行清理本地历史，服务不能远程清空已入队 DDS 消息。后续估计节点必须处理时间回退或跨试验重启。

## 6. ROS 坐标、时间和发布责任

标准链：`map → odom → base_link → {laser, imu_link}`。

| 模式 | `map → odom` | `odom → base_link` | 真值用途 |
| --- | --- | --- | --- |
| truth | 唯一定位适配节点发布单位变换，起点/世界约定固定 | 同一适配节点由选中的真值生成 | 建图、控制、评估 |
| slam | SLAM 后端发布全局修正，前端阶段为单位变换 | 唯一定位适配节点由连续估计生成 | 只用于评估 |

两个模式启动互斥。估计器与真值发布器先只发布原始里程计消息，由一个简单选源/适配节点给下游统一 `/odometry` 并负责连续 TF，不引入通用路由框架。

`/ground_truth/odometry` 使用 `world` / `ground_truth_base` 的 frame 标识，默认不广播真值 TF。显示对照轨迹时，评估工具明确做一次坐标对齐并发布 Marker，不让真值抢占 `base_link` 或给 SLAM 提供隐藏的绝对定位。

`Odometry.pose` 在 `header.frame_id`；`twist` 在 `child_frame_id`，由内部世界速度显式旋转到机体系。所有传感器使用采样时间，禁止用回调到达时间替代。

在线运行的唯一 `/clock` 来自模拟器，传感器bag回放时由播放器接管；所有相关课程节点 `use_sim_time=true`。默认仿真200Hz、IMU200Hz、激光10Hz、PD/MPC控制50Hz。SensorScheduler 使用 t_k=round(k*1e9/rate) 纳秒，不重复累加已舍入周期。模拟器接受一个运动区间后，按时间合并派发两类到期样本，最多落后采样时刻一个模拟 tick；这个界限不包含墙钟执行/DDS 时延。低负载以外不承诺墙钟实时性。

reference 按绝对时刻求解析轨迹，inertial/velocity 在最近接受区间内用当时保持的输入进行解析分步。ideal 仅在 tick 末端跳变，不插值瞬移且禁止 IMU。恰好落在 tick 末端的 IMU 采用刚结束区间的加速度（左极限）；新命令不改写已发布样本。t=0 是新试验的初始状态，发力命令在之后的区间生效。

模拟器在每个采样时刻发布原子的 /ground_truth/odometry，并补发 tick 末端状态；同一时刻两传感器共享一次真值发布。第 07 章由 truth_odometry 适配器负责 /odometry、map→odom 和 odom→base_link；launch 强制 publish_truth_tf=false。第 04–06 章默认仍由模拟器直接广播这两条 TF。暂停时只重复真值/时钟/TF 心跳，不重新抽取噪声；单步只释放本步到期样本；碰撞拒绝整个区间，时间与采样序号都冻结。这些 TF 只属于显式 truth 教学模式；第08章起的估计入口使用estimated_odometry替换真值适配器。原始真值使用可靠、transient-local、深度 100 的队列，选源 /odometry 使用可靠、volatile、深度 100；重复的暂停心跳不算新观测。

传感器使用与订阅端匹配的 SensorDataQoS；控制/轨迹采用小队列和明确可靠性；静态/低频地图使用适合晚加入 RViz2 的 transient-local 配置。具体值在第 02 章验证，不复制未经检验的 QoS 模板。

第 07 章 mapping 已接入 /scan 和 /odometry 的整数纳秒精确配对。输出 /cloud/scan（laser）与 /cloud/registered（odom），均为当前帧有限回波，snapshot、圆心单位外参、XYZ float32、z=0。不订阅 TF 或真值调试话题；当前 registered 仅指位姿变换，不含扫描匹配。第10章SLAM另提供/map/cloud关键帧重建。

## 7. 话题概览

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
| `/plan/trajectory` | `motion2d_interfaces/Trajectory2D` | odom；执行起点时间、分段时长、系数 |
| `/command/pose` | `geometry_msgs/PoseStamped` | odom；仅理想到点模式 |
| `/command/velocity` | `geometry_msgs/TwistStamped` | 声明的 odom/base_link；仅运动学模式 |
| `/command/wrench` | `geometry_msgs/WrenchStamped` | odom；力和偏航力矩 |
| `/visualization/*` | `visualization_msgs/MarkerArray` | 对应帧；机器人、走廊、ESDF、轨迹 |

`Trajectory2D` 采用二维五次表示：Header 与执行起始时刻、每段正 duration、x/y 各 6 个系数；系数按升幂，时间变量为每段起点开始的局部秒。yaw 参考单独明确，不从平移轨迹默默推断。第14章已定义 motion2d_interfaces/Trajectory2D 与 QuinticPiece2D；具体约定见文末。

栅格原点/分辨率和 ESDF 符号属于数据契约。ESDF 初版作为普通值对象传给规划器并用 Marker 可视化；确实跨节点共享时再引入包含 Header、origin、resolution、width/height、有效性掩码和距离数组的消息。不使用 OccupancyGrid 的 0-100 值域偷装米制距离。

运行状态以少量明确状态表示：就绪、运行、到达、不可达、估计失效、求解失败、碰撞；不得把空轨迹解释为成功。reset/暂停/单步接口先用标准 service，可表达不了的输入再自定义。

第 07 章已发布观测 /map：mapping 独立配置固定范围，map=odom 的显式 truth 基线。DDA 清理射线，有限端点占据，+inf 仅清理量程内，NaN 无更新；每帧每格一次、命中优先。未知 -1，已观测为 0–100 概率取整；默认 log-odds 增量 logit(.7)/logit(.4)、截断 ±4。/map 使用可靠 transient-local 深度 1，header 保留最后插入扫描时间。时间回退清图，先等待暂停状态下的首帧再恢复。

独立评估 rasterizeTruth 仅用于 mapping_demo：完整几何与格子接触即占据，使用圆/方格距离和凸多边形 SAT，不调用 raycast 或地图更新。evaluateObserved 记录 observed/uncertain，仅对明确分类的观测格统计误判；未知不作为自由，局部指标不代表全图完成度。

## 8. SLAM、规划、轨迹与控制的连接

SLAM 输出连续 odom、全局地图和 `map → odom`；关键帧位置图只在静态小场景内实现。局部配准失败要影响状态与地图更新；不静默插入未经验证的扫描。

地图读取使用一个版本的快照。默认 unknown 为阻塞，几何路径位于 map；优化在固定地图/走廊上完成，再在发布时转换到连续 odom。控制器执行 odom 中的参考，回环只触发重规划与平滑衔接，不能把一条执行中的轨迹随 TF 突然跳变。

全局目标未被观测时，基础演示先使用已观测自由域内目标；第 19 章已加入简单可达前沿局部目标，逐步扩展观测。没有可达前沿或目标不可达时明确停止，不承诺任意环境下的完备探索。

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
| `imu.accel_stddev` | [0.04,0.05,0.06] | m/s²；x/y/z 每采样 |
| `imu.gyro_stddev` | [0.002,0.003,0.004] | rad/s；x/y/z 每采样 |
| `imu.noise_seed` / `imu.gravity` | 6060 / 9.81 | 独立随机种子 / 重力大小 m/s² |
| IMU 偏置（无参数） | 0 | 基础不注入；随机游走为可选练习 |
| `sim.dt` | 0.005 | s |
| `mapping.resolution` | 0.10 | m/格 |
| `planning.safety_margin` | 0.05 | m；与半径分开 |
| `control.rate` | 50 | Hz |

传感器各自持有可配置种子。接口支持真值/估计里程计、理想/惯性模型和 CPU/CUDA 后端独立选择，但提供经过验证的 A/B/C 完整预设；不支持的组合在启动时给出一条直接、可理解的错误。

## 第 08 章落地：估计入口与失败帧

ch08 的 lidar_odometry 只读 /scan，在采样时刻发布 /odometry/estimated（odom/base_link）；
estimated_odometry 适配器发布 /odometry、运动 TF 与 /path/estimated。该入口与 truth 适配器互斥。
首帧定义 odom 的原点与朝向，map 暂同 odom。/cloud/local_map 是有限关键帧表面，/map 仍由观测建图生成。
/estimation/status 给出初始化、收敛、退化、迭代耗尽或 tracking_lost 等状态。
失败帧没有位姿输出；mapping 可越过缺失位姿的旧扫描处理更新的精确配对。
协方差为未经标定的固定测量尺度；twist 为成功帧之间的割线速度，不把估计信息矩阵当成真实协方差。

## 第 09 章落地：融合时间边界

- `/odometry/lidar`：激光里程计观测，保留扫描时间；融合只订阅此流和 `/imu/data_raw`。
- `/odometry/estimated`：最新 IMU 采样时间的预测；迟到校正在下一次高频输出生效，不重复 TF 时间。
- `/odometry/scan`：激光时刻被接受的 EKF 校正位姿；本章 mapping 将位姿输入重映射到此流。
- 历史窗口默认 2 s，按左侧 IMU 样本保持、在激光时刻切分与重放；超出窗口拒绝，不使用墙钟。
- `/fusion/status` 区分等待、激光创新拒绝、过旧、激光陈旧及 IMU 中断；激光超过 .5 s 无校正仍传播但标记陈旧，
  IMU 相邻间隔超过 .1 s 清空初始化并等待激光。控制器必须检查这些状态。
- reset 先暂停并排空旧消息，分别接收两传感器的新原点；不保证跨网络积压的多轮试验消息自动分代。

## 第 10 章落地：回环后端与全局地图

- 后端 `/scan` 与 `/odometry/scan` 必须同一整数纳秒；只保存有效关键帧。
- `/map/cloud`、`/map`、`/path/graph_before`、`/path/graph_after` 与 `/slam/edges` 都以map为坐标，
  各次关键帧更新使用同一时间戳；before是固定首帧基准下的原始里程计基线。
- `/slam/status` 包括结果名称、frames和loops计数；capacity_reached 表示达到200帧教学容量。
- `slam_node` 是map→odom的唯一动态发布者，estimated适配器设置publish_map_alignment=false。
  回环不改写odom轨迹；原始扫描以各自优化位姿重建，不追加到旧地图。
- 双向几何回环仅在附近旧关键帧上搜索，重复几何仍可能误接；不是失锁后的全局重定位。

## 第11–12章已落地的规划与距离接口

`planner_node`消费`/map`、选源`/odometry`、map系`/goal_pose`及TF；生成`/planning/grid`配置空间0/100掩码、
`/plan/path`几何折线与`/plan/status`。失败空路径，时间回退清除目标；不直接控制机器人。

`esdf_node`只消费未膨胀`/map`。`/esdf/cloud`为结构化PointCloud2，行列等于地图；
FLOAT32字段x/y/z/distance，偏移0/4/8/12，每点16B，z=0、其余单位m。距离是带符号中心距离，
不是占据概率或方格边界精确SDF；全阻塞为-inf、is_dense=false。
`/esdf/gradients`为map系MarkerArray显示箭头；`/esdf/status`区分ready/no_finite_field/invalid_map。
输入输出使用地图采样时间，输出保留最新快照；错误输入清空旧云与标记。
默认未知阻塞，自由阈值35；半径只在规划膨胀或未膨胀ESDF净空约束的一处计算。


## 第13章已落地的走廊接口

planner内同一次搜索生成走廊，直接复用PlanningGrid和原始A*路径，不异步拼接地图与路径。
/plan/path_raw保留邻格点，/plan/corridor_path存每区对应线段端点；/plan/corridors只用于显示。
这些路径和Marker使用起点里程计时间、map frame；无执行时长，单位姿态不是yaw规划。
/plan/corridor_status单列走廊构造状态；关闭功能为disabled。每批DELETEALL后重新画闭合LINE_STRIP。
错误/reset清空全部规划显示；算法消费ConvexRegion，不解析Marker。半径只在配置栅格处理一次。

## 第14章轨迹内核

QuinticPiece包含正duration和2×6秒制升幂系数；列k单位m/s^k。
PolynomialTrajectory验证非空/有限/C²连接，evaluate/sample只接收[0,total]，内部连接点取右段。
TranslationState给p/v/a，yaw独立。stopAtWaypoints每路点零v/a，nominal_speed是平均速度参数。
CSV包含段时长与全部系数；ROS时刻与执行策略见下一节，不以Path伪装有时间的轨迹。


## 第14章ROS执行契约

/plan/trajectory使用motion2d_interfaces/Trajectory2D。header是发布时间和固定odom执行帧，start_time为绝对仿真起点（零就是原点）。
pieces非空，duration正有限，x/y各6个升幂秒制系数，C²连接；yaw为独立常值朝向。
trajectory_node显式/trajectory/execute服务将同一路线一次转换到odom并发布；下游不随TF跳变改写它。
/plan/trajectory_path是预览，逐点stamp为执行时间。控制/仿真只使用系数。
model=trajectory只订阅该命令；本章要求当前及起终点静止、位置/yaw匹配、起点非过去，运行中拒绝替换。
/sim/trajectory_status单列idle/accepted/waiting/executing/completed/rejected与碰撞/非法状态。
未来/终点保持、C²采样、非整步传感器均共用解析函数。扫掠用全区间加速度上界扩展圆盘；碰撞前冻结所有时间。
错误新消息不会改写已接受曲线；reset清空。在线p/v/a接续与失败制动在第19章扩展。


## 第15章MINCO内核

Minco2D固定首尾p/v/a，内部路点N-1个、正时长N个；coefficients为6N×2升幂行矩阵。
trajectory()返回统一PolynomialTrajectory。energyPartials()的times是固定系数偏导；
propagate()返回消元后的总路点/时长梯度，不能混用。核心仍无障碍/动力学约束，异常尺度报数值失败。

### 第16章连续认证候选

`/plan/certified_candidate`使用Trajectory2D，可靠保留1，odom帧，header.stamp为规划所用odom时刻；
start_time=0是候选局部原点，不是执行命令。空pieces撤销；仅收敛且独立Bézier证书通过发布非空候选。
`trajectory.source=certified`在`/trajectory/execute`时验证静止、匹配边界、时间差[0,.5]s，再给出实际绝对开始时刻并发`/plan/trajectory`。
该证书只针对冻结的观测配置空间栅格和参考曲线，不覆盖后续地图变化/定位或跟踪误差。

## 第17章控制接口

tracker_node只读取/odometry（odom/base_link），先将body twist旋转到odom；不读取truth、TF或world。
/control/reference为odom/reference_base诊断Odometry（不发布TF）；/control/reference_acceleration为odom系AccelStamped。
/control/wrench_requested是限幅前输入；/command/wrench为实际限幅输入，header.frame_id=odom，正常stamp为观测时刻。
/control/reference_path仅预览；/control/status可靠保留1。/tracker/enable SetBool关闭后发新阻尼制动，reset保留启用选择。
50Hz按唯一仿真观测时间戳，无补发突发；回退清空曲线和计时起点，暂停不重复控制。
.12s仿真时间无新观测或积压旧观测触发最后可用速度的限幅阻尼制动，stamp为当前ROS时间；无停车/避障保证。
reference.source=trajectory时消费/plan/trajectory，要求当前与首尾静止、起点位置/yaw匹配、未来开始；执行中拒绝替换。

## 第18章MPC内核

LinearModel为精确ZOH的4×4 A、4×2 B；状态[p_x,p_y,v_x,v_y]，输入[Fx,Fy]，统一odom。
LinearMpc::problem暴露实际BoxQp(P,q,A,lower,upper)，强凸代价，N个未来参考/N个可选节点凸区，上一拍实际力。
LinearMpc::step只有solved才执行并热启动；其他状态丢弃序列、返回新的限幅阻尼力，不复用旧输入。
走廊/速度仅预测节点约束；力/变化率均按轴，变化率乘控制间隔后单位N。未实现终端不变集或连续避障保证。

### 第18章ROS诊断

control.controller=pd/mpc，两者使用同一tracker与时间/坐标边界；mpc.dt由名义control.rate_hz得出，参数仅启动读取。
/control/mpc_status使用MpcStatus，可靠volatile100，header为此次观测stamp/odom。compute_seconds包括QP组装/求解/热启动，不含整个ROS链路。
max_violation只在solved有效，失败NaN；未迭代残差inf。/control/prediction为保留1的odom Path，点stamp=t+(k+1)h；失败清空。
正常闭环以上一次发布的限幅力作为已接收执行器输入；不是额外的力反馈传感器。失效状态产生新的限幅制动并清热启动，yaw同样制动。

## 第19章参考接续内核

NavigationReference绑定TimedTrajectory、可选每段ConvexRegion和map快照ns，整体在一个固定坐标系；transformReference对曲线与区域一起冻结变换，yaw也旋转。
ReferenceSchedule只保留活动和一个待交接参考；sample/region仅查询当前或未来，不因预读改变活动状态，advance以实际观测时间激活。
accept要求未来开始、首p/v/a/yaw与旧参考该时刻匹配、终点静止；拒绝保留已有有效参考。reset(anchor)显式清空，紧急reset不承诺C2。

### 第19章观测局部规划

observedLocalRoute读取同快照原始占据与PlanningGrid；全局目标可达时A*，否则从可达配置格挑邻近原始未知区且有至少.1m目标距离改善的候选，按弧长截局部前缀。
原始goal占据>=50、地图外、起点阻塞等单列状态；未知始终阻塞。局部前沿无进展不证明整个未知世界无路。
replanObserved固定未来首p/v/a、静止终点，同快照ESDF/走廊；优化收敛且certifyBezier通过才接受，否则验证1/1.5/2/3倍时长的停点备用段。结果分别标optimized/fallback、原优化状态、耗时，失败无曲线。

### ch19 原子导航参考

`/navigation/reference` 为 NavigationReference：Trajectory2D、每段一个 ConvexRegion2D(float64 CCW顶点)、snapshot_time。
控制器重新验证消息和曲线/区域证书；自由空间真实性由规划器负责。`/control/accepted_reference` Int64为接受的start_ns；一次仅一个待确认、起点唯一。
`/navigation/stop` Empty显式清空参考与显示并用当前观测制动。`control.controller=ideal` 输出odom PoseStamped（下一周期目标），需ideal仿真/禁用IMU；pd/mpc仍输出力矩。
静止启动/重启允许位置0.03m、yaw0.02rad、速度/角速度0.02以内；普通交接严格匹配p/v/a/yaw，拒绝不覆盖旧曲线。

### ch19 在线导航快照与状态

NavigationNode订阅/map、/odometry、/scan（只检查时间）、/goal_pose、/control/status、/control/accepted_reference及TF；禁止真值输入。
地图快照一次构建raw/config-grid/ESDF，单线程求解期间不修改；最近可用map←odom对齐在本次求解固定。剩余区域在新快照/对齐下重新整体验证，失败制动。
规划5Hz、控制50Hz（A理想点200Hz），未来.15s交接，优化预算.04s，等待ACK时不发第二候选。`NavigationStatus` 含状态、规划/优化状态、墙钟求解时长、接受数与停车数。
默认激光.35s、odom.12s、map3s过期触发停车；只有仿真时钟推进才判过期。时间回退清空目标；新目标也显式停车。静止后自动重试当前目标，不承诺完备探索。
C默认初始yaw=0，估计odom的轴与仿真物理输入轴一致、原点在第一帧；改初始yaw必须同时实现力向量坐标适配，不能仅平移目标。A/B真值适配器明确world=map=odom。

### ch20 CUDA后端与性能诊断

`lidar.backend=cpu|cuda` 启动时选择；CPU-only构建请求cuda抛出清晰错误。CUDA理想扫描返回共同float数组编码，随后仍由CPU共享噪声函数处理；timestamp仍来自SensorScheduler，不由GPU墙钟替代。
`lidar.profile=false`默认；true时发布`/sim/lidar_timing` LidarTiming。header是采集stamp/laser；scan/noise/message/publish/total以墙钟秒计，CUDA另列kernel事件时间/download，CPU两个字段NaN。total至publish返回，不含DDS接收、RViz、调试Marker或诊断自身；不可称为完整传输延迟。静态几何/输出/事件复用，reset不重新上传。

## 附录扩展边界

差速stepDifferential与偏置stepBiasWalk为独立可选核心工具，不新增默认模拟器模式或噪声参数。附录B的轮动力学/NMPC、C的滚动扫描/复杂障碍/重定位为设计练习。
replay_slam只消费传感器bag，播放器为唯一时钟；启动文件仅提供本课程重合的静态外参。Doxygen和报告模板见附录D。
