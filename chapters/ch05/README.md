# 第 05 章：CPU 激光雷达

先修 01-04。本章从一束射线开始，扩展到圆心安装的均匀扫描雷达。已实现射线几何、CPU snapshot 扫描、LaserScan 消息、RViz2 和可复现的测距噪声；束数与性能实验随后加入。

## 运行整帧扫描

在 ros2_ws 重新构建、加载 install 后：

~~~bash
ros2 launch motion2d_bringup ch05.launch.py
# 另一个同 ROS 环境的终端，从 ros2_ws 执行
/usr/bin/python3 ../scripts/check_ch05.py
~~~

默认使用 reference 圆轨迹。可改为 model:=ideal 后点击 RViz2 的 2D Goal Pose，或用第 04 章的 velocity/inertial 控制输入。ch01-04 默认不启用雷达。脚本会修改场景状态并最终停在新试验 t=0。

参数在 motion2d_bringup/config/ch05.yaml 中，修改后重启：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| lidar.beams | 720 | 均匀角度束数，支持 2-100000 |
| lidar.angle_min / lidar.fov | -pi / 2*pi | 雷达系起始角和视场，rad |
| lidar.rate | 10 | Hz；周期须为 dt 的整数倍，且不低于 0.1 Hz |
| lidar.range_min / lidar.range_max | 0.05 / 10 | m，包含两个端点 |
| lidar.range_stddev / lidar.noise_seed | 0.01 / 4242 | 每个有效回波的标准差（m）与独立随机种子 |
| lidar.visualize_beams | true | 发布光束显示辅助量，可关闭 |
| lidar.backend / lidar.scan_model | cpu / snapshot | 本章唯一支持组合；CUDA/rolling 未实现 |

整周角间隔为 2*pi/N，不重复首末方向；部分视场为 FOV/(N-1)，包含两个端点。近距真实命中为 NaN；量程内无回波为 +inf；不跳过近障碍而报告后墙。课程 NaN 约定与 REP-117 对过近的 -inf 编码不同，硬件适配时需区别。

所有束使用同一采样位姿，time_increment=0，scan_time=1/rate。周期不对齐时（如 dt=.005、rate=7）报错，不悄悄改变频率。暂停/碰撞冻结不重采样；reset 在 t=0 开始新序列。晚加入的 /scan 订阅者在暂停时需等恢复运行，才能收到新帧。

RViz2 显示橙色有效射线、淡蓝无回波射线及红色命中点。/scan 为 Best Effort、Volatile；光束 Marker 保留在采样时刻的 odom 坐标，不随当前机器人移动。光束辅助量不供 SLAM 使用。reset 后如 RViz 显示正在恢复，先恢复运行，等待下一帧；已知的主机 RViz 关闭问题见验收记录。

## 先验证一束射线

在 ros2_ws 构建并加载 install 后：

~~~bash
ros2 run motion2d raycast_demo
colcon test --packages-select motion2d
colcon test-result --verbose
~~~

演示的三个距离依次是 2、4、2 m：圆心 (3,0)、半径 1 的圆先于 x=10 边界被击中。雷达不会穿过这个圆去报告后面的墙。

## 代码导读

- `include/motion2d/sim/raycast.hpp`：单位、坐标与输入前提。
- `src/sim/raycast.cpp`：圆二次方程、线段叉积求交、所有表面取最近值。
- `src/examples/raycast_demo.cpp`：可手算的最小实验。
- `src/sim/lidar_cpu.cpp`：角度、量程编码和整数 tick 周期。
- `src/ros/lidar_messages.cpp`：标准消息与光束 Marker 转换。
- `src/nodes/simulator_node.cpp`：直接在同一仿真 tick 调用扫描，没有第二个世界或时钟。
- `test/test_raycast.cpp`：相切、背向、线段端点、平行/共线、边界、遮挡、刚体变换。

射线写为 o+t*d，d 必须为单位向量，t 才是米。返回最近的 t>=0；没有表面命中时为正无穷。圆内起点返回出口交点；实际机器人起点应在自由空间。雷达在圆心且不检测自身，机器人半径只影响碰撞。

几何求交不裁剪量程、不加噪声。这样后续 CPU 扫描和 CUDA 对照可以共享明确的几何基准。平行判定使用适合本课程米制小场景的浮点容差，不宣称任意坐标尺度下的精确几何谓词。

## 练习与验收

完成 [练习 ch05-1](../../exercises/ch05/README.md)。先手推两个圆交点，再运行检查；把圆放到射线背后，不能只检查判别式非负。

ch05-2 补完角度采样。check_ch05.py 针对默认 10 Hz、360 度、开启光束显示的配置，支持 --beams 90/360/720。非默认频率/视场使用对应手算场景验证，不能直接套默认检查断言。

## 高斯测距噪声

scanCpu 始终产生无噪声几何距离，addRangeNoise 再对有限回波加独立高斯误差。sigma 是每束每次采样的标准差，不是噪声密度；设为零不消耗随机数。无回波/盲区不加噪，噪声后越界设 NaN。reset 重新设定雷达种子，同配置、轨迹、采样顺序和标准库下可重放；世界与雷达使用独立随机流。

~~~bash
# ros2_ws
ros2 run motion2d lidar_noise_demo
ros2 run motion2d lidar_noise_demo > ../textbook/data/ch05_noise.csv
~~~

独立实验是 seed=4242、50000 个真实距离为 5 m 的回波、sigma=0.01 m。CSV 由教材直接读取。靠近量程边界时会丢弃越界样本，保留误差可能有偏，不应套用远离边界的统计验收。

真实消息实验使用另一份明确的空房间配置。停止上一场景，再从 ros2_ws 启动：

~~~bash
ros2 launch motion2d_bringup ch05.launch.py model:=ideal rviz:=false \
  config:=src/motion2d_bringup/config/ch05_noise_lab.yaml
# 另一终端，同 ROS 环境
/usr/bin/python3 ../scripts/check_ch05_noise.py
~~~

机器人静止在 20 m 方形房间中心，无内部障碍。360 束、50 Hz、最大量程 20 m；脚本会 reset 并恢复运行，收集 t=0 到 3.98 s 的 200 帧、72000 个残差，与解析墙距比较。缺帧会明确失败，需降低系统负载后重试。它只适用于这份配置。ch05-3 练习对应噪声后的量程处理。
