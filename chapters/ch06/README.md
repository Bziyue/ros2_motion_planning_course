# 第 06 章：高斯白噪声 IMU 与传感器时间

先修 01-05。本章逐功能实施；当前完成理想比力、六轴白噪声、ROS 原始 IMU、统计/积分漂移实验和 RViz。非整数频率调度在随后增量中实现。

## 从理想测量开始

IMU 在圆心，imu_link 与 base_link 重合；只有水平平移和 yaw，roll=pitch=0。世界 z 向上，g=(0,0,-9.81) m/s²。

~~~text
omega_body = (0, 0, yaw_rate)
f_body = R_world_body^T * (a_world - g_world)
~~~

静止与匀速均测得 f=(0,0,+9.81)，不是零。加速度计测比力；只在补回世界重力之后才得到世界运动加速度。仿真内部用真值 yaw 转换坐标，返回值不包含 yaw、位置或速度。

重新构建 ros2_ws 并加载 install 后：

~~~bash
ros2 run motion2d imu_demo
colcon test --packages-select motion2d
colcon test-result --verbose
~~~

CSV 的五个场景是静止、匀速、yaw=pi/2 的世界 +x 加速度、圆心原地自转，以及 r=1 m、omega=.4 rad/s 的圆参考。后者输出机体 f_y=.16 m/s² 与 gyro_z=.4 rad/s，f_z 始终为 9.81。

## 代码与边界

- include/motion2d/sim/imu.hpp：三轴角速度、三轴比力与模型前提。
- src/sim/imu.cpp：idealImu，无 ROS 依赖。
- src/examples/imu_demo.cpp、test/test_imu.cpp：手算与自动检查。
- [练习 ch06-1](../../exercises/ch06/README.md)：世界加速度到机体比力。

reference 与 inertial 能提供物理导数。ideal 瞬移和 velocity 直接切换速度不适合 IMU 融合，不能用位置差分伪造巨大加速度。加装非零杆臂、倾斜运动、偏置与随机游走均不属于当前理想模型。

## 白噪声与 ROS 原始消息

重新构建并加载 install，从 ros2_ws 启动：

~~~bash
ros2 launch motion2d_bringup ch06.launch.py
# 另一终端，相同 ROS 环境；脚本会 reset 并最终暂停
/usr/bin/python3 ../scripts/check_ch06.py
~~~

默认 reference 圆轨迹，雷达配置继承第 05 章，IMU 参数见 config/ch06.yaml，修改后重启：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| imu.enabled | true | ch01-05 默认 false |
| imu.rate | 200 | Hz；当前增量周期与 dt 对齐 |
| imu.gravity | 9.81 | 世界重力大小，m/s² |
| imu.gyro_stddev | [0.002, 0.003, 0.004] | x/y/z 每采样标准差，rad/s |
| imu.accel_stddev | [0.04, 0.05, 0.06] | x/y/z 每采样标准差，m/s² |
| imu.noise_seed | 6060 | IMU 独立随机流，reset 重播 |
| imu.visualize | true | RViz 辅助显示，最多 20 Hz |

/imu/data_raw 使用 SensorDataQoS，header.frame_id=imu_link，时间为采样模拟时间。只允许 reference/inertial；imu.enabled=true 与 ideal/velocity 组合会明确拒绝。恒定偏置和随机游走尚未注入，b_g=b_a=0。每轴独立白噪声，不裁剪异常值。设六个 sigma 为 0 可做无噪声实验；零噪声轴不消耗随机数。

orientation 的单位四元数只是占位，orientation_covariance[0]=-1 表示姿态不可用，禁止读取它作为航向。两项测量协方差为 diag(sigma_x²,sigma_y²,sigma_z²)，单位分别为 (rad/s)² 和 (m/s²)²。若三个轴都无噪声，全零协方差按 ROS 标准表示未知；后续滤波应明确配置自己的噪声模型，不能把零当无限置信度。

RViz 紫色箭头是实测水平比力，经仿真采样姿态变到 odom，仅为显示；长度比例为 3 m/(m/s²)。标签显示 f_z 和 omega_z，原地静止也应有 f_z≈9.81。图像、/sim 真值与显示 Marker 不能输入后续 SLAM。暂停不产生新样本，reset 清空采样记录并重置随机引擎；同配置、轨迹、采样顺序及标准库下才保证逐值重放。

## 统计与积分漂移实验

~~~bash
# ros2_ws，纯 C++ 实验，不需要启动场景
ros2 run motion2d imu_noise_demo > ../textbook/data/ch06_noise.csv
ros2 run motion2d imu_noise_demo --drift > ../textbook/data/ch06_drift.csv
~~~

第一项对每轴 60000 个残差做均值与总体标准差统计，seed=6060。单位测试要求均值不超过 5 个标准误、标准差误差小于 2%，并检查轴间与相邻时刻的归一化相关量。ROS 脚本另外收集默认圆参考 t=0..10 s 的 2001 条消息，要求无缺帧，均值在 5 个标准误内、标准差误差小于 8%；还检查暂停、单步、41 帧完整重放、无真值姿态、协方差及 QoS。

--drift 固定 200 Hz、10 s、1000 次独立试验；sigma 倍率 1/2 使用相同种子作配对比较。只用 x 加速度误差和 z 角速度误差，已知固定姿态并去掉重力，展示一维积分误差，不是 SLAM。每步 p+=v*dt+0.5*a*dt²、v+=a*dt、yaw+=gyro*dt。该模型下标准差翻倍使每条误差轨迹和 RMS 翻倍；更长时间会积累更大不确定性，单条轨迹的绝对误差不保证单调。

噪声密度不是每采样 sigma。只有先规定单/双边谱、滤波带宽和采样方式，才可换算。对连续白噪声强度 q、独立区间均值这一特定定义，有 Var(n_k)=q/dt；本章参数固定每采样 sigma，改变频率不会自动重标度。
