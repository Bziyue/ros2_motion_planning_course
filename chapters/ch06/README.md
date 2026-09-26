# 第 06 章：高斯白噪声 IMU 与传感器时间

先修 01-05。本章逐功能实施；当前完成理想比力模型与解析实验。白噪声、ROS 原始 IMU 和非整数频率调度在随后增量中实现。

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
