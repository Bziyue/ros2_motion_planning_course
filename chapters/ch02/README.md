# 第 02 章：二维坐标变换与时间

先修：第 01 章。本章包含独立 SE(2) 数学库、整数仿真时钟、暂停/单步/reset 和完整的 TF 演示。

## 一、用可以手算的点检查坐标方向

机器人圆心在 map 的 `(1,2) m`，朝向逆时针 90 度。机器人前方 2 m 的点在机体系为 `(2,0)`，在 map 中应为 `(1,4)`。

~~~bash
source /opt/ros/lyrical/setup.bash
cd /home/zdp/ForCodex/ros2_motion_planning_course/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 run motion2d se2_demo
colcon test --packages-select motion2d --event-handlers console_direct+
colcon test-result --verbose
~~~

预期输出 body `(2,0)`、map `(1,4)`、recovered `(2,0)`。这个程序只调用数学库，不创建 ROS 节点，也不需要 RViz2。

## 二、函数和公式

源码：`motion2d/geometry/se2.hpp` 与 `src/geometry/se2.cpp`。

- `Pose2D` 表示 `T_A_B`：B 的原点和朝向用 A 的坐标表达，单位 m/rad。
- `transformPoint`：`p_A = R(yaw) p_B + t`。
- `inverse`：位置为 `-Rᵀt`，角度为 `-yaw`，不能简单把位置取负。
- `compose`：从右向左应用变换，`T_A_C = T_A_B T_B_C`。
- `wrapAngle` 统一到 `[-pi,pi)`；不要直接相减跨越 ±pi 的角度。

测试包括手算 90 度旋转、带外参的坐标链、200 个固定种子的往返变换、角度边界。练习见 [ch02](../../exercises/ch02/README.md)。

## 三、统一仿真时钟与 TF

停止其他课程 launch，重新构建并加载环境后运行：

~~~bash
ros2 launch motion2d_bringup ch02.launch.py
~~~

圆心固定在 (1,2)，朝向为 `pi/2 + 0.2t`，蓝色箭头为 laser 帧的 (2,0) 向量。它是坐标演示，还没有动力学和射线求交。TF 链为 `map → odom → base_link → {laser, imu_link}`，只有 odom 到 base_link 随时间变化。map/odom 在本章对齐。

另一个终端加载相同环境：

~~~bash
ros2 service call /sim/pause std_srvs/srv/SetBool "{data: true}"
ros2 service call /sim/step std_srvs/srv/Trigger "{}"
ros2 service call /sim/reset std_srvs/srv/Trigger "{}"
ros2 service call /sim/pause std_srvs/srv/SetBool "{data: false}"
~~~

dt 默认 0.005 s；step 只在暂停时有效；reset 把时钟和演示状态归零并保持暂停。暂停后 /clock 仍重复发布相同时间，让晚加入者取得当前时间。所有消费者使用 use_sim_time；不能同时启动多个课程时钟。

工作空间内运行 `/usr/bin/python3 ../scripts/check_ch02.py` 验证默认配置的时钟、TF 和服务，脚本最终恢复运行。reset 导致 TF 时间缓存清空是新实验的正常边界。

节点每 1 s 墙钟时间重发静态外参，暂停时也重发当前动态边。这样 TF 缓存清空后仍能恢复显示，状态没有额外前进；检查脚本明确验证 reset 后 map 到 laser 可查询。

## 四、时间实验

暂停后单步十次：应增加 0.05 s，朝向增加 0.01 rad。修改 dt 和 yaw_rate 后先手算再测。参数只在启动读取，修改 YAML 需要重启。

实际画面：[ch02_rviz.png](../../textbook/figures/ch02_rviz.png)。已确认 reset 后 TF 缓存恢复，RViz2 的 TF 与 Marker 状态正常。

本机 RViz2 15.2.6 的 TF 显示插件在时间回退后退出时出现过崩溃；默认配置采用两个 Axes 显示地图/机体坐标轴，重置、显示和退出均已通过。完整 TF 数据链仍正常广播和检查，无需 TF 显示插件才能使用 TF。
