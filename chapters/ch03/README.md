# 第 03 章：随机圆与凸多边形世界

先修：01-02。目标是在 20×20 m 的边界里生成可复现的圆和凸多边形，用圆盘几何检查碰撞，显示起终点。没有机器人动力学、真实激光或地图估计；世界图形是仿真真值。

## 运行

停止其他课程 launch，从工作空间加载 ROS 环境、重新构建并 source install 后：

~~~bash
ros2 launch motion2d_bringup ch03.launch.py
~~~

默认 seed=42，圆和多边形各 8 个，机器人位于 (-8,-8)，目标位于 (8,8)。完整 YAML 位于 `ros2_ws/src/motion2d_bringup/config/ch03.yaml`。使用 `rviz:=false` 运行无界面版本；使用 `config:=/绝对路径/scene.yaml` 读取自己的配置。

实际截图：[ch03_rviz.png](../../textbook/figures/ch03_rviz.png)。

默认开启起终点净空及保守连通性检查。生成失败时节点退出并关闭这个 launch 的其余进程，日志给出 seed 和原因，不替换种子假装成功。

## 代码入口

| 文件（相对 motion2d/src） | 作用 |
| --- | --- |
| geometry/obstacles.cpp | 点线距离、凸包、严格凸多边形校验、有符号距离 |
| sim/world.cpp | 世界边界、圆盘碰撞与保守 flood fill |
| sim/world_generator.cpp | 固定种子、圆与随机点凸包、起终点保护 |
| nodes/world_scene_node.cpp | 读参数，把真实几何转换为 MarkerArray |

数学与几何只有一份实现；ROS 节点只做配置与消息转换。第 02 章的坐标演示节点提供唯一 /clock 和 TF，本章关闭示意箭头、令 yaw_rate=0。共用 YAML 的 x/y/radius 让显示和碰撞使用相同起点与半径。

## 随机性、外形与可达性

- 圆的半径，或多边形采样圆的半径，落在 size_min/size_max 范围内。polygon_samples 是采样点数量，凸包顶点可能更少。
- 采样半径使用 `R sqrt(u)`，使点在圆盘面积上均匀。直接用 R*u 会更偏向圆心。
- 障碍物允许重叠；所有多边形都是有效的逆时针凸包。形状不越出世界，起终点圆盘与 margin 保持净空。
- 同一配置、种子和 C++ 标准库复现同一世界；跨标准库版本不承诺随机分布实现逐位一致。
- 连通性只做四邻接 flood fill，不是后面的 A*。在半径和 margin 之外，再增加单元半对角线的保守余量，避免把单元中心无碰撞误认为整块无碰撞。
- 判为不连通可能是离散过粗造成的保守结果，可以明确降低 check_resolution 后再试；不会自动反复换 seed。`require_connected: false` 可显示并研究未通过筛选的世界。

自由区域中 clearance 是离障碍/边界的最小距离；占据或越界时用其符号判碰撞，不把它当作整个空间的精确 ESDF。相切计为碰撞。

## 检查和实验

工作空间中运行：

~~~bash
colcon test --packages-select motion2d
colcon test-result --verbose
/usr/bin/python3 ../scripts/check_ch03.py
~~~

通信脚本验证默认的 8+8 个障碍、边界、圆盘尺寸、共享起点和唯一时钟。改了参数后按实验目标检查，不应继续用默认值断言。

将 seed 改为 43 重启，再把两个 count 改为 20。保留每次 seed、参数、是否可达与日志净空；不要只展示成功场景。习题见 [ch03](../../exercises/ch03/README.md)。
