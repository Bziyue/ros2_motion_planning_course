# 第 01 章：第一个 ROS 2 / RViz2 圆盘场景

目标：理解 node、topic、launch 与工作空间，显示半径 0.20 m 的圆盘。无需运动学或 SLAM 基础。

## 环境准备

本课程首先验证 Ubuntu 26.04 + ROS 2 Lyrical。当前主机已有 ROS 2、colcon、Eigen、RViz2 和 TeX Live，不需要重装。

干净主机先按 [ROS 官方安装说明](https://github.com/ros2/ros2_documentation/blob/rolling/source/Get-Started/Installation/Ubuntu-Install-Debs.rst) 配置 UTF-8、Universe 和 ros-apt-source 软件源，再安装匹配系统的发行版。仅在 Ubuntu 26.04 且已配置 ROS 软件源时：

~~~bash
sudo apt update
sudo apt install ros-lyrical-desktop ros-dev-tools libeigen3-dev
~~~

安装命令仅供准备新主机，本次没有执行。后续可用 `rosdep install --from-paths src --ignore-src -r -y` 补齐依赖。Lyrical 的 ROS 接口需要 C++20；本课程独立算法保持 C++17，代码避免高级模板技巧。

## 构建与运行（以下使用 bash）

~~~bash
bash
source /opt/ros/lyrical/setup.bash
cd /home/zdp/ForCodex/ros2_motion_planning_course/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch motion2d_bringup ch01.launch.py
~~~

zsh 用户对应使用 `setup.zsh`。第一次构建从只 source 主机 ROS 的新终端开始。显式指定系统 Python 是为了避免桌面工具的辅助 Python 覆盖 ROS 使用的 Python。

RViz2：浅色背景、0.5 m 网格、原点青色圆盘，俯视。Fixed Frame 为 `map`。这个静态入门场景暂用系统时间，第 02 章引入统一仿真时间。

实际截图：[ch01_rviz.png](../../textbook/figures/ch01_rviz.png)。Global Status 此时可能提示尚无 TF 数据；所有图形直接位于 map 帧，无需坐标转换仍可显示。第 02 章建立 TF 后消除该提示。

无图形界面时加 `rviz:=false`。这只验证通信，不等于图形显示验收。第二个终端同样 source 两级环境，进入工作空间后：

~~~bash
ros2 topic list
ros2 topic echo /visualization/robot --once --qos-durability transient_local
/usr/bin/python3 ../scripts/check_ch01.py
~~~

## 代码与公式

入口为 `ros2_ws/src/motion2d/src/nodes/hello_scene_node.cpp`。发布一次后 spin 保持进程存活。Marker 的 x/y 尺寸是直径，所以 `scale.x = scale.y = 2r = 0.40 m`。`orientation.w=1` 是单位四元数，`color.a=1` 是不透明。

`transient_local` 保留最新消息，RViz2 订阅端也选相同持久性，启动得晚也能收到圆盘。发布者退出后不能依赖这个缓存继续存在。

## 参数实验

编辑 `ros2_ws/src/motion2d_bringup/config/ch01.yaml`，半径改为 0.35、x 改为 1.0，重启 launch。预期直径 0.70 m、圆心 `(1,0)`。参数只在启动时读取；运行中 `ros2 param set` 不会重画，本章应重启。检查脚本验证默认配置，实验后恢复再检查。

## 常见问题和习题

- 找不到包：检查已经 source 工作空间的 `install/setup.*`。
- 不可见：检查 Fixed Frame、Marker 话题、alpha 与订阅持久性。
- 编译器标准错误：ROS 节点使用 C++20，不强制降为 C++17。
- 练习见 [ch01 习题](../../exercises/ch01/README.md)，正文见 [course.pdf](../../output/pdf/course.pdf)。
