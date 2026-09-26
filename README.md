# ROS 2 二维机器人运动规划学习框架

从二维世界、圆盘机器人和模拟传感器开始，逐步学习定位建图、路径与轨迹规划、跟踪控制。教材、代码和练习按章节对应。

**当前状态：第 01-03 章已完成，第 04 章实施中。** 已有随机世界、理想到点、速度控制与受力惯性模型，连续理想参考随后补齐。大纲提交为 `5c0c642`；每完成一个可运行、已验证的功能就单独提交，教材和练习同步更新。传感器、SLAM 和规划尚未实现。

[教材 PDF](output/pdf/course.pdf) · [实施进度](docs/PROGRESS.md) · [主机验收记录](docs/VALIDATION.md)

| 章节 | 内容与运行说明 | 练习 | 启动文件 |
| --- | --- | --- | --- |
| 01 | [环境与圆盘场景](chapters/ch01/README.md) | [ch01](exercises/ch01/README.md) | `ch01.launch.py` |
| 02 | [SE(2)、时钟与 TF](chapters/ch02/README.md) | [ch02](exercises/ch02/README.md) | `ch02.launch.py` |
| 03 | [随机圆与凸多边形世界](chapters/ch03/README.md) | [ch03](exercises/ch03/README.md) | `ch03.launch.py` |
| 04 | [机器人模型](chapters/ch04/README.md) | [ch04](exercises/ch04/README.md) | `ch04.launch.py` |

~~~bash
source /opt/ros/lyrical/setup.bash
cd /home/zdp/ForCodex/ros2_motion_planning_course/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch motion2d_bringup ch04.launch.py model:=inertial
~~~

上面使用 bash；zsh 对应使用 setup.zsh。启动时加 `rviz:=false` 可仅运行节点。同一时间只启动一个章节，避免多个时钟与 TF 发布者。初次学习请从第 01 章开始。

第 04 章支持 `model:=ideal`、`velocity`、`inertial`，分别接收位姿、速度、力/力矩。完整输入命令、参数与实验见 [第 04 章说明](chapters/ch04/README.md)。惯性模式初始静止，需要发送力；停发命令后仍可能滑行。

## 审阅入口

- [教材详细大纲](docs/COURSE_OUTLINE.md)：20 章、先修关系、代码入口、练习、验收与实施里程碑。
- [PDF 大纲审阅稿](output/pdf/course_outline.pdf)：章节摘要、模型建议和接口概览。
- [接口与建模草案](docs/INTERFACES.md)：状态、控制量、传感器、TF、话题、参数与扩展点。
- [AGENTS.md](AGENTS.md)：教学和开发规范；[agent.md](agent.md) 为同一规范的入口。
- [主机环境记录](docs/HOST_ENVIRONMENT.md)与[参考资料](docs/REFERENCES.md)。

## 建议主线

主机 ROS 2 Lyrical + Eigen + RViz2 + XeLaTeX。独立算法 C++17，ROS 节点按 Lyrical 要求使用 C++20。机器人以圆心状态建模，保留可配置碰撞半径；差速驱动作为附录扩展。

仿真与传感器 → 真值里程计和激光建图 → 激光/IMU 定位 → 闭环 SLAM → A* → ESDF 与安全走廊 → 二维 MINCO / SplineTrajectory → PD / MPC → 在线重规划。CUDA 激光仿真为选学章节。

采用已批准的全向圆盘与 C++17 算法主线，当前已完成第 01-04 章的环境与机器人模型。大纲 PDF 保留为原始审阅记录，实际教材使用独立 PDF。

## 当前目录

```text
AGENTS.md                     协作规范
agent.md                      规范入口
docs/                         大纲、接口、环境、资料
textbook/outline.tex           原始 PDF 审阅稿的 LaTeX 源
textbook/course.tex            实际教材入口，按章引用真实源码
textbook/Makefile              教材与大纲构建入口
output/pdf/course_outline.pdf 已编译的审阅稿
output/pdf/course.pdf         已实现章节的正文教材
ros2_ws/src/                   已实现的 ROS 包
chapters/                      逐章运行说明
exercises/                     独立练习、提示与参考解
```

具体完成范围见实施进度；后续章节的计划不代表已经实现。package.xml 暂用 Proprietary 表示尚未授予对外开放许可，不影响本地学习。

## 重新编译教材 PDF

```bash
cd /home/zdp/ForCodex/ros2_motion_planning_course
make -C textbook
```

需要 XeLaTeX、latexmk、ctex、Noto CJK 和 DejaVu 字体；本机已检测到。原始大纲可用 `make -C textbook outline` 重新编译。

批准记录：2026-09-26，用户同意大纲并要求开始实现，逐功能提交。
