# ROS 2 二维机器人运动规划学习框架

从二维世界、圆盘机器人和模拟传感器开始，逐步学习定位建图、路径与轨迹规划、跟踪控制。教材、代码和练习按章节对应。

**当前状态：大纲已批准，开始实施第 01-03 章。** 大纲提交为 `5c0c642`。按用户要求，每完成一个可运行、已验证的功能就单独提交，教材和练习随功能同步更新。实际完成范围见 [实施进度](docs/PROGRESS.md)。

## 审阅入口

- [教材详细大纲](docs/COURSE_OUTLINE.md)：20 章、先修关系、代码入口、练习、验收与实施里程碑。
- [PDF 大纲审阅稿](output/pdf/course_outline.pdf)：章节摘要、模型建议和接口概览。
- [接口与建模草案](docs/INTERFACES.md)：状态、控制量、传感器、TF、话题、参数与扩展点。
- [AGENTS.md](AGENTS.md)：教学和开发规范；[agent.md](agent.md) 为同一规范的入口。
- [主机环境记录](docs/HOST_ENVIRONMENT.md)与[参考资料](docs/REFERENCES.md)。

## 建议主线

主机 ROS 2 Lyrical + C++17 / Eigen + RViz2 + XeLaTeX。机器人以圆心状态建模，保留可配置碰撞半径。先用理想全向运动模型，再用受平面力与偏航力矩驱动的惯性模型；差速机器人作为附录扩展。

仿真与传感器 → 真值里程计和激光建图 → 激光/IMU 定位 → 闭环 SLAM → A* → ESDF 与安全走廊 → 二维 MINCO / SplineTrajectory → PD / MPC → 在线重规划。CUDA 激光仿真为选学章节。

采用已批准的全向圆盘与 C++17 主线，首先完成第 01-03 章的小型可视化环境。大纲 PDF 保留为原始审阅记录，后续实际教材使用独立 PDF。

## 当前目录

```text
AGENTS.md                     协作规范
agent.md                      规范入口
docs/                         大纲、接口、环境、资料
textbook/outline.tex           本轮 PDF 审阅稿的 LaTeX 源
textbook/Makefile              本轮审阅稿构建入口
output/pdf/course_outline.pdf 已编译的审阅稿
ros2_ws/README.md              未来 ROS 包布局说明
exercises/README.md            未来习题组织说明
```

上述目录不表示课程代码已经完成。未来的包与文件路径在大纲中都标为“计划”。

## 重新编译大纲 PDF

```bash
cd /home/zdp/ForCodex/ros2_motion_planning_course
make -C textbook
```

需要 XeLaTeX、latexmk、ctex、Noto CJK 字体和 DejaVu 字体；本机已检测到。正文教材将在获批后逐章编写，届时增加独立的教材构建目标。

批准记录：2026-09-26，用户同意大纲并要求开始实现，逐功能提交。
