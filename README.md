# ROS 2 二维机器人运动规划学习框架

从二维世界、圆盘机器人和模拟传感器开始，逐步学习定位建图、路径与轨迹规划、跟踪控制。教材、代码和练习按章节对应。

**当前状态：第 01–16 章已完成。** 已有随机世界、四种机器人模型、CPU 激光雷达、六轴白噪声 IMU、统一传感器时间、真值建图、激光/IMU 融合与回环 SLAM。大纲提交为 `5c0c642`；每完成一个可运行、已验证的功能就单独提交，教材和练习同步更新。已接入观测地图上的 A* 和 RViz 目标；已提供距离场与梯度可视化，第13章走廊已接入观测地图，第14章五次轨迹和理想连续执行已完成，第15章 MINCO 内核、软约束优化与观测地图ROS预览已完成，第16章样条、连续认证与执行已完成，第17章起跟踪控制继续实施。

[教材 PDF](output/pdf/course.pdf) · [实施进度](docs/PROGRESS.md) · [主机验收记录](docs/VALIDATION.md)

后续第 08–20 章已获连续实施授权。第 08 章已完成点到点/点到线配准、局部里程计与 ROS 接入；
运行入口见 [第 08 章说明](chapters/ch08/README.md) 和 [第 09 章说明](chapters/ch09/README.md)。
第 09–10 章已完成 EKF、时间融合、关键帧回环和观测地图重建，见 [第 10 章](chapters/ch10/README.md)。
后续继续配置空间、规划与轨迹控制。

| 章节 | 内容与运行说明 | 练习 | 启动文件 |
| --- | --- | --- | --- |
| 01 | [环境与圆盘场景](chapters/ch01/README.md) | [ch01](exercises/ch01/README.md) | `ch01.launch.py` |
| 02 | [SE(2)、时钟与 TF](chapters/ch02/README.md) | [ch02](exercises/ch02/README.md) | `ch02.launch.py` |
| 03 | [随机圆与凸多边形世界](chapters/ch03/README.md) | [ch03](exercises/ch03/README.md) | `ch03.launch.py` |
| 04 | [机器人模型](chapters/ch04/README.md) | [ch04](exercises/ch04/README.md) | `ch04.launch.py` |
| 05 | [CPU 激光雷达](chapters/ch05/README.md) | [ch05](exercises/ch05/README.md) | `ch05.launch.py` |
| 06 | [IMU 与传感器时间](chapters/ch06/README.md) | [ch06](exercises/ch06/README.md) | `ch06.launch.py` |
| 07 | [真值里程计与观测建图](chapters/ch07/README.md) | [ch07](exercises/ch07/README.md) | `ch07.launch.py` |
| 08 | [扫描匹配与激光里程计](chapters/ch08/README.md) | [ch08](exercises/ch08/README.md) | `ch08.launch.py` |
| 09 | [IMU 预测与时间融合](chapters/ch09/README.md) | [ch09](exercises/ch09/README.md) | `ch09.launch.py` |
| 10 | [关键帧回环 SLAM](chapters/ch10/README.md) | [ch10](exercises/ch10/README.md) | `ch10.launch.py` |
| 11 | [配置空间与 A*](chapters/ch11/README.md) | [ch11](exercises/ch11/README.md) | `ch11.launch.py` |
| 12 | [距离场与梯度](chapters/ch12/README.md) | [ch12](exercises/ch12/README.md) | `ch12.launch.py` |
| 13 | [凸安全走廊](chapters/ch13/README.md) | [ch13](exercises/ch13/README.md) | `ch13.launch.py` |
| 14 | [五次轨迹](chapters/ch14/README.md) | [ch14](exercises/ch14/README.md) | `ch14.launch.py` |
| 15 | [二维 MINCO](chapters/ch15/README.md) | [ch15](exercises/ch15/README.md) | `ch15.launch.py`、`minco_demo` |
| 16 | [二维 Spline2D](chapters/ch16/README.md) | [ch16](exercises/ch16/README.md) | `ch16.launch.py`、`spline_demo`、上游对照 |
| 17 | [前馈与PD跟踪](chapters/ch17/README.md) | [ch17](exercises/ch17/README.md) | 核心实验 `pd_tracking_demo` |

~~~bash
source /opt/ros/lyrical/setup.bash
cd /home/zdp/ForCodex/ros2_motion_planning_course/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch motion2d_bringup ch08.launch.py
~~~

上面使用 bash；zsh 对应使用 setup.zsh。启动时加 `rviz:=false` 可仅运行节点。同一时间只启动一个章节，避免多个时钟与 TF 发布者。初次学习请从第 01 章开始。

第 04 章支持 `model:=ideal`、`velocity`、`inertial`，分别接收位姿、速度、力/力矩；`model:=reference` 自动运行解析圆轨迹，适合后续传感器实验。完整输入命令、参数与实验见 [第 04 章说明](chapters/ch04/README.md)。惯性模式初始静止，需要发送力；停发命令后仍可能滑行。

第 05 章默认沿圆参考运动并发布 720 束、10 Hz 的 CPU snapshot 激光，测距噪声标准差 0.01 m。RViz2 同时显示光束和命中点；参数、无噪声基准、空房间统计与束数实验见 [第 05 章说明](chapters/ch05/README.md)。

第 06 章加入 200 Hz 的 /imu/data_raw、可配置的六轴白噪声与采样时间调度，也支持 137 Hz IMU / 7 Hz 雷达。仅 reference/inertial 支持 IMU，不发布真值姿态。运行命令、静止比力、噪声/漂移与时间实验见 [第 06 章说明](chapters/ch06/README.md)。本机 RViz 在 reset/退出时的已知异常及显示建议已记录，自动重置验收可加 `rviz:=false`。

第 07 章用显式真值里程计定位，仅凭扫描构建点云与占据栅格。RViz 默认关闭完整世界对照，展示未知区域与可见轮廓；提供分辨率、噪声、墙厚和独立真值栅格的可复现实验，详见 [第 07 章说明](chapters/ch07/README.md)。这是真值定位基线，尚不包含扫描匹配或 SLAM。

## 审阅入口

- [教材详细大纲](docs/COURSE_OUTLINE.md)：20 章、先修关系、代码入口、练习、验收与实施里程碑。
- [PDF 大纲审阅稿](output/pdf/course_outline.pdf)：章节摘要、模型建议和接口概览。
- [接口与建模草案](docs/INTERFACES.md)：状态、控制量、传感器、TF、话题、参数与扩展点。
- [AGENTS.md](AGENTS.md)：教学和开发规范；[agent.md](agent.md) 为同一规范的入口。
- [主机环境记录](docs/HOST_ENVIRONMENT.md)与[参考资料](docs/REFERENCES.md)。

## 建议主线

主机 ROS 2 Lyrical + Eigen + RViz2 + XeLaTeX。独立算法 C++17，ROS 节点按 Lyrical 要求使用 C++20。机器人以圆心状态建模，保留可配置碰撞半径；差速驱动作为附录扩展。

仿真与传感器 → 真值里程计和激光建图 → 激光/IMU 定位 → 闭环 SLAM → A* → ESDF 与安全走廊 → 二维 MINCO / SplineTrajectory → PD / MPC → 在线重规划。CUDA 激光仿真为选学章节。

采用已批准的全向圆盘与 C++17 算法主线。大纲 PDF 保留为原始审阅记录，实际教材使用独立 PDF。

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

需要 XeLaTeX、latexmk、ctex、Noto CJK 和 DejaVu 字体，以及 Poppler（poppler-utils）与系统 Python 的 Pillow（python3-pil）；本机均已有。
原始大纲可用 `make -C textbook outline` 重新编译。

每次构建都会生成 144 dpi 的逐页 PNG、四页联系表、浏览入口与待办清单，位于
`tmp/pdfs/review/course/`（大纲为 `outline/`）。打开其中的 `index.html` 可逐页查看并点击放大。
**生成预览不代表通过检查**：交付前必须检查并修复重叠、失真、截断等问题，重编后复核最新版；
具体步骤见 [PDF 视觉复核流程](docs/PDF_REVIEW.md)，结果写入验收记录。

批准记录：2026-09-26，用户同意大纲并要求开始实现，逐功能提交。
