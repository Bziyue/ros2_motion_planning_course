# 主机环境记录

检查日期：2026-09-26。只做检测，没有安装依赖、修改系统配置或启动课程节点。

| 项目 | 本次检测结果 | 对课程的意义 |
| --- | --- | --- |
| 系统 | Ubuntu 26.04.1 LTS，x86_64 | 以当前主机为首个验证平台 |
| ROS 2 | Lyrical，`/opt/ros/lyrical` | 使用主机 ROS 2，不假设 Humble/Jazzy |
| rclcpp | 32.0.3 | 核心使用 C++ 节点 |
| RViz2 | 15.2.6，命令与软件包存在 | 第 01 章再验证 GUI、渲染及显示配置 |
| RMW | `rmw_fastrtps_cpp` | 当前 `ros2 doctor --report` 的检测结果 |
| ROS 基础工具 | colcon、tf2_ros、sensor_msgs 存在 | 具备初始教学工具链 |
| C++ | GCC 15.2.0，CMake 4.2.3，Eigen 3.4.0 | 本项目算法目标为 C++17 |
| Python | 系统 Python 3.14.4 | ROS launch 使用系统 Python；避免混用虚拟环境 |
| 排版 | TeX Live 2026，XeLaTeX、LuaLaTeX、latexmk | 使用 XeLaTeX 与 ctex |
| 字体 | Noto Sans/Serif CJK SC 存在 | 可排版中文正文和标题 |
| PDF 检查 | pdftoppm、pdfinfo、pdftotext 存在 | 编译后渲染审阅 |
| GPU | NVIDIA GeForce RTX 5060，8151 MiB，驱动 595.91.07 | CUDA 课程具有硬件基础 |
| CUDA 工具链 | PATH 中未找到 `nvcc`；常见 `/usr/local/cuda*`、`/opt/cuda*` 路径未找到 | 不能据此断言任意位置都未安装；第 20 章再检查/配置 |
| Doxygen | PATH 中未找到命令 | 编码仍遵守 Doxygen 注释语法；实施时再补齐文档生成工具 |

`ros2 doctor --report` 成功返回环境信息，本次没有发现活动话题。它不能替代节点构建、通信和 RViz2 图形测试。为避免记录无关网络信息，本仓库不保存完整原始报告。

第 01 章计划给出当前主机已有环境的检查步骤，并提供干净主机上的安装说明。具体软件源、支持平台与安装命令在实施当天依据 ROS 官方资料复核。不会把其他 ROS 发行版的命令直接套到本机。
