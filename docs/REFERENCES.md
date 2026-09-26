# 参考资料与版本

查阅日期：2026-09-26。下列为原始规范、作者论文或官方仓库；本轮仅查阅，未复制第三方实现、安装依赖或宣称已复现性能。

实施补充：[Lyrical 官方支持平台与语言要求](https://github.com/ros2/ros2_documentation/blob/rolling/source/Get-Started/Releases/lyrical/supported-platforms.rst) 明确 ROS 接口要求 C++20；因此节点用 C++20，纯算法保留 C++17。[更新后的安装源文档](https://github.com/ros2/ros2_documentation/blob/rolling/source/Get-Started/Installation/Ubuntu-Install-Debs.rst) 用于第 01 章。原始大纲 PDF 保留为历史记录，不再以其中 C++17 概括 ROS 节点要求。

1. **ROS 坐标与单位**：[REP-103 源文件](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)。SI 单位、右手系、机体轴与偏航方向。
2. **ROS 坐标树**：[REP-105 源文件](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)。`map → odom → base_link` 和变换发布责任。
3. **传感器消息**：[LaserScan](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg)、[Imu](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg)。网络文档是 rolling 分支；本次也检查了主机 `/opt/ros/lyrical/share/sensor_msgs/msg/` 下的定义，实施以本机定义为准。
4. **ROS 2 Lyrical**：[发行配置](https://github.com/ros/rosdistro/blob/master/lyrical/distribution.yaml)、[安装入口](https://docs.ros.org/en/lyrical/Installation.html)。本次文档站点受访问保护，未据其页面作安装承诺；已用主机软件包和 doctor 输出确认实际环境。
5. **MINCO**：Wang, Zhou, Xu, Gao, *Geometrically Constrained Trajectory Optimization for Multicopters*, IEEE T-RO 38(5), 2022, 3259-3278。[论文 DOI](https://doi.org/10.1109/TRO.2022.3160022)、[作者 GCOPTER 仓库](https://github.com/ZJU-FAST-Lab/GCOPTER)。课程只讲二维最小 jerk 子问题；不整体移植其 ROS 1 演示。
6. **SplineTrajectory**：[用户指定仓库](https://github.com/Bziyue/SplineTrajectory)，本次查询 HEAD 为 `126525e49a43b0548bc6960e4979dd6b6258f289`。[固定版本中文说明](https://github.com/Bziyue/SplineTrajectory/blob/126525e49a43b0548bc6960e4979dd6b6258f289/README_zh.md)、[许可证](https://github.com/Bziyue/SplineTrajectory/blob/126525e49a43b0548bc6960e4979dd6b6258f289/LICENSE)。其任意维轨迹、块三对角求解、解析能量梯度和凸包接口为第 16 章参考。课程拟实现二维五次版本，并用 `QuinticSplineND<2>` 作对照；不将其与 MINCO 描述成两种完全不同的目标函数，也不等同于一般 B 样条。
7. **MPC 的 QP 求解器候选**：[OSQP 官方仓库](https://github.com/osqp/osqp)。第 18 章实施前再验证接口并锁定版本；先解释离散动力学与 QP 矩阵，再接入求解器。

SplineTrajectory 的性能数字依赖具体问题、优化设置和硬件。教材将自行测量，不直接承诺上游测试的加速比。若引入上游代码，保留其许可及第三方声明；课程自身的对外发布许可证尚未确定。

第 06 章补充：[REP-145 原始文件](https://github.com/ros-infrastructure/rep/blob/master/rep-0145.rst)（状态为 Draft）用于核对传感器轴、比力、静止向上 +g 与原始 IMU 不含融合姿态的约定。再次核对本机 Imu.msg：协方差为行主序，未知为全零，不提供某项估计时其协方差首元素为 -1。课程使用 /imu/data_raw，不把真值姿态写入 orientation。

第 05 章补充：再次核对主机 LaserScan.msg 与 [官方原始定义](https://raw.githubusercontent.com/ros2/common_interfaces/rolling/sensor_msgs/msg/LaserScan.msg)，并阅读 [REP-117](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0117.rst)。REP-117 区分 -inf（过近）、NaN（无效）与 +inf（无回波）。已批准的课程契约将近距盲区统一编码为 NaN、让建图跳过，因此不宣称完整实现 REP-117 的三种编码；转换到真实硬件时必须保留这一区别。官方 HTML 文档本次受访问保护，采用原始源文件和本机定义核对。

第 07 章补充：核对本机 nav_msgs/Odometry、OccupancyGrid、MapMetaData 与 sensor_msgs/PointCloud2，以及 [Odometry 原始定义](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg)、[OccupancyGrid 原始定义](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg)、[PointCloud2 原始定义](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg)。Odometry 的位姿与速度使用不同的声明帧；栅格按 x 最快的行主序排列，原点是 (0,0) 格的左下角。OccupancyGrid 值域由应用约定，本课程采用 -1 未知、0-100 占据概率百分比。

## 二维欧氏距离变换

Felzenszwalb 与 Huttenlocher，*Distance Transforms of Sampled Functions*，Theory of Computing 8 (2012), 415–428。
[正式论文及 DOI](https://theoryofcomputing.org/articles/v008a019/)；[作者保存的 PDF](https://cs.brown.edu/people/pfelzens/papers/dt-final.pdf)。
第12章使用可分离的抛物线下包络方法，按公式独立实现，没有复制上游代码。
场的正负约定、栅格面积/插值下界以及ROS接口是本课程的具体选择，不将中心EDT称为方格边界精确SDF。


第15章再次查阅[MINCO论文v4第IV节](https://arxiv.org/html/2103.00190v4)，核对位置路点对应的C⁴最优性、带状线性表示与伴随。
课程由最小jerk变分条件独立组装二维系统，未复制上游代码；无主元教学求解器限制与密集KKT对照已说明。

### 第16章实际对照

固定上游126525e49a43b0548bc6960e4979dd6b6258f289，QuinticSplineND<2>，MIT LICENSE随临时克隆保留。课程Spline2D根据端点能量二次型独立实现；未复制上游源码。`scripts/benchmark_ch16.sh`将三种实现以相同-O3/-fno-fast-math编译进一个程序，测构造+能量+完整能量梯度，结果见textbook/data/ch16_comparison.csv。

## 第18章 QP

[OSQP 官方求解器说明](https://osqp.org/docs/solver/index.html)，2026-09-26核对二次型、ADMM线性系统/投影、残差终止及原始不可行证书。
课程为独立编写的强凸稠密教学实现，不复制OSQP源码，不包含其完整预处理/缩放/抛光功能。
