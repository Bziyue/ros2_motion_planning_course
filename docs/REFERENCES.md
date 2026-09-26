# 参考资料与版本

查阅日期：2026-09-26。下列为原始规范、作者论文或官方仓库；本轮仅查阅，未复制第三方实现、安装依赖或宣称已复现性能。

1. **ROS 坐标与单位**：[REP-103 源文件](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)。SI 单位、右手系、机体轴与偏航方向。
2. **ROS 坐标树**：[REP-105 源文件](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)。`map → odom → base_link` 和变换发布责任。
3. **传感器消息**：[LaserScan](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg)、[Imu](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg)。网络文档是 rolling 分支；本次也检查了主机 `/opt/ros/lyrical/share/sensor_msgs/msg/` 下的定义，实施以本机定义为准。
4. **ROS 2 Lyrical**：[发行配置](https://github.com/ros/rosdistro/blob/master/lyrical/distribution.yaml)、[安装入口](https://docs.ros.org/en/lyrical/Installation.html)。本次文档站点受访问保护，未据其页面作安装承诺；已用主机软件包和 doctor 输出确认实际环境。
5. **MINCO**：Wang, Zhou, Xu, Gao, *Geometrically Constrained Trajectory Optimization for Multicopters*, IEEE T-RO 38(5), 2022, 3259-3278。[论文 DOI](https://doi.org/10.1109/TRO.2022.3160022)、[作者 GCOPTER 仓库](https://github.com/ZJU-FAST-Lab/GCOPTER)。课程只讲二维最小 jerk 子问题；不整体移植其 ROS 1 演示。
6. **SplineTrajectory**：[用户指定仓库](https://github.com/Bziyue/SplineTrajectory)，本次查询 HEAD 为 `126525e49a43b0548bc6960e4979dd6b6258f289`。[固定版本中文说明](https://github.com/Bziyue/SplineTrajectory/blob/126525e49a43b0548bc6960e4979dd6b6258f289/README_zh.md)、[许可证](https://github.com/Bziyue/SplineTrajectory/blob/126525e49a43b0548bc6960e4979dd6b6258f289/LICENSE)。其任意维轨迹、块三对角求解、解析能量梯度和凸包接口为第 16 章参考。课程拟实现二维五次版本，并用 `QuinticSplineND<2>` 作对照；不将其与 MINCO 描述成两种完全不同的目标函数，也不等同于一般 B 样条。
7. **MPC 的 QP 求解器候选**：[OSQP 官方仓库](https://github.com/osqp/osqp)。第 18 章实施前再验证接口并锁定版本；先解释离散动力学与 QP 矩阵，再接入求解器。

SplineTrajectory 的性能数字依赖具体问题、优化设置和硬件。教材将自行测量，不直接承诺上游测试的加速比。若引入上游代码，保留其许可及第三方声明；课程自身的对外发布许可证尚未确定。
