# 选学附录

- A：数学回查、SE(2)/协方差/最小二乘/凸集/强凸QP，与中央差分练习；`exercises/appA`。
- B：无滑移差速模型，左右轮速与精确圆弧积分；`exercises/appB`。
- C：偏置随机游走独立实验，延迟/逐束畸变/复杂障碍/重定位扩展边界；`exercises/appC`。
- D：传感器rosbag录制、关闭模拟器后SLAM回放、外部对照协议、Doxygen和实验报告；`exercises/appD`。

附录用于主线之外的扩展，不改变第01–20章编号或默认全向受力模型。

## B 差速模型运行

先按仓库README构建/source，在仓库根目录运行：
```bash
ros2 run motion2d differential_drive_demo > textbook/data/appB_circle.csv
/usr/bin/python3 scripts/plot_appB.py
```
默认轮半径.05m、轮距.4m，左2/右6rad/s；t=pi秒处(.4,.4)m、yaw=pi/2，4pi秒一圈。
该演示复用原有SE(2)精确速度积分；这是独立无滑移运动学示例，无障碍、不发布ROS话题，未新增模拟器模式。
第18章全向力MPC不直接适用；停点/倒车模式、轮动力学与NMPC见教材扩展说明。

## C 偏置与白噪声

```bash
ros2 run motion2d bias_walk_demo tmp/appC_bias
```
生成单条路径path.csv与两种采样率各5000次独立试验的summary.csv。
密度.002(rad/s)/sqrt(s)、10s终点方差理论4e-5(rad/s)²；100/200Hz实测3.87825e-5/3.99626e-5。
该工具不自动注入主线IMU；基础参数、reset与滤波器行为不变。其余C主题为明确边界的选做设计题。

## D 只回放传感器

```bash
/usr/bin/python3 scripts/run_appD.py --output tmp/my_replay
```
自动运行真实录制、关闭模拟器、仅估计栈半速回放、逐字段散列与地图验收；目录已存在时换一个输出名。
结果为sensors/、summary.json和进程日志。回放已有bag：
```bash
ros2 launch motion2d_bringup replay_slam.launch.py rviz:=true
# Another terminal, with the same ROS_DOMAIN_ID and sourced workspace:
ros2 bag play tmp/my_replay/sensors --clock 200 --rate 0.5
```
停止所有其他模拟器/时钟；bag只含/scan和/imu/data_raw，启动文件提供本课程重合的静态外参。
主机验证为无界面，不宣称新的RViz画面已验收。外部SLAM对照为设计练习，必须记录其实际输入/版本/预算。

## D API与报告

仓库根目录运行 `bash scripts/build_api.sh`，打开`tmp/doxygen/html/index.html`。
系统Doxygen优先；当前主机自动使用已有局部1.15.0工具，其他机器应先安装Doxygen和LaTeX。
Doxyfile保留在版本控制中，生成HTML在ignored tmp中；文档语法告警会使生成失败。
[实验报告模板](../../docs/EXPERIMENT_REPORT_TEMPLATE.md)单列复现条件、输入隔离、指标和全部失败；
[验收记录](../../docs/VALIDATION.md)是本课程已实测的数据，模板不代表实测。
