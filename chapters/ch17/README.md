# 第17章：前馈 + PD 跟踪受力圆盘

先修04/14。状态与参考均在odom，控制输出为N/Nm；控制器模型与仿真参数分别配置。
`trackPd`计算平移F=m(a_ref+kp e_p+kd e_v)+c v，yaw独立包角误差并做同样的惯量/阻力换算。
返回请求与实际限幅力矩及饱和标志；每轴独立力限，不是范数限；不存在速度硬裁剪。
`dampingBrake`产生当前速度反向的新限幅阻尼命令；不保证停止路径避障，也不保证变化率限制。

```bash
ros2 run motion2d pd_tracking_demo tmp/ch17_pd
/usr/bin/python3 scripts/plot_ch17_pd.py
```

圆形与八字参考用2s相位渐入，起点v/a均零；独立yaw=.4sin(phase/2)。
空8m方形环境只用于评估净空，机器人半径.2m；参考给定，不称为规划避障实验。
24s、200Hz真值动力学、50Hz控制且无通信延迟。实际m=1kg、c=.15kg/s、I=.02、角阻力.01；kp=4,kd=3。
匹配前馈PD的圆/八字位置RMS为0.000393/0.000898m；去前馈0.06112/0.09151m；估计质量.6kg为0.04262/0.06691m。
把每轴力降到.2N后RMS达0.9713/0.9508m；提高反馈增益不意味着能突破实际执行器限制。
理想基线误差0但没有物理控制输入，用NaN报告力峰值。ROS实时链路另行测量，不将核心实验当作通信系统测量。


## ROS 运行与验收

```bash
ros2 launch motion2d_bringup ch17.launch.py rviz:=false
# 同域另一终端，source 相同环境
/usr/bin/python3 scripts/check_ch17.py
```

默认暂停。手动运行：`ros2 service call /sim/pause std_srvs/srv/SetBool '{data: false}'`。
参考/实际力分别在`/control/reference`、`/control/wrench_requested`和`/command/wrench`；状态`/control/status`。
里程计twist在body，控制前转odom。唯一观测时间戳触发50Hz控制；暂停去重，时钟回退清空参考。
`.12s`观测超时或积压旧数据触发最后已知速度的限幅制动，不回放旧跟踪输入。
`/tracker/enable` SetBool关闭控制也制动，reset不会擅自重新启用。
12s ROS：600个匹配时刻，RMS .00094048m、最大 .0017583m、最小净空2.79979m；GUI未验收。

独立边界测试：另起域运行`tracker_node --ros-args -p use_sim_time:=true -p reference.source:=trajectory`，
同域运行`scripts/check_tracker_boundary.py`；不要在该域同时启动模拟器，脚本提供唯一/clock。
外部轨迹要求静止且匹配的起终p/v/a、未来起点和yaw；执行中不替换，在线接续见ch19。
