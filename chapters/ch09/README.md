# 第 09 章：IMU 预测与激光校正的二维融合

先修第 06、08 章。实现平面 EKF、合成验证、真实射线扫描回放及在线 IMU/激光时间融合。

状态顺序固定为 `[px,py,vx,vy,yaw,bax,bay,bg]`；平移在 odom，偏置在机体坐标轴。
水平运动下原始 IMU 的 x/y 比力就是机体系 x/y 加速度；不使用 orientation 字段，不从真值读取航向。
`predictImu` 使用步首航向旋转并做常加速度积分，旋转在单步内保持不变，是离散近似。
`PlanarEkf` 默认锁定偏置为零；`EkfConfig.estimate_bias=true` 开启最后三个状态。

源码在 `estimation/imu_predictor.cpp` 与 `estimation/planar_ekf.cpp`。F/G 均有独立差分检查；
过程协方差为 `F P Fᵀ + G diag(sample_variance) Gᵀ`。
偏置随机游走标准差是每 sqrt(s)，与模拟 IMU 的每采样标准差不同；开启时用 `dt * sigma_bias²`。
位姿更新使用最短角残差、平方 Mahalanobis 门限与 Joseph 协方差形式。

重新构建并加载 ROS 工作区后，在仓库根目录运行：

```bash
ros2 run motion2d planar_ekf_demo tmp/ch09_filter
```

这一步故意隔离滤波器：位姿输入是从解析轨迹生成的带噪合成测量，**不是 SLAM 验证**。
使用同一组 200 Hz IMU、10 Hz 位姿观测，20 s 轨迹，4–5 s 暂停位姿校正；seed=9090。
注入固定机体系加计偏置 (.08,-.04) m/s² 与陀螺偏置 .01 rad/s；算法没有收到这些答案。
初速先验为零、标准差 1 m/s，不能从第一帧 IMU 得知真实初速。
对比纯 IMU、锁零偏置融合与估计偏置融合；报告全时段/缺测段位置 RMSE 及接受/拒绝计数。

练习、提示、独立检查程序见 exercises/ch09。常见错误是噪声方差少乘一次 dt、
使用真值 yaw、在世界系扣机体偏置、忘记角残差归一化，或把相关的局部地图观测当成独立真值。


## 在线运行与时间接口

```bash
ros2 launch motion2d_bringup ch09.launch.py rviz:=false
# 另一个终端加载相同 ROS 环境
/usr/bin/python3 scripts/check_ch09.py
ros2 run motion2d lidar_imu_demo tmp/ch09_fusion
/usr/bin/python3 scripts/plot_ch09_fusion.py
```

默认200 Hz原始IMU、10 Hz snapshot激光。激光里程计仅接收 `/scan`，输出 `/odometry/lidar`；
融合器只读它和 `/imu/data_raw`。`FusionTimeline` 保留2 s原始IMU，使用区间左样本保持；
激光早到先排队，晚到在采样时刻校正后重放到最新IMU，不读取系统墙钟或TF。
非整数频率同样按整数纳秒处理。IMU重复/回退与reset由边界处理，跨越超过.1 s则失去初始化。
两个输入分别收到新原点时，保留先到的新试验数据；像前几章一样，reset要先暂停并清空外部旧队列。

`/odometry/estimated` 是IMU频率预测，经estimated适配器输出统一 `/odometry` 和唯一运动TF；
同一IMU时刻已发布后到达的校正，在下一次IMU输出体现，不重复发布相同时间的TF。
`/odometry/scan` 是激光时刻的校正位姿，mapping通过remap只使用此流，避免抢先把未校正预测插入地图。
`/fusion/status` 区分 waiting_laser、waiting_imu、laser_corrected、innovation_rejected、laser_too_old、
imu_gap与laser_stale；默认距有效激光超过.5 s报告stale，仍传播IMU，不能当成可靠定位无限制控制。

RViz去掉 `rviz:=false`，预期看到高频估计轨迹、低频扫描/占据图；地图来源仍是观测。
本机此前OpenGL截图与reset缓存有问题，这次验收使用真实ROS无界面运行，没有声称GUI画面通过。
可只启动fusion_node，在独立ROS_DOMAIN_ID运行 `scripts/check_ch09_time.py`，验证两种reset顺序及晚到/早到的非对齐观测。

回放：world=42、激光噪声seed=4242、IMU seed=6060，360束、距离噪声.01 m、半径1 m、10 s；
两种角速度.4/1.2 rad/s，3–3.5 s缺扫，每次激光结果延迟30 ms。IMU使用第06章默认白噪声、不注入偏置；
开启偏置估计但无真值输入。在全部可用IMU时刻比较融合与“保持最新激光位姿”，这包含后者的采样延迟，
不是宣称ICP单帧精度提升。结果原始CSV在运行目录，教材读取 `textbook/data/ch09_fusion.csv`。

限制：局部地图相关性、固定测量协方差、区间旋转保持、将被分开的同一个IMU样本噪声近似独立；
因此不宣称统计一致。原始观测为snapshot，逐束畸变/去畸变作为可选进阶，没有启用一个空实现。
