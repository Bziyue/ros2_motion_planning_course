# 第 09 章：IMU 预测与激光校正的二维融合

先修第 06、08 章。当前实现平面 EKF 数学内核与合成位姿传感器实验；真实激光/IMU 的在线时间对齐接入是下一功能。

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
