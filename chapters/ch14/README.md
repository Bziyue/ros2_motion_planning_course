# 第14章：从路径到五次轨迹

先修04、11、13章。当前完成纯算法与实验；ROS理想连续执行继续实施。

```bash
ros2 run motion2d quintic_demo tmp/ch14_quintic
/usr/bin/python3 scripts/plot_ch14_quintic.py
```

TranslationState是同一固定坐标系的p/v/a。QuinticPiece保存正duration及2×6升幂系数，时间为段内秒。
interpolateQuintic用归一化时间求六个边界条件，最终转换为秒制系数。
PolynomialTrajectory检查C²连接，采样闭区间[0,total]；内部恰好连接时刻取右段，终点取末段。
超出时间范围拒绝；等待和终点保持是执行器的策略，不在数学采样器里偷偷截断。
所有平移量与yaw参考分离。系数CSV可以完整重建平移，不隐含朝向。

stopAtWaypoints给每个路点零v/a，T=max(min_duration,段长/nominal_speed)。
这是沿已知线段的单调运动，峰值速度为1.875倍平均速度；重复点为正时长停留。
任意非零连接导数可能让曲线偏离线段，必须另查避障/动力学。

一米静止到静止，T=2/4s的采样速度峰值.9375/.46875m/s，加速度1.44337/.360844m/s²，
jerk7.5/.9375m/s³。第三例有非零连接v/a，两段各4s，C²连续，不要求jerk连续。
测试使用解析缩放、100组seed=1414边界、导数差分、连接单侧、非法时长及重复点。
教材给公式与真实源码；ch14-1补完导数。常见错误：升降幂混淆、全局时间当局部时间、对秒制系数再除T。
