# 第16章：固定二维 Spline2D

先修14、15。独立教学实现，不是上游包装；固定同一clamped最小jerk问题。
端点映射C=E(T)W消去单段系数，R=EᵀQE得到只耦合相邻内部v/a的2×2块三对角系统。
块Cholesky消元与伴随均O(N)；解析能量梯度直接用高阶导数边界项。
普通ROS构建不依赖下载上游。

```bash
ros2 run motion2d spline_demo tmp/ch16_spline
bash scripts/benchmark_ch16.sh
/usr/bin/python3 scripts/plot_ch16_spline.py
```

可选对照锁定 https://github.com/Bziyue/SplineTrajectory ，SHA126525e49a43b0548bc6960e4979dd6b6258f289，MIT许可证保存在下载目录，未复制上游实现到课程。
对比QuinticSplineND<2>，同端点p/v/a、路点、时长和SI单位；使用同一g++ -O3 -fno-fast-math C++17程序。
21组×20次，10次预热，取每组平均的中位数；计时包含构造、解析能量与完整能量梯度，不含I/O/地图/ROS/优化外层。
测试1–12段与MINCO系数/梯度一致，通用伴随通过软代价中心差分；MINCO另有独立KKT测试。
当前实现保留6×6矩阵运算便于阅读，上游使用更专门化公式。性能只代表本机/此问题，不能由O(N)推出固定加速比。
连续Bézier认证与优化共享接口随后补充。

连续认证已实现：bezierMap把原曲线及导数转换为控制点，de Casteljau二分保持原曲线。
certifyBezier要求每段对应已验证配置空间区域，检查全部位置半空间和速度/加速度控制点范数上界。
默认5层二分、1e-9数值容差；这不是物理安全裕量，跟踪/定位误差不包含在证书内。
`ros2 run motion2d bezier_demo tmp/ch16_bezier`对比初始、采样软惩罚、控制点惩罚；最后一组收敛且连续认证通过。
三组目标定义不同，不比较总代价优劣。`/usr/bin/python3 scripts/plot_ch16_bezier.py`生成等比例几何与速度图。

ROS：`ros2 launch motion2d_bringup ch16.launch.py rviz:=true`；Goal Pose后显式调用`/trajectory/execute`。
`/plan/certified_candidate`可靠保留，odom帧，规划odom时间戳，start_time=0只表示局部原点，空pieces清除。
仅收敛且连续认证通过才能生成候选。trajectory.source=certified检查时效≤0.5s、静止端点/当前状态一致后，发送now+0.25s的实际命令。
`/usr/bin/python3 scripts/check_ch16.py`检查独立p/v/a、候选不自动执行、非法目标和reset清除。GUI仍未验收。
