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
