# 第 09 章练习

1. 理解：推导 dv_next/dyaw 和 dv_next/dba；解释为什么加速度计偏置在机体系。
2. 代码 ch09-1：补完 starter/yaw_residual.hpp。两输入分别在 ±pi 两侧时，残差仍应很小。
3. 实验：运行 planar_ekf_demo，把偏置置零或把校正噪声标准差减半；同时比较误差和拒绝数。

仓库根目录运行：

```bash
g++ -std=c++17 -Iexercises/ch09/starter exercises/ch09/check_yaw.cpp -o /tmp/ch09_yaw
/tmp/ch09_yaw
```

通过标准：四种角度残差与独立手算值误差小于 1e-12，尤其覆盖跨界和多圈输入。
starter 应失败；换为 solutions 检查参考解。提示与答案分开放置。

4. 代码 ch09-2：补完 `held_duration.hpp` 的区间交集。激光时间可能落在两个 IMU 样本之间；
   先用整数纳秒求交集，再换算秒。用 `check_time.cpp` 替换上面编译命令中的检查文件。
   四例误差应小于 1e-15 s，包括恰好接触和大时间戳下的 5 ns 小区间。
5. 实验：运行 `lidar_imu_demo tmp/ch09_fusion`，分别比较 .4/1.2 rad/s 的转动、半秒缺扫和 30 ms 延迟。
   分别报告高频位置 RMSE、航向 RMSE、缺扫段 RMSE 与拒绝数；保留相同世界及噪声种子。
   将延迟增到超过 history_seconds，解释为什么必须报告过旧，而不是伪装成当前观测。
