# 第 07 章练习

1. 理解：机器人 yaw=pi/2、世界速度 (2,1) m/s 时，Odometry.twist 应填什么？为什么位置要平移，速度转换却不加位置偏移？
2. 代码 ch07-1：补完 starter/body_velocity.hpp，区分 header.frame_id 与 child_frame_id。
3. 实验：运行 check_ch07_odometry.py，观察世界系圆周速度方向在变化，机体系线速度却始终约为 (.4,0)。核对 /tf 只有 truth_odometry 发布；说明为什么不能同时开启 simulator 的运动 TF。

从仓库根目录编译；将 starter 换为 solutions 可验证参考解：

~~~bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch07/starter \
  exercises/ch07/check_velocity.cpp -o /tmp/ch07_velocity
/tmp/ch07_velocity
~~~

评分：旋转方向、单位和两种帧的解释正确；能说明本章使用真值是已知定位基线，不是完成了 SLAM。提示见 [hints.md](hints.md)。未完成练习不影响演示构建。

## 扫描点投影

- 理解：+inf 能否直接当成 range_max 的回波？为什么当前机体姿态不能替代采样姿态？
- 代码 ch07-2：补完 starter/scan_point.hpp。将上述编译命令的源文件换为 check_projection.cpp；三组手算坐标误差须小于 1e-12 m。
- 实验：运行 check_ch07_cloud.py；再改 lidar.rate=7.0 重启，验证非整数 tick 的扫描仍匹配同一时间的里程计。主配置 10 Hz 的检查脚本需要 t=1 s 回波；7 Hz 同样含 t=1 s。
- 评分：旋转顺序、角度方向和时间语义正确；能说明点云只表示回波表面，缺点区域不等于自由。

## 占据栅格与射线遍历

- 理解：第一次自由证据为何输出 40，而不是 0？未观测的 -1 和已观测的 50 为什么不同？说明 +inf 与 NaN 对更新的区别。
- 代码 ch07-3：补完 starter/grid_traversal.hpp 的 DDA 循环，源文件换为 check_traversal.cpp 编译。覆盖水平/竖直、正反对角线、角点、同格和终点格线；必须终止且不重复访问。
- 实验：运行 check_ch07_map.py，然后查看 RViz 的地图；完整世界对照默认关闭。被挡住的区域和远处目标应仍是灰色未知，不能因没有点云就刷白。reset 后首帧地图应与第一次首帧逐字节一致。
- 评分：访问次序六例全部通过；讲清有限命中的终点优先、无回波不制造边界障碍、每帧每格至多更新一次。
