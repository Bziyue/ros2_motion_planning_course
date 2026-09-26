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
