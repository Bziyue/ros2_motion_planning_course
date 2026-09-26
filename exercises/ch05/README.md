# 第 05 章练习

1. 推导：将 o+t*d 代入圆方程，解释为什么单位方向向量让二次项系数等于 1。判别式非负是否足以说明射线命中？
2. 代码 ch05-1：补全 starter/forward_root.hpp，选择最近非负根；处理相切、背向、圆内起点与零距离。
3. 实验：在 raycast_demo 中将圆放在墙前、墙后和射线背后，先预测再运行；解释遮挡与“遍历所有表面后取最小值”的关系。

从仓库根目录编译：

~~~bash
g++ -std=c++17 -Iexercises/ch05/starter exercises/ch05/check_root.cpp -o /tmp/ch05_root
/tmp/ch05_root
~~~

将 starter 改为 solutions 可验证参考解。评分：六类情况都正确，漏掉第二个根或把背后的交点当回波均不通过。提示单独存于 [hints.md](hints.md)；练习不参与正常演示构建。

4. 理解：为何小于 range_min 的真实命中不能写成 +inf？为何不能忽略它并继续寻找后面的墙？解释 NaN 在浮点比较中的特殊性。
5. 代码 ch05-2：补全 beam_angle.hpp，兼容整周和部分视场。90/360/720 束的首末方向不能重复；90 度视场必须覆盖两个端点。
6. 实验：以 720 束、10 Hz 运行默认 reference 场景。暂停并单步 19 次、再走第 20 步，观察 /scan 时间戳；把频率改为 20 Hz，预测间隔。理解本章 lidarPeriodTicks 辅助函数为何拒绝 7 Hz；当前模拟器已有第 06 章调度，可运行 7 Hz 并对比它与整数 tick 基线的区别。

~~~bash
g++ -std=c++17 -Iexercises/ch05/starter exercises/ch05/check_angles.cpp -o /tmp/ch05_angles
/tmp/ch05_angles
~~~

参考解仍在 solutions。合格标准是全部角度检查通过，并能解释 scan_time=0.1 与 time_increment=0 分别代表什么。

7. 代码 ch05-3：补全 noisy_range.hpp。保留 NaN/+inf；只给有效回波加误差，越出量程后标为 NaN，不夹断。
8. 推导：sigma 翻倍时方差怎样变化？为何用平均残差验收时，容差应除以样本数的平方根？
9. 实验：运行 lidar_noise_demo 与 ch05_noise_lab.yaml 的 ROS 验收。记录标准差、种子、样本数、均值与实测标准差。思考近量程边界的截尾会怎样改变结果。

~~~bash
g++ -std=c++17 -Iexercises/ch05/starter exercises/ch05/check_noise.cpp -o /tmp/ch05_noise
/tmp/ch05_noise
~~~

判定要求：普通回波、上下边界、两类特殊值均通过。不要用不同种子的两次扫描差异直接衡量 CPU/CUDA 的几何误差。

10. 代码 ch05-4：补全 beam_gap.hpp，由两条相邻射线的等腰三角形求相同距离处的端点间距，注意角度单位为 rad。
11. 实验：运行 lidar_benchmark，比较 90/360/720 束。保持世界 seed=42、起点 (-8,-8)、yaw=0、噪声关闭，记录中位数/P95 和可见回波数。不要把世界生成或 CSV 写入混进扫描计时。
12. 拓展：把独立小圆实验的方位从 2 度改为 0 度，先预测 90 束是否还会漏掉它；再改变距离与尺寸。解释为何提高束数仍不保证任意小障碍可见。

~~~bash
g++ -std=c++17 -Iexercises/ch05/starter exercises/ch05/check_gap.cpp -o /tmp/ch05_gap
/tmp/ch05_gap
~~~

评分要求：精确弦长通过手算检查；实验报告明确 CPU、构建类型、场景、预热、样本数和计时范围，不将算法内核耗时宣称为端到端消息时延。

## 推导与手算路径

- 圆心 (3,0.6)、半径1、射线从原点沿 +x 时，从二次方程配方推导两个根；用圆心垂距再核对一次。
- 墙段 [(4,-1),(4,1)] 与同一射线求交，分别写出取叉积消去 t 或 u 的过程。再计算共线投影区间 [-2,3]、[2,4]、[-4,-2] 的结果。
- 展开样本均值的平方，证明独立零均值误差的均值标准差为 sigma/sqrt(M)；说明 M 与 M−1 两种样本方差分母的关系。
- 对直径0.10 m、距离5 m、方位2°的小圆，求两条切线的角区间，再数90/360/720束扫描中落入区间的方向。
- 推导方形房间中心的墙距公式；手算方向0°、45°、90°，再与无噪声扫描核对。
