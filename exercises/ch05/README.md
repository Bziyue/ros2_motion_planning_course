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
6. 实验：以 720 束、10 Hz 运行默认 reference 场景。暂停并单步 19 次、再走第 20 步，观察 /scan 时间戳；把频率改为 20 Hz，预测间隔。试填 7 Hz，阅读拒绝原因，不要静默改成邻近频率。

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
