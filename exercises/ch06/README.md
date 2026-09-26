# 第 06 章练习

1. 理解：机器人静止、以 2 m/s 匀速运动时，加速度计分别读什么？原地自转时，圆心处的加速度计会测到向心加速度吗？如果传感器偏离圆心，这个结论还成立吗？
2. 代码 ch06-1：补全 starter/specific_force.hpp。已知世界系加速度、yaw 和正重力大小，输出机体系三轴比力。特别检查 yaw=pi/2 时世界 +x 对应机体哪个轴。
3. 实验：运行 imu_demo，先手算五行结果；把圆轨迹半径翻倍、角频率翻倍，分别预测陀螺仪和加速度计如何变化。

从仓库根目录编译；将 starter 换为 solutions 可验证参考解：

~~~bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch06/starter \
  exercises/ch06/check_specific_force.cpp -o /tmp/ch06_force
/tmp/ch06_force
~~~

评分：静止重力符号、正负 90 度与 180 度变换均正确；能解释速度与加速度的区别。练习独立于演示构建，未完成的 starter 会明确失败。提示在 [hints.md](hints.md)。

4. 理解：若陀螺仪 sigma=0.004 rad/s，协方差单位和值是什么？为什么 orientation_covariance[0]=-1 与全零测量协方差不是一回事？
5. 代码 ch06-2：补全 covariance.hpp，以行主序写入三轴方差，非对角元素为零。
6. 实验：运行 imu_noise_demo 和 --drift。记录种子、采样间隔、试验时长、次数和 RMS；比较 sigma 翻倍前后，解释为何零均值不等于积分后零误差。

~~~bash
g++ -std=c++17 -Iexercises/ch06/starter \
  exercises/ch06/check_covariance.cpp -o /tmp/ch06_covariance
/tmp/ch06_covariance
~~~

评分：协方差九项及零噪声情形正确；区分“未提供姿态”和“未知协方差”；配对漂移实验 RMS 比值为 2。可选进阶题：额外注入恒定加速度偏置，推导位置误差 b*T²/2，并与白噪声比较。随机游走偏置若以后实现，应是有状态模型，不能每帧重新抽一个“偏置”。
