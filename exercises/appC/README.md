# 附录C：独立扰动实验与进阶设计

EXERCISE(appC-1)：补全biasIncrement，使固定T内偏置方差不随采样频率改变。
```bash
c++ -std=c++17 -I exercises/appC/starter exercises/appC/check_bias.cpp -o tmp/appC-check
./tmp/appC-check
```
参考解在solutions。评分：dt=.01解析值40%，dt增4倍增量增2倍40%，dt=0不变20%。
提示：独立增量的方差相加；K=T/dt，K乘每步方差应等于density²T。

运行 `ros2 run motion2d bias_walk_demo tmp/appC_bias`：比较100Hz与200Hz各5000次独立实验的终点方差。
不要用同一条路径上强相关的1000个点假装1000次独立试验；记录种子、密度单位、T和试验次数。
理解题：若错误地乘dt，固定T方差怎样随采样率变化？白噪声n_k与持久偏置b_k应该分别加在哪里？

选做设计（不伪装为主线已实现功能）：

- 延迟/丢包：保留采集stamp，用单独到达时间队列，解释为何不能把旧测量stamp改成now。
- 逐束畸变：写出T_ref^-1 T(t_i) p_i；只用估计轨迹去畸变，模拟真值仅用于误差评估。
- 凹多边形：找一个位于凹口中的自由点，测试点内判断、线段碰撞与雷达最近交点，不能只替换渲染顶点。
- 动态障碍：画出同位置不同时间的碰撞反例，说明静态ESDF不能表达运动障碍。
- 全局重定位：设计远离旧轨迹的复位场景，区别局部ICP、回环候选与全局搜索。
