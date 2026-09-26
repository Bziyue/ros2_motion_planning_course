# ch03：线段距离与随机世界

1. 理解题：一个圆心点不碰障碍，为什么圆盘仍可能碰撞？相切算不算碰撞？
2. 代码题：实现 starter 中的 `EXERCISE(ch03-1)`，正确处理投影落在线段内部、端点之外以及退化线段。
3. 实验题：固定 seed=42 运行两次；然后改为 43，再把障碍数量各增加到 20。记录是否可达、起终点净空和几何变化。不可达检查失败也是应记录的结果。

从仓库根目录：

~~~bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch03/starter \
  exercises/ch03/check.cpp -o /tmp/ch03_exercise
/tmp/ch03_exercise
~~~

参考解使用 solutions 替代 starter。正式演示使用已经完成的库函数，练习未完成不会影响运行。提示见 [hints.md](hints.md)，答案见 [solutions/segment_distance.hpp](solutions/segment_distance.hpp)。
