# 第13章练习：整块安全，而不只是顶点安全

1. 理解：在一块凸四边形内部放一个小障碍，说明四个顶点和四条边均自由为何还不够。
2. 代码：完成EXERCISE(ch13-1)的CCW边到单位外法向半空间转换。输入边非零，坐标单位m。
3. 实验：corridor_demo固定地图、起终点及半径，比较局部矩形与安全凸包合并后的区域数、顶点数、交叠面积。

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch13/starter \
  exercises/ch13/check_halfspace.cpp -o /tmp/ch13_halfspace
/tmp/ch13_halfspace
```

评分：方向40%、单位法向30%、偏移30%。提示、参考解分别存放。
进一步改变max_extension，观察区域数量和形状，不根据一张图推断全局最优体积。
相邻走廊只共边或共点时为什么不适合分配有运动余量的连接点？
