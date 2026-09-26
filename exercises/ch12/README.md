# 第12章练习：距离、插值与单位

- 理解：画半径1 m圆的解析有符号距离 `||p||-1`，说明圆心处梯度为何不唯一。
- 推导：障碍方格的中心距为何可能高估净空？用三角不等式解释 `rho/sqrt(2)` 的面积修正。
- 代码：完成 `EXERCISE(ch12-1)` 的双线性梯度；区分对 u/v 的导数与对米制 x/y 的导数。
- 实验：运行 `esdf_demo` 比较 .2/.1/.05 m 分辨率的误差和完整距离场构建时间。是否所有查询点都单调改善？

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch12/starter \
  exercises/ch12/check_gradient.cpp -o /tmp/ch12_gradient
/tmp/ch12_gradient
```

评分：常数场20%、含xy交叉项的解析场50%、单位换算30%。参考解独立存于solutions。
进一步思考：在直墙两侧中心分别取±rho，插值的零点正确是否意味着梯度恒为单位向量？

## ROS 消息实验

启动ch12.launch.py，在另一个同domain终端运行`python3 scripts/check_ch12.py`。
固定reset后0.5s，100个seed=1214抽查点按原始地图独立枚举最近中心，容差1e-5m。
对所选格子手算中心位置和字节偏移：`16*(y*width+x)`，distance在该点起始偏移12字节。
评分：正确读取字段40%、米制中心和行步长40%、未知/无穷与色标饱和解释20%。

修改配置中gradient_stride=5再重启，只应增加箭头，不改变相同地图的distance数据。
不要把颜色通道饱和误读成距离被截断；不要将梯度箭头当成机器人控制命令。

全未知/自由与坏图测试需要独立domain，先只启动`ros2 run motion2d esdf_node`，
再运行`python3 scripts/check_ch12_empty.py`。全未知输出-inf且无箭头，坏图清空显示。
