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
