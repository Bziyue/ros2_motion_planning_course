# 第 08 章练习：二维配准

先运行 scan_matcher_demo，再阅读教材第 08 章。starter 不参与正式构建。

1. **理解**：最近邻会随位姿改变，为什么一次线性最小二乘不能解决整个 ICP？
   只有一面直墙时，点到线残差为什么不能约束沿墙平移？
2. **代码 ch08-1**：补全 starter/point_jacobian.hpp。输入是旋转后的点，
   不含平移；输出是对 tx、ty、yaw 三个增量的 2×3 雅可比。
3. **实验**：改变 scan_matcher_demo 的已知变换与初值，分别记录两种残差的
   状态、误差和迭代次数；再删除一面墙，观察退化。不要把“停止迭代”当作恢复真值。

在仓库根目录独立编译：

~~~bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch08/starter \
  exercises/ch08/check_jacobian.cpp -o /tmp/ch08_jacobian
/tmp/ch08_jacobian
~~~

评分：三个方向的中心差分误差 <1e-8；能解释旋转列的符号和单位；
能说明点到线的局部 Hessian 秩检查不能识别所有对称几何错误关联。
分级提示见 hints.md，参考解见 solutions/point_jacobian.hpp 和 solutions/README.md。
将编译命令的 starter 换为 solutions 即可检查参考解。
