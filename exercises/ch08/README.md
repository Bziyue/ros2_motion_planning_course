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

## ch08-2：按成功观测间隔预测

完成 `starter/constant_velocity.hpp`。输入为前后两次成功配准的位姿、两者间隔与预测时长。
处理跨越 ±pi 的航向；失败帧不改变上次成功时间。仓库根目录运行：

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iros2_ws/src/motion2d/include \
  -Iexercises/ch08/starter exercises/ch08/check_prediction.cpp \
  ros2_ws/src/motion2d/src/geometry/se2.cpp -o /tmp/ch08_predict
/tmp/ch08_predict
```

通过标准：位置误差和角误差都小于 1e-12；starter 应失败，替换为 solutions 验证参考解。
理解题：为什么一次失败后不能继续用固定 0.1 s 计算割线速度？
实验题：运行本章六组回放，分别报告成功帧误差、失败率与失败时保持旧位姿的误差。
