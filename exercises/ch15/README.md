# 第15章练习：从系数到路点和时间

1. 推导：对三阶导数平方做变分，解释固定中间位置、自由中间速度/加速度时，jerk和snap为何应连续。
   一段静止到静止、位移L的J=720L²/T⁵，求完整dJ/dT；与固定系数的partial J/partial T比较。
2. 代码 EXERCISE(ch15-1)：补全jerk能量矩阵Q(k,l)。k/l是升幂指数，不是导数阶数。
3. 实验：minco_demo固定四段时长和三个内部路点，比较停靠与MINCO的能量和速度/加速度采样峰值。
   改一段时间，用中心差分检查propagate输出，不只比较曲线“看起来是否平滑”。

```bash
g++ -std=c++17 -Iexercises/ch15/starter exercises/ch15/check_jerk_gram.cpp -o /tmp/ch15_gram
/tmp/ch15_gram
```

评分：导数与积分50%，完整时间梯度30%，固定同问题实验20%。小能量不等于避障或动力学可行。

代码ch15-2：正时长变量的链式梯度。用同一命令将文件换成check_time_gradient.cpp。
参考解和starter分别编译；参考解应通过，starter应失败。实验比较软约束收敛与独立残差，不能把converged等同安全。
