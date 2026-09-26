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
