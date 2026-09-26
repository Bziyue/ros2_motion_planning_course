# ch01：把半径转换成 Marker 尺寸

1. 理解题：节点和话题有什么区别？为什么半径 0.20 m 的圆盘要填 `scale.x=0.40`？
2. 代码题：完成 `starter/marker_scale.hpp` 的 `EXERCISE(ch01-1)`。
3. 实验题：让圆盘半径为 0.35 m、圆心为 `(1,0)`；对照预期值和实际消息。

从仓库根目录运行（不影响正式演示）：

~~~bash
g++ -std=c++17 -Wall -Wextra -Iexercises/ch01/starter exercises/ch01/check.cpp -o /tmp/ch01_exercise
/tmp/ch01_exercise
~~~

通过标准：两个半径得到对应直径。starter 默认未完成，会输出 FAIL。提示见 [hints.md](hints.md)，参考解见 [solutions/marker_scale.hpp](solutions/marker_scale.hpp)。把编译命令的 starter 换成 solutions 可以验证答案。
