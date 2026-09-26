# 第 11 章练习

1. 理解：在5×5方格上手算四邻接与八邻接的g/h/f；证明八邻接距离的octile公式。
2. 代码ch11-1：补完`diagonal_step.hpp`，不能只检查对角终点，须检查两侧轴向邻格。
3. 实验：运行`astar_demo tmp/ch11_astar`。比较radius=0/.15/.30 m和四/八邻接，
   记录可达性、原始网格代价、扩展节点数和简化点数。为何直径.6 m的圆盘在.7 m门里也可能被栅格拒绝？

构建后从仓库根目录运行：

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iros2_ws/src/motion2d/include \
  -Iexercises/ch11/starter exercises/ch11/check_neighbors.cpp \
  -Lros2_ws/install/motion2d/lib -lmotion2d_core -o /tmp/ch11_neighbors
/tmp/ch11_neighbors
```

评分：五个边界判断全部正确；包含两种被阻挡的侧邻格、外边界、合法轴向与合法对角。
starter会因穿角而失败；solutions单独保存参考解。

## ch11-2：真实端点之间的路径长度

完成 `EXERCISE(ch11-2)`：`pathLength` 返回折线长度，单位 m；空路径、单点、重复点均合法。
输入为实际坐标而非栅格索引。分别解释此量、A* 的 `grid_cost` 和简化后的长度为何可能不同。

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iexercises/ch11/starter \
  exercises/ch11/check_length.cpp -o /tmp/ch11_length
/tmp/ch11_length
```

评分：空/单点 20%，3-4-5 三角形 40%，折线及重复点 40%。
实验：运行 `scripts/check_ch11.py`，保存实际观测地图和路径；对已知自由处、未知处、错误坐标系各发一个目标，
检查失败是否清空路径。RViz 目标点不应直接改变机器人状态。
