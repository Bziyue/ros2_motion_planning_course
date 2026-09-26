# 第 10 章练习

1. 理解：所有位姿同时左乘任意固定 SE(2)，为什么相对边残差不变？因此为何要固定第一帧？
2. 代码 ch10-1：补完 `starter/edge_residual.hpp`。先把全局差移到起点坐标，再移到测量残差坐标。
   手算旋转90度与跨 ±pi 两例，容差1e-12；参考解在solutions，不影响演示。
3. 实验：运行pose_graph_demo，改变合成相对航向偏差、回环协方差或移除回环。
   报告全程位置RMSE、首末闭合误差、残差代价和状态。代价下降为何不等于找对了地点？

从仓库根目录编译（先加载ROS工作区）：

```bash
g++ -std=c++17 -I/usr/include/eigen3 -Iros2_ws/src/motion2d/include \
  -Iexercises/ch10/starter exercises/ch10/check_edge.cpp \
  ros2_ws/src/motion2d/src/geometry/se2.cpp -o /tmp/ch10_edge
/tmp/ch10_edge
```

starter预期失败；将starter换成solutions验证参考解。
