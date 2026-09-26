# 第 08 章：二维扫描匹配与激光里程计

先修 01–07。当前已实现配准内核；在线里程计、局部子图和 ROS 显示在本章后续功能接入。

~~~bash
source /opt/ros/lyrical/setup.bash
cd /home/zdp/ForCodex/ros2_motion_planning_course/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 run motion2d scan_matcher_demo
colcon test --packages-select motion2d --event-handlers console_direct+
colcon test-result --verbose
~~~

演示把两面墙上的点用已知变换反投影，再只交给 ICP 两组点。
目标为 x=.12 m、y=-.08 m、yaw=.04 rad；分别报告点到点、点到线的状态、
RMSE、关联数量与迭代次数；同时比较零初值与 (.11,-.075,.038) 的邻近初值。
零初值下点到点会落入错误局部极小值，见实际 CSV；这是初值实验的一部分。
算法不接收已知答案、世界几何或 TF。

核心位于 estimation/scan_matcher.cpp。MatchConfig 控制关联门限、Huber 尺度、
最少关联数、收敛容差、最大迭代次数与残差门限，单位见头文件。
点到线法向量来自参考点的邻域 PCA，邻域不足或不呈线状时跳过。

常见错误：变换方向写反；把平移加入旋转雅可比；用到迭代上限当作收敛；
只凭残差小忽略走廊退化。最近邻搜索暂用清楚的双循环，不承诺大点云实时性。
练习、提示和参考解见 exercises/ch08。
