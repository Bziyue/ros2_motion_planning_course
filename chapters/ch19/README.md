# 第19章：在线接续与综合导航

先完成参考接续内核，再接地图与ROS导航。
ReferenceSchedule只保存活动曲线和一个未来候选；MPC的sample查询为const，不会因预读未来而提前切换当前参考。
只有实际观测时间advance才激活候选；拒绝积压的第二个候选、过去起点、不匹配p/v/a/yaw与非静止终点。
首个参考接静止锚点；后续接旧参考在未来时刻的p/v/a，不把运动机器人每次当成静止。

```bash
ros2 run motion2d reference_handover_demo tmp/ch19_handover
/usr/bin/python3 scripts/plot_ch19_handover.py
```

演示1.5s交接处p=(.550415,0)m、v=(.823975,0)m/s、a=(.439453,0)m/s²，前缀不变，后段转向(3,1)。
冻结变换同时作用于曲线和每段配置空间区域，yaw随坐标旋转；仅c0受平移。
后续map->odom校正不再输入已冻结的执行曲线。样条接续不是碰撞证书，也不是跟踪稳定性证明。
reset(anchor)会清空两条曲线并保持位置；紧急处理不宣称保持C2。时间回退由ROS边界显式reset。
