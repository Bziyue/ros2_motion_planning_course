# 选学附录

- A：数学回查、SE(2)/协方差/最小二乘/凸集/强凸QP，与中央差分练习；`exercises/appA`。
- B：无滑移差速模型，左右轮速与精确圆弧积分；`exercises/appB`。
- 后续C/D按功能补充进阶传感器边界、记录回放与API文档。

附录用于主线之外的扩展，不改变第01–20章编号或默认全向受力模型。

## B 差速模型运行

先按仓库README构建/source，在仓库根目录运行：
```bash
ros2 run motion2d differential_drive_demo > textbook/data/appB_circle.csv
/usr/bin/python3 scripts/plot_appB.py
```
默认轮半径.05m、轮距.4m，左2/右6rad/s；t=pi秒处(.4,.4)m、yaw=pi/2，4pi秒一圈。
该演示复用原有SE(2)精确速度积分；这是独立无滑移运动学示例，无障碍、不发布ROS话题，未新增模拟器模式。
第18章全向力MPC不直接适用；停点/倒车模式、轮动力学与NMPC见教材扩展说明。
