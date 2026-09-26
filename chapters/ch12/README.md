# 第12章：二维距离场、插值与梯度

先修07、11章。已实现EDT内核、离散有符号场、双线性梯度及保守净空下界，以及观测地图的 ROS 距离点云与梯度可视化。

```bash
ros2 run motion2d esdf_demo tmp/ch12_esdf
/usr/bin/python3 scripts/plot_ch12_esdf.py
```

输入是未膨胀占据图，默认只有已观测且编码<=35的格子自由，未知阻塞。
两遍一维抛物线下包络输出精确的格子中心平方距离，单位cell²；开方并乘分辨率才是m。
分别到阻塞/自由中心计算距离，自由为正，阻塞为负，外部补一圈阻塞格。
整个地图阻塞时内部值为-inf，查询返回空，不能伪造零梯度可用空间。

Esdf2D::sample支持最外围格子中心围成的闭矩形内查询；不向边界外延伸或截断坐标。
返回distance和gradient是同一个双线性插值函数的值与解析梯度，梯度单位m/m，在patch边界可能跳变。
相邻自由/阻塞中心为±rho时，跨边界梯度可达2，不是连续精确SDF。

`clearance_lower_bound`对阻塞方格面积及地图外部给保守下界：
仅当四个插值角均自由时，从插值中心距离减去加权角点距离（插值误差界）及方格半对角线rho/sqrt(2)。
其他情况为0，0表示不能保证正净空。必须严格大于radius+margin才满足本课程“不接触”约定。
这是单点的几何证书，不覆盖未采样的曲线区间；gradient不是此下界的导数。
已经膨胀的规划掩码不传进本章默认ESDF，以免重复扣除半径。

演示在8×8 m给定地图中按中心采样一个半径1 m圆，.2/.1/.05 m三种分辨率。
在同一查询晶格、0.3<=||p||<=1.6 m环带对比圆的解析有符号距离；方格轮廓和圆并不相同。
构建计时含分类、padding及两遍EDT，先预热5次再100次，报告中位数/P95；不含ROS消息序列化。
单元测试包括50张独立暴力对照、直墙/符号/单位、梯度差分、1000点方格精确距离下界及全阻塞/未知/越界。
练习ch12-1补完插值梯度。常见错误是未开方、漏乘分辨率、将中心距直接作净空、混用梯度与下界。


## ROS 实验与可视化

```bash
ros2 launch motion2d_bringup ch12.launch.py
ros2 topic echo /esdf/status
# 同一domain的另一终端：
/usr/bin/python3 scripts/check_ch12.py
/usr/bin/python3 scripts/plot_ch12_observed.py
```

`esdf_node`只订阅原始`/map`。`/esdf/cloud`保持原地图行列结构，每点16字节，x/y/z/distance四个FLOAT32，
距离单位m，z=0；全阻塞内部为-inf，is_dense=false，状态no_finite_field。输入无效时清空云和箭头。
消息时间与地图一致；独立快照重建，reset不保留旧场。地图/规划共用外部消息几何验证。
`/esdf/gradients`每gradient_stride格显示箭头，方向朝距离增加；长度缩放/截断仅供显示。
RViz使用Intensity、distance通道，固定颜色范围[-.5,2]m；修改分辨率后相应修改Size(m)。

实际验收：固定0.5s，独立100点最近中心枚举，误差<1e-5m，时间/字段/箭头方向/输入隔离/reset均检查。
`scripts/check_ch12_empty.py`需在独立domain中只启动esdf_node，用于全未知、全自由和坏地图验收。
本次主机GUI未验收；书中彩图来自实际消息绘制，不标作RViz截图。
