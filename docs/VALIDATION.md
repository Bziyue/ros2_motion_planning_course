# 第 01-03 章主机验收记录

日期：2026-09-26。环境：Ubuntu 26.04.1、ROS 2 Lyrical、GCC 15.2、Eigen 3.4、RViz2 15.2.6、XeLaTeX。完整探测记录见 [HOST_ENVIRONMENT.md](HOST_ENVIRONMENT.md)。

这是实际运行结果，不表示后续章节已经完成。测试期间用独立 ROS_DOMAIN_ID 和 LOCALHOST 发现范围，避免干扰其他 ROS 图。

## 构建与算法

在 bash 中：

~~~bash
source /opt/ros/lyrical/setup.bash
cd /home/zdp/ForCodex/ros2_motion_planning_course/ros2_ws
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DPython3_EXECUTABLE=/usr/bin/python3
colcon test --packages-select motion2d
colcon test-result --verbose
~~~

- 两个包构建成功；ROS 节点使用 Lyrical 要求的 C++20，算法库使用 C++17。
- 13 个 GoogleTest 用例通过：4 个 SE(2)、3 个时钟、6 个障碍几何与世界测试。
- colcon 汇总为 16 项，包含 3 个 CTest 包装结果；0 error、0 failure、0 skipped。
- 三章习题参考解分别通过各自 check；未完成 starter 会失败，且不进入正常课程构建。

## ROS 消息、TF 与实际窗口

加载 install 后依次启动各章；停止前一个 launch 再启动下一个。演示运行时，在另一已加载环境且 ROS_DOMAIN_ID 相同的终端运行：

~~~bash
/usr/bin/python3 ../scripts/check_ch01.py
/usr/bin/python3 ../scripts/check_ch02.py
/usr/bin/python3 ../scripts/check_ch03.py
~~~

三个脚本分别针对对应章节的默认配置，不能在同一个场景下全部运行。ch02 的脚本会暂停、单步、重置再恢复仿真。

| 场景 | 实测结果 |
| --- | --- |
| ch01 | 延迟订阅收到保留的 map 帧 Marker；默认直径 0.40 m |
| ch01 参数实验 | radius=0.35、x=1.0，实际消息直径 0.70 m、x=1.0 |
| ch01 错误输入 | radius=-1 时节点报错，返回非零 |
| ch02 | 单一时钟、单父 TF、解析旋转、暂停冻结、单步 5 ms 均通过 |
| ch02 reset | 时间归零，TF 缓存恢复后可查询 map 到 laser |
| ch03 | 8 圆、8 凸网格、20 m 边界；圆盘半径、起点和时钟正确 |
| ch03 错误输入 | 起点 x=20 越界；world 节点及 launch 返回 1，关闭本次其余进程 |

三个 RViz2 场景均在本机实际打开并目视检查；教材截图来自实际窗口。ch01 尚未广播 TF，map 帧直接显示的圆盘正常，Global Status 的缺少 TF 提示已在教材解释。ch02/ch03 完整坐标链的显示正常。

本机 RViz2 的 TF 显示插件在时间回退后退出曾崩溃。ch02 默认改用 map 和 base_link 两个 Axes 显示；重新测试 reset、恢复和退出成功。此记录仅描述本机观察，未归因到未核实的上游缺陷。

ch03 在一次约 10 分钟的演示后关闭时，RViz2 仍出现一次退出码 -11，世界与坐标节点正常退出。用 GDB 独立启动的两次退出，以及原始 launch 的三次终端 Ctrl-C 复测均正常，未取得崩溃栈，根因仍未确定。随后在 world_exited 中加了 is_shutdown 判断，避免关闭过程中再发关闭事件；这只修正重复关闭，不宣称已经根治该偶发问题。[ch03 说明](../chapters/ch03/README.md) 给出独立启动 RViz2 的命令，便于分开关闭和继续诊断。

退出事件去重后，再次连续运行三次真实伪终端 Ctrl-C 实验，三个进程均正常退出、launch 返回 0；无效起点的无界面启动仍返回 1。

## 固定种子与障碍密度

直接调用课程同一 generateWorld 实现，除下表的 seed、circle_count、polygon_count 外均为默认参数：20×20 m，机器人半径 0.20 m，margin=0.05 m，检查分辨率 0.25 m。

| seed | 圆 + 多边形 | 保守连通性检查 | 起点净空 / m | 终点净空 / m |
| --- | --- | --- | --- | --- |
| 42 | 8 + 8 | 通过 | 2.000 | 2.000 |
| 42 | 20 + 20 | 通过 | 1.224 | 0.649 |
| 43 | 8 + 8 | 通过 | 0.888 | 2.000 |
| 43 | 20 + 20 | 未通过，明确抛错 | 不返回成功世界 | 不返回成功世界 |

净空是圆心到障碍或边界的距离，尚未扣除半径和 margin。未通过保守检查不等于连续空间必然不可达，可能受栅格分辨率影响。复现实验可以逐次修改 ch03.yaml 中上述三个参数后重启；失败时不自动改 seed。

## 教材与范围

第 01-03 章实际教材 15 页，XeLaTeX 编译成功，已渲染检查分页、代码、公式和截图。练习、提示、参考解与正文源码引用对应。Python 脚本和 launch 均通过语法编译检查。

S1 结束时展示的是静态世界和第 02 章坐标演示；后续增量记录如下，整体状态见 [PROGRESS.md](PROGRESS.md)。

## 第 04 章增量：理想到点

2026-09-26：两个 ROS 包主机构建通过。累计 17 个 GoogleTest 用例（colcon 含包装项为 21 项）全部通过；新增检查整段跨圆/薄墙、相切、矩形边界、原地运动和 radius=0。

实际运行 ch04.launch.py model:=ideal；check_ch04.py 通过暂停命令、单步 5 ms、非法 frame、失败扫掠冻结位置与时间、reset 清命令与唯一时钟的检查。RViz2 实际显示从 (-8,-8) 到 (-5,-8) 的移动与轨迹，截图写入教材。扫掠代码练习参考解通过。正文扩展至 20 页，XeLaTeX 编译并渲染检查。

该次 ch04 理想模式在多次时间重置后关闭 RViz2，也出现已记录的退出码 -11；运行与通信检查正常，不能将第 03 章的退出事件去重解释成已经修复主机 RViz2 的根因。

## 第 04 章增量：速度模型

累计 21 个 GoogleTest 用例（colcon 含包装项为 25 项）通过。新增直线/yaw 边界、机体系圆弧与零角速度极限、方向保持限速、圆弧碰撞余量测试。速度积分习题参考解通过。

实际 ROS 检查通过：暂停不提前运动、10 m/s 请求限为 1 m/s、2 rad/s 限为 1 rad/s；在 yaw=0.05 rad 时机体系前进与解析方向一致。命令到期切为零速度，reset 清空旧输入，速度模式不订阅位姿输入。教材扩至 22 页并完成新增页渲染检查。

## 第 04 章增量：受力惯性

两个包构建通过，累计 28 个 GoogleTest 用例通过（colcon 含包装项为 32 项）。检查恒力/力矩解析解、零力滑行、线性/角阻尼指数衰减、饱和、质量/步长/制动距离、小阻尼极限、一步与两个半步的一致性、曲线碰撞和无效物理参数。

实际 ROS 检查分别在 mass=1 和 mass=2 kg 下通过；第二组自动惯量为 0.04 kg·m²。覆盖错误力坐标拒绝、非活动话题不订阅、暂停不推进、每轴 2 N 与 0.2 N·m 限幅、超时撤力后继续滑行、反向力制动和 reset 清空输入。

运行 inertia_demo 生成教材 CSV。1 N 作用 1 s，1/2 kg 的位移分别 0.50/0.25 m；从相同 1 m/s 制动，停止距离分别 0.50/1.00 m。dt=0.02 和 0.005 s 的结果一致到输出的六位小数；核心测试按 1e-12 容差比对解析值。图表由 LaTeX 直接读取 CSV。

mass=0、inertia_z=0、linear_drag=-1 时 launch 均返回 1。radius=0 且自动惯量时被拒绝；显式给 0.02 kg·m² 后可运行，首个恒力步正确，RViz 用 0.04 m 的可见点显示但物理半径仍是 0。

在本机 RViz2 运行受力演示，显示平移轨迹和旋转坐标轴，截图与公式、恒力练习一起写入教材。参考解通过，正文扩至 26 页并检查新增页排版。RViz2 退出阶段的主机已知问题仍保留记录。

## 第 04 章增量：连续理想参考与章节收尾

两个包构建通过，累计 31 个 GoogleTest 用例通过（colcon 含包装项为 35 项，0 error、0 failure、0 skipped）。新增检查旋转初始朝向下的四分之一圆、p/v 与 v/a 的中心差分核对、完整周期与非零初速。差分步长为 1e-4 s，误差容差为 1e-8。

实际运行 reference 模式，check_ch04.py 检查通过：不订阅三个外部命令话题；reset 恢复非零初速与解析加速度；暂停不推进；100 次单步到 0.5 s，p/v/a 与解析值一致。随后恢复自动运行到 15.71 s（首个超过 2*pi/0.4 s 的 tick），暂停后位置与同一采样时刻的解析值在 1e-10 m 容差内一致。没有把离散采样时刻误当成精确周期终点。

本章四个代码练习的参考解均通过，四个尚未完成的 starter 均分别验证会失败。教材直接引用实际实现，包含三个 RViz2 实测画面、质量/步长/制动实验 CSV 图表、解析公式、练习与验收命令。最终合订 PDF 共 28 页（含封面和目录），XeLaTeX 编译成功，已渲染检查分页、图表、公式和代码，无越界或缺字。

本次参考模式在多次 reset 后关闭 RViz2，再次出现退出码 -11；simulator 与 world 正常关闭。独立新开、仅查看暂停状态的 RViz2 未出现该退出码。显示和通信检查通过，但主机 RViz2 退出问题仍未定位，不能宣称已修复。

## 第 05 章增量：CPU 射线几何

主机两个包构建通过，累计 38 个 GoogleTest 用例通过（colcon 含 5 个包装项共 43 项，0 error、0 failure、0 skipped）。7 个新增测试覆盖圆近根/出口/相切/背向、线段端点与平行共线退化、矩形角点、遮挡顺序与刚体变换。

ros2 run motion2d raycast_demo 实测输出 2、4、2 m，与解析值一致。ch05-1 参考解通过，未完成 starter 按预期失败。教材扩展为 31 页，XeLaTeX 编译并渲染检查新增公式、几何示意图和代码；本增量尚未发布 LaserScan 或新增 RViz 场景。
