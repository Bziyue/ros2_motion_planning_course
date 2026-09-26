# ROS 2 工作空间（计划）

当前没有 ROS 包。以下划分在大纲获批后逐步创建：

| 包 | 责任 | 引入时间 |
| --- | --- | --- |
| `motion2d` | 清晰的算法库、按职责划分的薄 ROS 节点 | 第 01 章起 |
| `motion2d_bringup` | 章节 launch、YAML、RViz2 配置 | 第 01 章起 |
| `motion2d_interfaces` | 标准消息不能表达的轨迹/距离场及必要 reset 请求 | 首次需要时 |

`motion2d/include/motion2d/` 与 `motion2d/src/` 按 `geometry`、`sim`、`estimation`、`mapping`、`planning`、`trajectory`、`control` 分类；ROS 转换与节点放 `nodes/`。不要为每章复制一个包，也不要一次生成全部空目录或节点。

未来教材命令均从此工作空间执行 `colcon build`，并明确先加载 `/opt/ros/lyrical/setup.bash` 或 `setup.zsh`，再加载本工作空间 `install/setup.*`。本轮尚无可执行的课程构建命令。
