# ROS 2 工作空间

已建立 motion2d 和 motion2d_bringup；按进度逐步增加功能。自定义消息包尚未创建。

| 包 | 责任 | 引入时间 |
| --- | --- | --- |
| `motion2d` | 清晰的算法库、按职责划分的薄 ROS 节点 | 第 01 章起 |
| `motion2d_bringup` | 章节 launch、YAML、RViz2 配置 | 第 01 章起 |
| `motion2d_interfaces` | 标准消息不能表达的轨迹/距离场及必要 reset 请求 | 首次需要时 |

`motion2d/include/motion2d/` 与 `motion2d/src/` 按 `geometry`、`sim`、`estimation`、`mapping`、`planning`、`trajectory`、`control` 分类；ROS 转换放 `ros/`，节点放 `nodes/`。当前仅创建已经需要的目录。不要为每章复制一个包，也不要一次生成全部空目录或节点。

构建步骤见 [第 01 章](../chapters/ch01/README.md)。先加载主机 ROS，再构建并加载 install；独立算法为 C++17，ROS 节点使用 C++20。源码许可尚未确定，package.xml 使用 Proprietary 占位，不添加开放许可证。
