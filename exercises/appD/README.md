# 附录D：固定输入，重放与比较

1. **EXERCISE(appD-1)**：补全 `starter/topics.py` 中传感器话题选择函数，按scan、原始IMU顺序输出，缺少输入时抛出ValueError。

```bash
PYTHONPATH=exercises/appD/starter /usr/bin/python3 exercises/appD/check_topics.py
```

混合真值/TF/clock列表筛选50%，仅两传感器25%，缺IMU报错25%。参考解在 `solutions`。

2. 手算采集间隔.005s、角速度.2rad/s，在播放倍率.5与2下的墙钟间隔和角度积分增量。区分header.stamp、存储时间与本机等待时间。
3. 执行 `python3 scripts/run_appD.py --output tmp/my_replay`，使用新的输出目录。阅读summary.json、bag元数据与录制/播放日志，检查文件条数、处理终点和观测地图。
4. 解释录制器最近收到的clock为何可能与传感器stamp差一个更新间隔。设计一个保留两种时刻的表格，检查回放是否保持header。
5. 从 `scripts/run_appD.py` 的digest函数追踪字段展开、排序编码与散列。分别改变量程、frame或协方差中的一个值，预测摘要变化；说明为什么先解码字段再比较。
6. 画出map→odom→base_link及laser/imu_link两条外参，标出发布者。尝试更换外参时，先手算一个点的变换再观察RViz。
7. 设计外部SLAM对照：固定bag，列出扫描、外参、里程计与IMU信息来源；串行运行以保持TF边的发布者唯一，按第19章相同规则评价。
8. 运行 `bash scripts/build_api.sh`，从偏置函数的API追到实现与调用点。复制 `docs/EXPERIMENT_REPORT_TEMPLATE.md`，填写附录C实验的问题、预测、输入、命令、数据和结论。

回放先核对输入完整性，再评价算法输出。保持一组实验只改变一个因素，便于解释结果。
