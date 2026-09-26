# 附录D：录制、回放与可复现实验

EXERCISE(appD-1)：补全传感器话题选择，输出只能是scan和原始IMU；缺少输入时明确失败。
```bash
PYTHONPATH=exercises/appD/starter /usr/bin/python3 exercises/appD/check_topics.py
```
参考解在solutions。评分：含真值/TF/clock混合输入时筛选正确50%，仅两传感器时25%，缺IMU时明确报错25%。
提示：使用明确允许列表，不靠匹配名字中的“sensor”。

实验：执行 `python3 scripts/run_appD.py --output tmp/my_replay`，阅读summary.json和bag元数据。
为何存储时间和header.stamp可能差一个时钟周期？为什么不能改header来消除这点差异？
为什么检查序列化消息的字段值比比较重新编码后的CDR填充字节更合适？

理解：回放时停止模拟器。若两个算法都发布map→odom，应串行比较或使用完整的独立命名空间/TF树。
先固定同一传感器记录、输入里程计来源、版本、参数及ATE对齐规则，再比较结果。
现有SLAM外部对照作为接入练习，本机验收对象是课程自身SLAM回放，未声称其他系统已实测。

API练习：运行 `bash scripts/build_api.sh`，从函数文档找出坐标系/单位/前提/失败方式。
报告练习：复制`docs/EXPERIMENT_REPORT_TEMPLATE.md`，填写附录C的两频率实验；只改密度重跑，先预测方差的缩放。
逐项记录版本/命令/原始数据、失败次数及PDF本次新增页的检查结果，不用模板空项冒充结果。
