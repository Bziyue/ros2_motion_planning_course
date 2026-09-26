# 第20章：先对齐结果，再讨论加速

1. 推导：B束、C个圆、E条边时逐束遍历的工作量O(B(C+E))。为什么增加线程不能消除启动和回传开销？
2. EXERCISE(ch20-1)：补齐 `starter/ray_circle.hpp` 的最近非负根；这是内核代数的普通C++版本，无CUDA机器也能练习。
```bash
c++ -std=c++17 -I exercises/ch20/starter exercises/ch20/check_circle.cpp -o tmp/ch20-circle
./tmp/ch20-circle
```
参考解位于solutions；评分：前向命中、相切、圆内出口、完全未命中、根在后方各20%。通过后再将同样公式写入个人CUDA练习内核，不修改正式演示以免破坏其他章节。
3. GPU实验：运行 `lidar_cuda_benchmark`，比较90/720/4096/16384束和0/16/128/512障碍；保留CPU/GPU特殊值和最大误差。不能只用独立同seed随机数宣称“相同噪声”。
4. 分别报告一次性上传、内核、回传、驻留扫描、完整ROS消息成本。用真正测到的小场景说明GPU也可能更慢。
