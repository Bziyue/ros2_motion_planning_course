# 第20章：可选CUDA激光雷达

前置：第05章CPU解析射线及C++数组。先比较正确性，再计时。CPU主线不依赖CUDA。

本机RTX5060(sm_120)、驱动595.91.07、GCC15.2；原先没有nvcc。使用官方NVIDIA CUDA13.2.1最小组件，SHA256锁定下载至ignored tmp，不改变驱动/系统编译器。
13.0.2在本机glibc2.43出现rsqrt异常规格冲突，13.2.1最小GPU内核已通过；不要靠修改系统头或忽略编译器版本继续。

```bash
# At repository root; Linux x86_64 only; keeps NVIDIA licenses with toolkit.
python3 scripts/setup_cuda_local.py
source /opt/ros/lyrical/setup.bash
cd ros2_ws
# Build CPU prerequisites first on a fresh checkout.
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3
colcon build --packages-select motion2d --symlink-install --cmake-args \
  -DMOTION2D_ENABLE_CUDA=ON \
  -DCMAKE_CUDA_COMPILER="$PWD/../tmp/cuda-13.2.1/toolkit/bin/nvcc" \
  -DCMAKE_CUDA_HOST_COMPILER=/usr/bin/g++ -DCMAKE_CUDA_ARCHITECTURES=120
source install/setup.bash
ros2 run motion2d lidar_cuda_benchmark > ../tmp/ch20_cuda.csv
```
其他GPU请设置正确的架构，现代CMake也支持本机探测`native`；不要照抄5060架构到旧显卡。
关闭GPU：`colcon build --packages-select motion2d --cmake-args -DMOTION2D_ENABLE_CUDA=OFF`。新的CPU工作区默认OFF，不查找CUDA。

核心入口：`CudaLidar(world, capacity)`、`scan(pose, config, timing)`。静态圆与全部边（含外框）打包为double SoA，一次上传，复用输出和事件。128线程/块，一束一个线程，束内串行取最小距离；没有另做障碍归约或BVH。
CPU与CUDA均返回float：近于min为NaN，超max为+inf；同一角度规范。测距噪声仍用共同CPU函数施加。
计时：一次setup（打包/分配/上传）、H2D上传、CUDA事件内核、同步后的D2H回传、驻留扫描墙钟总耗时分别报告。首次上下文初始化可能计入setup。每组预热20次、60次统计P50/P95，double且关闭FMA融合，无fast-math。

GPU测试含40个自由随机位姿、相切/共线/部分视场、near-hit NaN/no-return inf、共享噪声与容量边界。
没有GPU时不启用这个测试目标；显式启用CUDA但设备不可用时应失败，不冒充CPU成功。
