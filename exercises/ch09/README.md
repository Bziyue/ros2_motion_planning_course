# 第 09 章练习

1. 理解：推导 dv_next/dyaw 和 dv_next/dba；解释为什么加速度计偏置在机体系。
2. 代码 ch09-1：补完 starter/yaw_residual.hpp。两输入分别在 ±pi 两侧时，残差仍应很小。
3. 实验：运行 planar_ekf_demo，把偏置置零或把校正噪声标准差减半；同时比较误差和拒绝数。

仓库根目录运行：

```bash
g++ -std=c++17 -Iexercises/ch09/starter exercises/ch09/check_yaw.cpp -o /tmp/ch09_yaw
/tmp/ch09_yaw
```

通过标准：四种角度残差与独立手算值误差小于 1e-12，尤其覆盖跨界和多圈输入。
starter 应失败；换为 solutions 检查参考解。提示与答案分开放置。
