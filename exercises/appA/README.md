# 附录A：数学回查与差分练习

- EXERCISE(appA-1)：补全point_jacobian.hpp。明确扰动是地图系tx/ty/yaw加法，不是右乘机体系SE(2)扰动。
- 编译：`c++ -std=c++17 -I/usr/include/eigen3 -I exercises/appA/starter exercises/appA/check_jacobian.cpp -o tmp/appA-check`；运行`./tmp/appA-check`。
- 实验：改变h为1e-3/1e-5/1e-7/1e-9。为什么误差不随h无限减小？参考解在solutions；h=1e-5时矩阵误差范数<1e-7为通过。
- 推导：将各列单位写出；给定2×2协方差，说明协方差非对角项的含义。为什么最小二乘应解线性方程而非显式求逆？

提示：前两列是单位阵，第三列是旋转矩阵对角度的导数乘以point。差分只验证局部导数，不能证明代价或观测模型正确。
