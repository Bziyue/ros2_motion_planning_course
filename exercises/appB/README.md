# 附录B：差速轮速与轨迹可行性

EXERCISE(appB-1)：补全starter/wheels.hpp，输出顺序为左、右轮，正角速度为逆时针。

```bash
c++ -std=c++17 -I exercises/appB/starter exercises/appB/check_wheels.cpp -o tmp/appB-check
./tmp/appB-check
```

直行、倒车、原地转动、圆弧各25%，所有轮速绝对误差<1e-12。参考解在solutions；不修改正式演示。
提示：相加得到前进速度，相减得到角速度；轮距不是圆盘半径。

推导：写出侧向速度必须为零的约束。为什么全向轨迹给出的独立yaw通常不能直接用于差速车？
实验：运行differential_drive_demo，把左右轮速都乘2；圆半径是否变化，转一圈的时间如何变化？
扩展：先在非零前进速度处由p/v/a导出yaw/omega，再说明停点或倒车时必须额外选择运动模式。
本附录只实现无滑移运动学，轮力矩/NMPC作为进一步设计，不宣称已接入第18章控制器。
