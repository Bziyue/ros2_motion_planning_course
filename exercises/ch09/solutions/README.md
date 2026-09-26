# 答案解释

两个已归一化的角分别位于边界两侧时，它们的差仍可接近 2pi。
只有归一化创新才能得到短转角。参考解明确约定 +pi 映射为 -pi。

v_next=v+hR(f-ba)，因此 dv_next/dyaw=hR[-(fy-bay),fx-bax]，dv_next/dba=-hR。
每次采样标准差 sigma 的速度方差增量为 h²sigma²；不是 h sigma²。
启用偏置状态后，P 中的互协方差让位置/航向校正也能改变偏置；没有外部校正时，IMU 无法独自区分运动与偏置。
