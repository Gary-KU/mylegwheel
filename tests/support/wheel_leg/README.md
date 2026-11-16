# 轮腿测试辅助代码

这些代码用于独立计算对照、几何兼容验证和构造控制测试姿态；实际固件不调用，也不注册到 Keil。

- `wheel_leg_control`：独立双腿 LQR/VMC 计算示例。实际固件使用任务中的共享状态平衡路径。
- `leg_trajectory`：测试用逆运动学与方形轨迹，支持构造对称/错位腿姿态。
- `leg_spd`、`leg_conv`：原 MATLAB 兼容接口。实际任务直接复用 `LegModel.jacobian` 换算速度和力矩。

实现仍通过原来的主机测试验证，公共几何与 LQR 依赖保留在 `Control/WheelLeg`。
