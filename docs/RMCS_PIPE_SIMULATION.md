# 本机 PhysX 与 RMCS 管道步进

双车比较的本机控制链由正常的 `rmcs_executor` 加载五个组件：`PipeFeedback`、状态估计器、观测、`RlBridge` 和 `WheelLegController`。执行器必须支持 `clock: stepped`，并在调用组件 `before_updating()` 之前发布有效的 `update_rate`。

默认配置为 RMCS 仓库中的 `rmcs_ws/src/rmcs_rl/config/simulation/wheel-leg-pipe.yaml`。配置只加载仿真反馈和控制插件。部署前按正式导出合同更新其中的模型与元数据；尚未提供匹配的模型时启动检查会失败。实车数据源继续只订阅镜像。

`scripts/wheel_leg_policy_pipe.sh --rmcs-root <RMCS>` 会在本机 ROS 环境或指定 devcontainer 中运行：

```sh
ros2 run rmcs_executor rmcs_executor --ros-args --params-file <rmcs_rl-share>/config/simulation/wheel-leg-pipe.yaml
```

`wheeled_algo.rmcs_policy_pipe.PolicyHost` 在启动前检查组件清单与 stepped 时钟，再通过 stdin/stdout 发送和接收每行一个 JSON 对象。`ticks` 表示一组反馈连续执行的 tick 数；通用插件省略时执行 1 tick，历史 PhysX 字段适配器保留原来的 5 tick 缺省值。执行器输出的时间按 tick 推进，策略动作年龄使用同一时间源；异步推理结果到达的 tick 仍取决于实际计算耗时。

| PhysX 字段 | 管道接口 |
|---|---|
| `positions`、`velocities` | 六个 `/wheel_leg/<joint>/angle`、`velocity` |
| `quaternion_wxyz`、`gyro_body` | `/wheel_leg/imu/quaternion`、`angular_velocity` |
| `command`、`height`、`mode`、`enabled` | `/chassis/control_velocity`、`control_height`、`control_mode`、`control_enable` |
| `reset_count`、`jump_request_count` | `/chassis/reset_count`、`jump/request_count` |
| `effort_api` | 六个 `/wheel_leg/<joint>/control_torque` 的返回值 |
| `state`、`action_seq`、有效性 | `/wheel_leg/rl/state`、`action_seq`、`valid`、`healthy` |

`feedback_to_interfaces()` 与 `interfaces_to_result()` 是车型适配边界。C++ 插件只处理配置中的路径与通用类型。控制器解除使能时的 NaN 力矩在 JSON 中表现为 `null`；适配层仅在 idle 且策略未使能时将其转换为本机仿真执行器的零力矩，运行中的非有限力矩会报错。

退出时关闭 stdin，插件读到 EOF 后关闭执行器。客户端只管理自己创建的本机子进程。CPU 协议验证与控制输出对照不代表 PhysX 行为验收或实车验收。
