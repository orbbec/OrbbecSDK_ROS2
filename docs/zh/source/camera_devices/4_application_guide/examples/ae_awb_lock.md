# AE/AWB 锁定测试

本示例提供一个测试用 ROS 2 action，通过相机驱动服务和彩色帧元数据验证自动曝光与自动白平衡收敛后切换到手动值的流程。[查看示例源码](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples/ae_awb_lock)。

## 运行

先启动相机驱动，再在同一命名空间启动测试节点：

```bash
ros2 run orbbec_camera ae_awb_lock_test_node --ros-args -r __ns:=/camera
```

发送目标并打印反馈：

```bash
ros2 action send_goal \
  /camera/run_ae_awb_lock_test \
  orbbec_camera_msgs/action/RunAeAwbLockTest \
  "{timeout_ms: 10000}" \
  --feedback
```

## 流程与结果

测试节点订阅相对话题 `color/metadata`，启用自动曝光和自动白平衡，等待 SDK 状态变为 `1`。随后，它从最新彩色帧元数据中读取曝光、彩色增益和色温，并通过结构化属性服务读取 AWB 的 R/B/G 增益。关闭自动控制后，按以下顺序写回捕获值：

1. 彩色曝光
2. 彩色增益
3. AWB R/B/G 增益
4. 色温

最后读取的 AWB 增益必须与捕获值完全一致。设备可能量化其他控制值，因此其他读回差异只会作为警告报告。失败或取消时，示例会尝试恢复自动曝光和自动白平衡。

每个反馈阶段都包含从相机服务新读取的状态值。只有必需服务全部可用后才会发布 `waiting_for_services` 反馈，因为此前无法读取状态。

运行时必须开启彩色流；使用 `/camera` 命名空间时，`/camera/color/metadata` 必须可用。如果超时前没有元数据，或元数据缺少 `exposure`、`gain`、`white_balance`，测试会失败，不会写入默认值。目标超时覆盖服务发现、服务调用、收敛、捕获、写回和验证；失败后的自动控制恢复使用独立的尽力而为超时。
