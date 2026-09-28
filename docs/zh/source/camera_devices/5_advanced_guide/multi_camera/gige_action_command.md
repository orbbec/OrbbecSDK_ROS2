# GigE Action Command

本示例通过 Group Actions 同步模式启动两台 Gemini 335Le 相机，并启动一个主机侧 Action Command 发送节点。一个 GVCP Action Command 可以触发多台相机，因此发送节点只在顶层启动一次。[查看示例源码](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples/gige_action_command)。

## 运行条件

- Gemini 335Le 固件 1.8.24 或更高版本
- Orbbec SDK 2.10.2 或更高版本
- 两台相机与主机位于同一网络

运行前，修改 `multi_gige_action_command.launch.py` 中两处 `net_device_ip`，使其与相机 IP 一致。

## 启动相机和发送节点

```bash
ros2 launch orbbec_camera multi_gige_action_command.launch.py
```

启动文件为每台相机提供配置服务，并启动一个发送服务：

```text
/camera_01/get_action_config
/camera_01/set_action_config
/camera_02/get_action_config
/camera_02/set_action_config
/gige_action_command_node/send_action_command
```

## 配置相机

将两台相机的 Action Signal block 0 配置为相同的 key 和 mask：

```bash
ros2 service call /camera_01/set_action_config \
  orbbec_camera_msgs/srv/SetActionConfig \
  "{device_key: 1, selector: 0, group_key: 1, group_mask: 1}"

ros2 service call /camera_02/set_action_config \
  orbbec_camera_msgs/srv/SetActionConfig \
  "{device_key: 1, selector: 0, group_key: 1, group_mask: 1}"
```

需要时可读取配置：

```bash
ros2 service call /camera_01/get_action_config \
  orbbec_camera_msgs/srv/GetActionConfig \
  "{selector: 0}"
```

## 触发相机组

请求中的 device key、group key 和 group mask 与相机配置匹配时，相机会收到触发命令。发送服务支持以下三种模式。

### 立即触发

将 `trigger_mode` 设为 `0`，延迟和指定时间均设为零：

```bash
ros2 service call /gige_action_command_node/send_action_command \
  orbbec_camera_msgs/srv/SendActionCommand \
  "{device_key: 1, group_key: 1, group_mask: 1, broadcast_ip: '255.255.255.255', trigger_mode: 0, delay_ms: 0, scheduled_time: 0}"
```

### 相对延迟触发

将 `trigger_mode` 设为 `1`，并提供正数毫秒延迟。节点读取主机系统时钟，加上延迟后转换为 SDK 所需的绝对 GVCP/PTP 时间戳。以下示例延迟一秒：

```bash
ros2 service call /gige_action_command_node/send_action_command \
  orbbec_camera_msgs/srv/SendActionCommand \
  "{device_key: 1, group_key: 1, group_mask: 1, broadcast_ip: '255.255.255.255', trigger_mode: 1, delay_ms: 1000, scheduled_time: 0}"
```

主机的 `CLOCK_REALTIME` 必须与相机处于同一 PTP 时间域，例如使用 `phc2sys` 同步。启动文件会启用相机的 PTP 同步，但不会配置主机 PTP 服务。延迟应足够长，确保命令能在目标时间前到达相机。

### 绝对 PTP 时间触发

将 `trigger_mode` 设为 `2`，`delay_ms` 设为零，并提供未来的编码 PTP 时间戳。高 32 位为秒，低 32 位为纳秒：

```bash
ros2 service call /gige_action_command_node/send_action_command \
  orbbec_camera_msgs/srv/SendActionCommand \
  "{device_key: 1, group_key: 1, group_mask: 1, broadcast_ip: '255.255.255.255', trigger_mode: 2, delay_ms: 0, scheduled_time: <PTP_TIMESTAMP>}"
```

响应中的 `encoded_scheduled_time` 是发送给 SDK 的准确 64 位数值。延迟触发时，该值由节点计算。`success: true` 表示主机已发出 GVCP 命令；协议不会返回设备确认。
