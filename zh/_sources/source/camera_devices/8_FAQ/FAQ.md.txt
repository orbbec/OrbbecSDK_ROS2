# 常见问题

### 意外崩溃

如果相机节点意外崩溃，它将在当前运行目录中生成崩溃日志：`Log/camera_crash_stack_trace_xx.log`。请将此日志发送给支持团队或提交到GitHub issue以获得进一步帮助。

### 多相机无数据流

**电源供应不足**：

- 确保每个相机连接到单独的集线器。
- 使用有源集线器为每个相机提供足够的电力。

**高分辨率**：

- 尝试降低分辨率以解决数据流问题。

**增加usbfs_memory_mb值**：

- 通过运行以下命令将 `usbfs_memory_mb` 值增加到128MB（这是参考值，可根据系统需求调整）：

```
    echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

- 要使此更改永久生效，请查看[此链接](https://github.com/OpenKinect/libfreenect2/issues/807)。

### 其他故障排除

- 如果遇到其他问题，将 `log_level` 参数设置为 `debug`。这将在运行目录中生成SDK日志文件：`Log/OrbbecSDK.log.txt`。请将此文件提供给支持团队以获得进一步帮助。
- 如果需要固件日志，将`log_level` 参数设置为 `debug`的同时，将 `enable_heartbeat` 设置为 `true` 以激活此功能。
- 若将`log_level` 参数设置为 `debug`的同时，又不想终端刷新太多日志，可以在`launch`中将`output="screen"`改为`output="log"`，日志会被保存在`~/.ros/log`目录下。

### 为什么有这么多启动文件？

- 不同的相机具有不同的默认分辨率和图像格式。
- 为简化使用，每个相机都有自己的启动文件。

### 多相机连接时如何指定启动某一个相机

如果启动文件未显式指定要使用的设备，在同时连接多台相机时，驱动会默认连接到其中一个（默认设备）。

可以先通过以下命令查看设备序列号：

```bash
ros2 run orbbec_camera list_devices_node
```

然后在启动时显式指定序列号，例如：

```bash
ros2 launch orbbec_camera femto_bolt.launch.py serial_number:=CL8H741005J
```

### 多相机启动或切换流时为什么需要设置延迟？

多相机系统对带宽和设备初始化时序要求较高。如果在同一时间启动或切换多个相机流，可能会引发带宽瞬时拥塞，进而导致设备初始化失败、流启动异常或丢帧等问题。为确保系统稳定性，建议注意以下几点：

- **多相机启动阶段**

  在启动多个相机时，建议在每个相机启动之间增加适当的延迟（例如 **2s**），以避免瞬时带宽过载或底层设备初始化冲突。

- **流开关与模式切换阶段**

  在调用开关流相关服务（如 `set_streams_enable`、`toggle_depth`、`toggle_color`）时，不建议同时触发多个接口调用，应在各操作之间设置合理的时间间隔（例如 **20 ms**），以保证流状态切换的可靠性。

遵循上述时序控制原则，有助于提升多相机系统在启动和运行过程中的稳定性，减少异常和不可预期行为的发生。
### femto bolt 深度流无数据

该模组运行时依赖 OpenGL 库，若系统未安装或驱动不完整，将导致深度流无数据。请先安装 OpenGL 相关库（以 Ubuntu 为例）：

```bash
  sudo apt update && sudo apt install -y mesa-utils libgl1-mesa-glx libglu1-mesa
```

  安装后可通过以下命令检查 OpenGL 是否可用：

```bash
  glxinfo -B
```

### 图像未达到预设帧率

首先需要确认图像是否确实未达到预设帧率。在 ROS 2 中可通过多种方式查看帧率，例如：

* `ros2 topic hz`
* `rqt`
* 自定义工具（如本 ROS 包提供的 `benchmark` 工具）

需要注意的是，不同工具的统计方式和 QoS 配置不同，因此得到的帧率结果可能存在差异。当发现帧率低于预期时，请优先排查是否为帧率统计工具本身导致的误差。

若确认图像帧率确实未达到预设值，可尝试以下排查步骤：

1. **降低分辨率或帧率**，判断是否由于 USB / 网络带宽受限导致帧率下降；
2. **确认相机固件版本及 ROS 包版本是否为最新**，旧版本可能存在性能或兼容性问题。

若以上方法仍无法解决问题，请联系我司 **FAE**，或在 **GitHub Issue** 中提交问题以获得进一步支持。


### 软触发模式相关问题

* **信号触发时各传感器未同时出流**
  请开启帧汇聚功能，将参数`frame_aggregate_mode`设置为`full_frame`，以保证多传感器数据在同一次触发下同步输出。

* **自动触发模式下无法达到预设帧率**
  设置 `software_trigger_period` 时，需要综合考虑实际开流帧率与曝光时间。例如，当 `color_fps` 设置为 10 FPS 时，`software_trigger_period` 不能低于以下计算值：

  ```
  software_trigger_period ≥ 1000000 / fps × N + 2 × expo
  ```

  其中：

  * `fps`：传感器帧率
  * `N`：单次触发采集的帧数量
  * `expo`：曝光时间
  * `单位`：µs

  若 `software_trigger_period` 设置过小，将导致触发频率受限，从而丢帧。


