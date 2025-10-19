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
- 如果需要固件日志，将 `enable_heartbeat` 设置为 `true` 以激活此功能。

### 为什么有这么多启动文件？

- 不同的相机具有不同的默认分辨率和图像格式。
- 为简化使用，每个相机都有自己的启动文件。
