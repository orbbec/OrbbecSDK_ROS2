# 深度工作模式切换

Orbbec SDK ROS 2 支持深度工作模式切换。Gemini 2 和 Gemini 2 L 相机支持深度工作模式切换。

- 在启动相机之前，可以为相应的 xxx.launch.py 文件的支持配置深度工作模式（depth_work_mode）。
- Gemini 2、Gemini 2 L 和 Gemini 2 XL 相机支持深度工作模式切换。
- xxx.launch.py 的默认深度工作模式配置是相机的默认配置。如果需要修改，可以根据需要切换到相应的模式。
- 具体相机深度工作模式支持类型可以在深度模式的注释中找到。

```python
# 深度工作模式支持如下：
# Unbinned Dense Default
# Unbinned Sparse Default
# Binned Sparse Default
# Obstacle Avoidance
DeclareLaunchArgument('depth_work_mode', default_value='')
```

- 查看深度工作模式：

```bash
ros2 run orbbec_camera list_depth_work_mode_node
```

* 示例：

```bash
ros2 launch orbbec_camera gemini2L.launch.py depth_work_mode:="Unbinned Dense Default"
```
