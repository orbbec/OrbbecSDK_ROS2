## 在ROS 2中启用和可视化点云

本节演示如何从相机节点启用点云数据输出并使用RViz2进行可视化。

### 启用深度点云

#### 启用深度点云的命令

要激活深度信息的点云数据流，使用以下命令：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_point_cloud:=true
```

#### 在RViz2中可视化深度点云

运行上述命令后，执行以下步骤可视化深度点云：

1. 打开RViz2。
2. 添加 `PointCloud2` 显示。
3. 选择 `/camera/depth/points` 话题进行可视化。
4. 将固定帧设置为 `camera_link` 以正确对齐数据。

- **可视化示例**

深度点云在RViz2中可能如下所示：

![深度点云可视化](../image/point_cloud/image5.jpg)

### 启用彩色点云

#### 启用彩色点云的命令

要启用彩色点云功能，输入以下命令：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_colored_point_cloud:=true
```

#### 在RViz2中可视化彩色点云

要可视化彩色点云数据：

1. 执行命令后启动RViz2。
2. 添加 `PointCloud2` 显示面板。
3. 从列表中选择 `/camera/depth_registered/points` 话题。
4. 确保固定帧设置为 `camera_link`。

- **可视化示例**

RViz2中彩色点云的结果应该如下所示：

![彩色点云可视化](../image/point_cloud/image6.jpg)
