## 在ROS 2中将深度图像与彩色图像对齐

本节介绍如何使用ROS 2将深度图像与彩色图像对齐，以创建叠加图像。这对于需要来自不同传感器模态的同步视觉信息的应用程序特别有用。

### 对齐并查看深度和彩色图像的命令

1. **基本的深度到彩色对齐：**

   要简单地将深度图像对齐到彩色图像，使用以下命令：

   ```bash
   ros2 launch orbbec_camera gemini_330_series.launch.py depth_registration:=true
   ```

   此命令激活深度注册功能，但不打开查看器。

2. **查看深度到彩色叠加：**

   如果您希望查看深度到彩色叠加，您需要通过使用以下命令来启用查看器：

   ```bash
   ros2 launch orbbec_camera gemini_330_series.launch.py depth_registration:=true enable_d2c_viewer:=true
   ```

   这将启动具有深度到彩色注册的相机节点，并打开一个查看器以显示叠加图像。

### 在RViz2中选择主题

要在RViz2中可视化对齐的图像：

1. 在运行上述命令之一后启动RViz2。
2. 选择深度到彩色叠加图像的主题。这里显示了一个示例主题选择：

   ![深度到彩色叠加主题选择](./images/image3.png)

### 深度到彩色叠加示例

在RViz2中选择合适的主题后，您将能够看到深度到彩色叠加图像。这是它可能看起来的样子：

![深度到彩色叠加图像](./images/image4.jpg)