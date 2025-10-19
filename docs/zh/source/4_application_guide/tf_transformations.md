### 从坐标系A到坐标系B的TF变换：

在Orbbec相机中，原点（0,0,0）取自camera_link位置

我们的封装器提供从每个传感器坐标到相机基座（camera_link）之间的静态TF变换

此外，它还提供从每个传感器ROS坐标到其对应光学坐标的TF变换。

Gemini335模块的RGB传感器和右红外传感器静态TF变换示例，如rviz2中所示：

```bash
ros2 launch orbbec_description view_model.launch.py model:=gemini_335_336.urdf.xacro
```

![rviz2中的模块](../image/application_guide/image2.png)
