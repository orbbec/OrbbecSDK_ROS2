### ROS2机器人坐标系 vs 相机光学坐标系

* 视角：
  * 想象我们站在相机后面，向前看。
  * 在讨论坐标、左右红外、传感器位置等时，始终使用此视角。

![ROS2和相机坐标系统](../image/application_guide/image0.png)

* ROS2坐标系：（X: 向前，Y: 向左，Z: 向上）
* 相机光学坐标系：（X: 向右，Y: 向下，Z: 向前）
* 我们封装器话题中发布的所有数据都是直接从相机传感器获取的光学数据。
* 静态和动态TF话题发布光学坐标系和ROS坐标系，使用户能够在两个坐标系之间转换。

### ROS2 TF工具的使用

#### 查看TF树结构

可以使用以下ROS2命令来打印和可视化相机包发布的TF树：

**打印所有TF关系：**

```bash
ros2 run tf2_tools view_frames
```

这个命令会生成一个 `frames.pdf`文件，展示所有frame之间的层级关系。

![image-20251027111351870](../image/application_guide/image4.png)

**查看所有正在发布的TF信息：**

```bash
ros2 topic echo /tf_static
```

#### 使用rviz2可视化TF树

在rviz2中可以实时可视化TF树结构和坐标系的相对位置：

```bash
rviz2
```

在rviz2中：

- 添加 `TF`显示插件
- 配置固定框架（Fixed Frame）为 `camera_link`或 `camera_depth_optical_frame`
- 选择显示的TF框架树

![image-20251027140652727](../image/application_guide/image5.png)

### 相机TF计算和发布机制

#### 核心函数：`OBCameraNode::calcAndPublishStaticTransform()`

相机节点通过此函数计算和发布所有传感器之间的静态转换关系。下面是代码的详细解释：

#### 四元数初始化与坐标系变换

```cpp
tf2::Quaternion quaternion_optical, zero_rot;
zero_rot.setRPY(0.0, 0.0, 0.0);
quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
```

**说明：**

- `quaternion_optical`：定义光学坐标系到ROS标准坐标系的旋转变换（90度旋转）
- 这个旋转将相机光学坐标系（X右、Y下、Z前）转换为ROS标准坐标系（X前、Y左、Z上）

#### 获取设备信息与基准流

```cpp
auto base_stream_profile = stream_profile_[base_stream_];
auto device_info = device_->getDeviceInfo();
// 通常基准流是深度流(DEPTH)
```

**说明：**

- 选择一个基准流（通常是深度流），所有其他传感器的变换都相对于这个基准流进行计算

#### 遍历所有流并计算相对变换

```cpp
for (const auto &item : stream_profile_) {
    auto stream_index = item.first;
    auto stream_profile = item.second;

    // 获取该流相对于基准流的外参
    OBExtrinsic ex;
    ex = stream_profile->getExtrinsicTo(base_stream_profile);

    // 将旋转矩阵转换为四元数
    auto Q = rotationMatrixToQuaternion(ex.rot);

    // 应用光学坐标系变换：Q_new = quaternion_optical * Q * quaternion_optical.inverse()
    Q = quaternion_optical * Q * quaternion_optical.inverse();

    tf2::Vector3 trans(ex.trans[0], ex.trans[1], ex.trans[2]);
```

**说明：**

- `OBExtrinsic`包含了两个传感器之间的旋转矩阵(`rot`)和平移向量(`trans`)
- 通过四元数乘法将光学坐标系变换应用到每个传感器的旋转关系中
- 这个变换将相机原生的光学坐标系转换为ROS标准坐标系

#### 发布TF变换

```cpp
// 发布传感器到基准流的变换（在ROS坐标系中）
publishStaticTF(timestamp, trans, Q, frame_id_[base_stream_], frame_id_[stream_index]);

// 发布传感器到其光学frame的变换
publishStaticTF(timestamp, zero_trans, quaternion_optical, frame_id_[stream_index],
                optical_frame_id_[stream_index]);
```

**说明：**

- 第一个 `publishStaticTF`：发布从基准流到当前传感器的变换（平移+旋转）
- 第二个 `publishStaticTF`：发布从物理frame到光学frame的变换（纯旋转，无平移）
- `frame_id_[stream_index]`：物理坐标系frame名称（如 `camera_depth_frame`）
- `optical_frame_id_[stream_index]`：光学坐标系frame名称（如 `camera_depth_optical_frame`）

#### 特殊处理左右红外摄像头

```cpp
if (stream_index.first == OB_STREAM_IR_RIGHT && base_stream_.first == OB_STREAM_DEPTH) {
    trans[0] = std::abs(trans[0]);
}
```

**说明：**

- 左右红外摄像头在设备坐标系中关于中心平面对称
- 通过 `abs()`确保X轴偏移为正值，保持几何一致性

#### 发布深度到其他传感器的外参

```cpp
if (enable_stream_[DEPTH] && enable_stream_[COLOR] && enable_publish_extrinsic_) {
    OBExtrinsic ex = base_stream_profile->getExtrinsicTo(stream_profile_[COLOR]);
    auto ex_msg = obExtrinsicsToMsg(ex, "depth_to_color_extrinsics");
    depth_to_other_extrinsics_publishers_[COLOR]->publish(ex_msg);
}
```

**说明：**

- 通过TF发布变换关系
