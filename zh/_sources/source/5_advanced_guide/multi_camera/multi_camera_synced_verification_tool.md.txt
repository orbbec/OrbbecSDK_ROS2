# 多相机同步验证工具

> 本文将介绍如何验证多相机同步的同步精度。
>
> 首先，请查看如何使用 [multi_camera_synced](./multi_camera_synced.md)。

您可以在 [example](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples) 中找到示例使用代码。

## 目录结构

```plaintext
.
├── multicamera_sync/
│   ├── output/
│       ├──20250218102900/
│          ├──DevicesInfo.txt
│          ├──StreamProfileInfo.txt
│   ├── Python/
│       ├──Config.ini
├── gemini_330_series_synced_verify.launch.py
├── multi_camera_synced_verify.launch.py
```

**multicamera_sync**

* `gemini_330_series_synced_verify.launch.py`：单相机运行启动文件，为 multi_camera_synced_verify.launch.py 提供相机运行节点。
* `multi_camera_synced_verify.launch.py`：多相机同步 + save_rgbir 工具的启动文件。

**output**

> `ouput` 文件夹是相机图片输出的文件夹

在提供的输出示例中

* `20250218102900`：表示在 2025 年 2 月 18 日 10:29:00 收集的相机图像信息。
* `TotalModeFrames`：相机图像信息存储目录。
* `results-2025-02-25_163648`：分析和匹配 TotalModeFrames 图像信息后的结果。
* `DevicesInfo.txt`：相机设备基本信息（需要修改）。
* `StreamProfileInfo.txt`：相机视频流信息（无需修改）。

**Python**

* `Config.ini`：Python 分析脚本的配置文件（需要修改）。

## 运行准备

**save_rgbir 节点**

编辑 multi_camera_synced_verify.launch.py 并填写激活的相机设备，我们可以看到 save_rgbir 在最后启动。

> 重要
>
> 如果您使用 ROS 2 Foxy，需要先启动 save_rgbir 节点。

```python
# Launch description
    ld = LaunchDescription(
        [
            shared_orbbec_container,
            TimerAction(period=0.0, actions=[GroupAction([launch2_include])]),
            TimerAction(period=2.0, actions=[GroupAction([launch1_include])]),
            # The primary camera should be launched at last
            TimerAction(period=6.0, actions=[GroupAction([save_rgbir])]),
            # save_rgbir is synced verification tool
        ]
    )

    return ld
```

save_rgbir 是一个用于保存图像的工具。该工具的配置文件在 [multi_save_rgbir_params.json](../../config/tools/multisavergbir/multi_save_rgbir_params.json) 中。

```json
{
  "save_rgbir_params": {
      "time_domain": "global",
      "usb_ports": [
          "2-1",
          "2-3"
      ],
      "camera_name": [
          "camera_01",
          "camera_02"
      ]
  }
}
```

* `time_domain`：时间戳类型
* `usb_ports`："primary"、"secondary 1"、"secondary 2"、"secondary 3"，根据相机数量填写 usb_ports
* `camera_name`：相机设置的名称，例如：camera_01

**DevicesInfo.txt**

编辑 DevicesInfo.txt。只需要更改 `primarySerialNumber`、`index` 和 `serialNumber` 参数。其他参数不需要更改。请参考 [20250218102900](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples/multi_camera_synced_verification_tool/multicamera_sync/output/20250218102900) 的示例。

* `primarySerialNumber`：主相机的 SN 序列号
* `index`：相机索引
* `serialNumber`：相机的 SN 序列号

**Config.ini**

编辑 Config.ini。修改 `frameRate` 和 `tspRangeThreshold`。

## 运行此示例

**运行启动文件并保存相机图片**

* 第一个终端

```bash
ros2 launch orbbec_camera multi_camera_synced_verify.launch.py
```

* 第二个终端

```bash
ros2 service call /save_rgbir/start_capture orbbec_camera_msgs/srv/SetInt32 '{data: 100}'
```

当终端显示"over"时，图像已保存。将在工作空间下生成一个新的 multicamera_sync 文件夹。

**相机图片命名格式**

以 [color_SNCP1E5420006D_Index0_g1739874543227_f0_s1739874543327_e50_d16_.jpg](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples/multi_camera_synced_verification_tool/multicamera_sync/output/20250218102900/TotalModeFrames/SNCP1E5420006D_Index0/color_SNCP1E5420006D_Index0_g1739874543227_f0_s1739874543327_e50_d16_.jpg) 为例

* `color`：此图像是彩色流的快照。
* `SNCP1E5420006D`：相机的 SN 序列号是 CP1E5420006D。
* `Index0`：相机 0（通常指主相机）。
* `g1739874543227`："g"表示全局时间戳，意味着此帧的全局时间戳是 1739874543227。
* `f0`：此相机彩色流采集的第 0 张图片。
* `s1739874543327`："s"表示当前系统的时间戳，意味着此帧的当前系统时间戳是 1739874543327。
* `e50`：此帧的曝光为 50。
* `d16`：此帧的增益为 16。

**分析相机图像数据**

您需要将修改后的 [Python 文件夹](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples/multi_camera_synced_verification_tool//multicamera_sync/Python) 复制到新的 multi_camera_synced 子目录，并将修改后的 [DevicesInfo.txt](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples/multi_camera_synced_verification_tool/multicamera_sync/output/20250218102900/DevicesInfo.txt) 和 [StreamProfileInfo.txt](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples/multi_camera_synced_verification_tool//multicamera_sync/output/20250218102900/StreamProfileInfo.txt) 复制到与 TotalModeFrames 文件夹同级的目录。

* 最后一切准备就绪，运行 python 脚本

```bash
cd multicamera_sync/Python
python3 SyncFramesMain.py
```

操作成功后，您可以在 `results 文件夹` 中查看同步效果

## 需要更改的文件

**分析工具**

* [multi_save_rgbir_params.json](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/config/tools/multisavergbir/multi_save_rgbir_params.json)
* [DevicesInfo.txt](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples/multi_camera_synced_verification_tool//multicamera_sync/output/20250218102900/DevicesInfo.txt)
* [Config.ini](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples/multi_camera_synced_verification_tool//multicamera_sync/Python/Config.ini)

**相机配置**

[camera_params.yaml](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/config/camera_params.yaml)（相机启动参数设置）

[multi_camera_synced_verify.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples/multi_camera_synced_verification_tool//multi_camera_synced_verify.launch.py)
