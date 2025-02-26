# Multi_camera_synced_verification_tool

> This article will introduce how to verify the synchronization accuracy of multi-camera synchronization.
>
> First, please see how to use [multi_camera_synced](../multi_camera_synced/README.MD).

## Directory Structure

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

### multicamera_sync

* `gemini_330_series_synced_verify.launch.py` : Single camera runs launch, which provides the camera running node for multi_camera_synced_verify.launch.py.
* `multi_camera_synced_verify.launch.py` : Multi camer synced + launch of save_rgbir tool.

#### output

> The `ouput` folder is the folder where the camera pictures are output

In the output example provided

* `20250218102900` : Represents the camera image information collected at 10:29:00 on February 18, 2025.
* `TotalModeFrames` : Camera image information storage directory.
* `results-2025-02-25_163648` : The result after analyzing and matching the image information of TotalModeFrames.
* `DevicesInfo.txt` : Camera equipment basic information (need to be modified).
* `StreamProfileInfo.txt` : Camera video stream information (no need to modify).

#### Python

* `Config.ini` : Configuration file for Python analysis script (need to be modified).

## Preparation for operation

### save_rgbir node

Edit multi_camera_synced_verify.launch.py and fill in the activated camera device,we can find that save_rgbir is started at the end.

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

save_rgbir is a tool for saving images. The configuration file of this tool is in [multi_save_rgbir_params.json](../../config/tools/multisavergbir/multi_save_rgbir_params.json).

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

* `time_domain` : Timestamp Type
* `usb_ports` : "primary", "secondary 1", "secondary 2", "secondary 3", fill in as many usb_ports as there are cameras
* `camera_name` : The name of the camera setting, for example: camera_01

### DevicesInfo.txt

Edit DevicesInfo.txt. Only the `primarySerialNumber`, `index` and `serialNumber` parameters need to be changed. Other parameters do not need to be changed.Refer to the example of [20250218102900](./multicamera_sync/output/20250218102900).

* `primarySerialNumber` : The SN serial number of the primary camera
* `index` : Camera index
* `serialNumber` : The SN serial number of the camera

### Config.ini

Edit Config.ini.Modify `frameRate` and `tspRangeThreshold`.

## Run this example

### Run launch and save camera pictures

* Frst terminal

```bash
ros2 launch orbbec_camera multi_camera_synced_verify.launch.py
```

* Second terminal

```bash
ros2 service call /save_rgbir/start_capture orbbec_camera_msgs/srv/SetInt32 '{data: 100}'
```

When the terminal displays "over", the image is saved.A new multicamera_sync folder will be generated under the workspace.

#### Camera pictures naming format

Take [color_SNCP1E5420006D_Index0_g1739874543227_f0_s1739874543327_e50_d16_.jpg](./multicamera_sync/output/20250218102900/TotalModeFrames/SNCP1E5420006D_Index0/color_SNCP1E5420006D_Index0_g1739874543227_f0_s1739874543327_e50_d16_.jpg) as an example

* `color` : This image is a snapshot of the color stream.
* `SNCP1E5420006D` : The camera's SN serial number is CP1E5420006D.
* `Index0` : Camera 0 (usually referred to primary camera).
* `g1739874543227` : “g” represents the global timestamp, which means the global timestamp of this frame is 1739874543227.
* `f0` : The 0th picture of the color stream acquisition of this camera.
* `s1739874543327` : "s" represents the timestamp of the current system, which means the current system timestamp of this frame is 1739874543327.
* `e50` : The exposure of this frame is 50.
* `d16` : The gain of this frame is 16.

### Analyzing camera image data

You need to copy the modified [Pyhotn folder](./multicamera_sync/Python) to the new multi_camera_synced subdirectory, and copy the modified [DevicesInfo.txt ](./multicamera_sync/output/20250218102900/DevicesInfo.txt)and [StreamProfileInfo.txt](./multicamera_sync/output/20250218102900/StreamProfileInfo.txt) to the same level directory as the TotalModeFrames folder.

* Finally everything is ready, run the python script

```bash
cd multicamera_sync/Python
python3 SyncFramesMain.py
```

After the operation is successful, you can view the synchronization effect in the r `esults folder`

## Files that need to be changed

### Analysis tools

* [multi_save_rgbir_params.json](../../config/tools/multisavergbir/multi_save_rgbir_params.json)
* [DevicesInfo.txt ](./multicamera_sync/output/20250218102900/DevicesInfo.txt)
* [Config.ini](./multicamera_sync/Python/Config.ini)

### Camera Configuration

[camera_params.yaml](../../config/camera_params.yaml)(Camera startup parameter settings)

[multi_camera_synced_verify.launch.py](./multi_camera_synced_verify.launch.py)
