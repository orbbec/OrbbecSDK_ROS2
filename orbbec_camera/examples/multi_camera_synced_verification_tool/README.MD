# multi_camera_synced_verification_tool

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

#### output

> The ouput folder is the folder where the camera pictures are output

In the output example provided

* `20250218102900` : Represents the camera image information collected at 10:29:00 on February 18, 2025.
* `TotalModeFrames` : Camera image information storage directory.
* `results-2025-02-25_163648` : The result after analyzing and matching the image information of TotalModeFrames.
* `DevicesInfo.txt` : Camera equipment basic information (need to be modified).
* `StreamProfileInfo.txt` : Camera video stream information (no need to modify).

#### Python

Config.ini : Configuration file for Python analysis script (need to be modified).

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

### Analyzing camera image data

You need to copy the modified [Pyhotn folder](./Python) to the new multi_camera_synced subdirectory, and copy the modified [DevicesInfo.txt ](./multicamera_sync/output/20250218102900/DevicesInfo.txt)and [StreamProfileInfo.txt](./multicamera_sync/output/20250218102900/StreamProfileInfo.txt) to the same level directory as the TotalModeFrames folder.

* Finally everything is ready, run the python script

```bash
cd multicamera_sync/Python
python3 SyncFramesMain.py
```

After the operation is successful, you can view the synchronization effect in the r`esults folder`
