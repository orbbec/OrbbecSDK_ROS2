## Multi-camera synchronous image storage and script analysis

This section describes how to use multiple Orbbec cameras to store images synchronously in a ROS 2 environment and use scripts to parse the data.

### Connection preparation

Please follow the instructions to connect multiple Orbbec cameras and Multi-Camera Sync Hub Pro.

* For detailed guidance on Multi-Camera Sync Hub Pro, you can refer to [Set up cameras for external synchronization](https://www.orbbec.com/docs/set-up-cameras-for-external-synchronization_v1-2/).
* Example of connecting multiple cameras to Multi-Camera Sync Hub Pro.

![Schematic diagram of multi-machine synchronizer connection](./images/image7.jpg)

### Multi-camera synchronous image storage

#### Check the USB port of your device

```bash
ros2 run orbbec_camera list_devices_node
```

![Check the USB port](./images/image8.jpg)

#### **Parameter configuration**

##### multi_camera_synced.launch.py

1. `usb_port` is set to the USB port of the device.
2. `device_num` is set to 2, indicating that two cameras are started.
3. `sync_mode` is set to primary, indicating that the camera device is set to host mode.
4. The slave sets `trigger_out_enabled` to false.

![Launch Configuration](./images/image9.jpg)

##### multi_save_rgbir_params.json

Configure USB ports and names for multiple devices.

![Json Configuration](./images/image10.jpg)

##### camera_params.yaml

Enable the left IR camera.

`enable_left_ir: true`

##### start_multi_sync.sh

`{data: 100}` means that each stream of each camera stores 100 images.

![Shell Configuration](./images/image11.jpg)

#### Running

Start multiple Orbbec cameras (select 1).

* On terminal 1

```bash
cd  ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools
sudo chmod +x start_multi_sync.sh
bash start_multi_sync.sh
```

![Startup Script](./images/image12.jpg)

Save the image synchronously (select 4).

* On terminal 2

```bash
cd  ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools
bash start_multi_sync.sh
```

### Script parsing

`multicamera_sync` - save the image and analysis script storage directory.

* `ouput` - save the image directory
* `Python` - analysis script directory

Note: `multicamera_sync` needs to be copied to the current workspace directory (same level as the `install` directory).

#### Configuration Files

* `Config.ini` configures the camera's actual frame rate and maximum error timestamp threshold.
* `DevicesInfo.txt` configures the number of cameras and the camera's pid, vid, SN number and other parameters.
* `StreamProfileInfo.txt` configures the camera resolution, frame rate, and encoding format.

Note: `DevicesInfo.txt` and `StreamProfileInfo.txt` are files required by the parsing script and need to be copied to the same directory as the `TotalModeFrames` folder (The `TotalModeFrames` directory is created by the image saving script and is used to store image data collected synchronously by multiple cameras).

#### Running

Run the parsing script to parse the images stored synchronously by multiple cameras.

```bash
cd  ~/ros2_ws/multicamera_sync/Python
python3 SyncFramesMain.py
```

#### Output

* `matchFrames`: Incomplete dataset of the same camera group, camera image information is missing.
* `notMatchFrames`: Complete dataset of the same camera group.
* `abnormal`: The same set of complete data sets of cameras, and the camera synchronization time exceeds the parameter `tspRangeThreshold` set in `Config.ini`.
