# Net_camera

> This section describes how to use Net camera in OrbbecSDK_ROS2.Currently, only Femto_Mega and Gemini 335Le devices are supported, and other Net devices will be supported in the near future.

## Femto Mega

### Parameter Introduction

Network device settings: `enumerate_net_device` is set to true, which will automatically enumerate network devices.

If you do not want to automatically enumerate network devices,you can set `enumerate_net_device` to false, `net_device_ip` to the device's IP address, and `net_device_port` to the default value of 8090.

* `enumerate_net_device` : Enable automatically enumerate network devices.
* `net_device_ip` : Setting net device's IP address.
* `net_device_port` : Setting net device's port.Usually, you can set it to 8090.

### Single Net camera

For [femto_mega.launch.py](../../launch/femto_mega.launch.py) as an example:

#### automatically enumerate network devices:

```bash
ros2 launch orbbec_camera femto_mega.launch.py enumerate_net_device:=true
```

#### Specify IP address to start the device:

Note: `net_device_ip` needs to be changed to the IP address of the device, here it is 192.168.1.10

```bash
ros2 launch orbbec_camera femto_mega.launch.py enumerate_net_device:=false net_device_ip:=192.168.1.10 net_device_port:=8090
```

## Gemini 335Le

Network device settings: `enumerate_net_device` must be set to true, set `net_device_ip` to the IP address of the device, and set `net_device_port` to the default value of 8090.

* `enumerate_net_device` : Enable automatically enumerate network devices.
* `net_device_ip` : Setting net device's IP address.
* `net_device_port` : Setting net device's port.Usually, you can set it to 8090.

### Single Net camera

For [femto_mega.launch.py](../../launch/femto_mega.launch.py) as an example:

#### automatically enumerate network devices:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enumerate_net_device:=true
```

#### Specify IP address to start the device:

Note: `net_device_ip` needs to be changed to the IP address of the device, here it is 192.168.1.10

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enumerate_net_device:=true net_device_ip:=192.168.1.10 net_device_port:=8090
```

## Multi Net camera

For [multi_net_camera.launch.py](./multi_net_camera.launch.py) as an example:

```bash
ros2 launch orbbec_camera multi_net_camera.launch.py
```
