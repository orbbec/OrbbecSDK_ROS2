# Net_camera

> This section describes how to use Net camera in OrbbecSDK_ROS2.Currently, only Femto_Mega, Gemini 335Le and Gemini 435Le devices are supported, and other Net devices will be supported in the near future.

You can find example usage code in the [example](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples).

## Femto Mega & Gemini 435Le

**Parameter Introduction**

Network device settings: `enumerate_net_device` is set to true, which will automatically enumerate network devices.

If you do not want to automatically enumerate network devices,you can set `enumerate_net_device` to false, `net_device_ip` to the device's IP address, and `net_device_port` to the default value of 8090.

* `enumerate_net_device` : Enable automatically enumerate network devices.
* `net_device_ip` : Setting net device's IP address.
* `net_device_port` : Setting net device's port.Usually, you can set it to 8090.

**Single Net camera**

> If you need to run Gemini 435Le, you only need to replace [femto_mega.launch.py ](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/launch/femto_mega.launch.py)in the run command with [gemini435_le.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/launch/gemini435_le.launch.py)

For [femto_mega.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/launch/femto_mega.launch.py) as an example:

- **automatically enumerate network devices:**

```bash
ros2 launch orbbec_camera femto_mega.launch.py enumerate_net_device:=true
```

- **Specify IP address to start the device:**

Note: `net_device_ip` needs to be changed to the IP address of the device, here it is 192.168.1.10

```bash
ros2 launch orbbec_camera femto_mega.launch.py enumerate_net_device:=false net_device_ip:=192.168.1.10 net_device_port:=8090
```

## Gemini 335Le

Network device settings: `enumerate_net_device` must be set to true, set `net_device_ip` to the IP address of the device, and set `net_device_port` to the default value of 8090.

* `enumerate_net_device` : Enable automatically enumerate network devices.
* `net_device_ip` : Setting net device's IP address.
* `net_device_port` : Setting net device's port.Usually, you can set it to 8090.

**Single Net camera**

For [gemini_330_series.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/launch/gemini_330_series.launch.py) as an example:

- **automatically enumerate network devices:**

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enumerate_net_device:=true
```

- **Specify IP address to start the device:**

Note: `net_device_ip` needs to be changed to the IP address of the device, here it is 192.168.1.10

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enumerate_net_device:=true net_device_ip:=192.168.1.10 net_device_port:=8090
```

## set_device_ip Utility

The **`set_device_ip`** executable allows you to configure the IP settings of a network camera directly from ROS 2, including switching between DHCP and static IP, and setting subnet mask and gateway. This is useful for quickly assigning or updating IP addresses without modifying launch files.

> **Note:** The IP settings applied with `set_device_ip` are **permanent** and **will not be reset** if the device is powered off or restarted.

**Example Usage**

```
ros2 run orbbec_camera set_device_ip --ros-args \
-p old_ip:=192.168.1.10 \
-p dhcp:=false \
-p new_ip:=192.168.1.11 \
-p mask:=255.255.255.0 \
-p gateway:=192.168.1.1
```

**Parameters**

- **`old_ip`** – Current IP address of the device.
- **`dhcp`** – Set to `true` to use DHCP or `false` for static IP.
- **`new_ip`** – Static IP address to assign when DHCP is disabled.
- **`mask`** – Subnet mask for the new IP.
- **`gateway`** – Gateway address for the new IP.

## Force IP Function

The **Force IP** feature allows you to assign a **static IP address** to a network camera, overriding DHCP settings. This is useful when multiple network cameras are connected, and you need each device to have a fixed IP for reliable communication.

> **Note:** The Force IP configuration **will be reset if the device is powered off or restarted**. You need to reapply the settings after reboot.

**Parameters**

- **`force_ip_enable`** – Enable the Force IP function. **Default:** `false`
- **`force_ip_mac`** – Target device MAC address when multiple cameras are connected (e.g., `"54:14:FD:06:07:DA"`). You can use the `list_devices_node` to find the MAC of each device. **Default:** `""`
- **`force_ip_address`** – Static IP address to assign . **Default:** `192.168.1.10`
- **`force_ip_subnet_mask`** – Subnet mask for the static IP. **Default:** `255.255.255.0`
- **`force_ip_gateway`** – Gateway address for the static IP. **Default:** `192.168.1.1`

**Example Usage**

- **Enable Force IP for a specific device:**

```
ros2 launch orbbec_camera gemini_330_series.launch.py force_ip_enable:=true force_ip_mac:=54:14:FD:06:07:DA force_ip_address:=192.168.1.50 force_ip_subnet_mask:=255.255.255.0 force_ip_gateway:=192.168.1.1 net_device_ip:=192.168.1.50 net_device_port:=8090
```

> Tip: Make sure the camera is connected and its MAC address is correct before enabling Force IP. Use `list_devices_node` to check the MAC address of all connected cameras.



## Multi Net camera

For [multi_net_camera.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/examples/net_camera/multi_net_camera.launch.py) as an example:

```bash
ros2 launch orbbec_camera multi_net_camera.launch.py
```
