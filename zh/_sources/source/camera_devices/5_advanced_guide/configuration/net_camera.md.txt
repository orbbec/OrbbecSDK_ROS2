# 网络相机

> 本节介绍如何在 OrbbecSDK_ROS2 中使用网络相机。目前仅支持 Femto_Mega、Gemini 335Le 和 Gemini 435Le 设备，其他网络设备将在不久的将来得到支持。

您可以在 [example](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples) 中找到示例使用代码。

## Femto Mega & Gemini 435Le & Gemini 335Le

**参数介绍**

网络设备设置：`enumerate_net_device` 设置为 true，将自动枚举网络设备。

如果您不想自动枚举网络设备，可以将 `enumerate_net_device` 设置为 false，将 `net_device_ip` 设置为设备的 IP 地址，并将 `net_device_port` 设置为默认值 8090。

* `enumerate_net_device`：启用自动枚举网络设备。
* `net_device_ip`：设置网络设备的 IP 地址。
* `net_device_port`：设置网络设备的端口。通常可以设置为 8090。

**单个网络相机**

> 如果您需要运行 Gemini 435Le/Gemini 335Le，只需在运行命令中将 [femto_mega.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/launch/femto_mega.launch.py) 替换为 [gemini435_le.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/launch/gemini435_le.launch.py)/[gemini_330_series.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/launch/gemini_330_series.launch.py)

以 [femto_mega.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/launch/femto_mega.launch.py) 为例：

- **自动枚举网络设备：**

```bash
ros2 launch orbbec_camera femto_mega.launch.py enumerate_net_device:=true
```

- **指定 IP 地址启动设备：**

注意：`net_device_ip` 需要更改为设备的 IP 地址，这里是 192.168.1.10

```bash
ros2 launch orbbec_camera femto_mega.launch.py enumerate_net_device:=false net_device_ip:=192.168.1.10 net_device_port:=8090
```

**多个网络相机**

以 [multi_net_camera.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/examples/net_camera/multi_net_camera.launch.py) 为例：

```bash
ros2 launch orbbec_camera multi_net_camera.launch.py
```

## set_device_ip 工具

**`set_device_ip`** 可执行文件允许您直接从 ROS 2 配置网络相机的 IP 设置，包括在 DHCP 和静态 IP 之间切换，以及设置子网掩码和网关。这对于快速分配或更新 IP 地址而无需修改启动文件非常有用。

> **注意：**使用 `set_device_ip` 应用的 IP 设置是**永久性的**，如果设备断电或重启，**不会重置**。

**示例用法**

```bash
ros2 run orbbec_camera set_device_ip --ros-args \
-p old_ip:=192.168.1.10 \
-p dhcp:=false \
-p new_ip:=192.168.1.11 \
-p mask:=255.255.255.0 \
-p gateway:=192.168.1.1
```

**参数**

- **`old_ip`** – 设备的当前 IP 地址。
- **`dhcp`** – 设置为 `true` 使用 DHCP 或 `false` 使用静态 IP。
- **`new_ip`** – 禁用 DHCP 时要分配的静态 IP 地址。
- **`mask`** – 新 IP 的子网掩码。
- **`gateway`** – 新 IP 的网关地址。

## 强制 IP 功能

**强制 IP** 功能允许您为网络相机分配**静态 IP 地址**，覆盖 DHCP 设置。当连接多个网络相机时，这非常有用，您需要每个设备具有固定的 IP 以实现可靠的通信。

> **注意：**如果设备断电或重启，强制 IP 配置**将被重置**。您需要在重启后重新应用设置。

**参数**

- **`force_ip_enable`** – 启用强制 IP 功能。**默认值：** `false`
- **`force_ip_mac`** – 连接多个相机时的目标设备 MAC 地址（例如 `"54:14:FD:06:07:DA"`）。您可以使用 `list_devices_node` 查找每个设备的 MAC。**默认值：** `""`
- **`force_ip_address`** – 要分配的静态 IP 地址。**默认值：** `192.168.1.10`
- **`force_ip_subnet_mask`** – 静态 IP 的子网掩码。**默认值：** `255.255.255.0`
- **`force_ip_gateway`** – 静态 IP 的网关地址。**默认值：** `192.168.1.1`

**示例用法**

- **为特定设备启用强制 IP：**

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
force_ip_enable:=true \
force_ip_mac:=54:14:FD:06:07:DA \
force_ip_address:=192.168.1.50 \
force_ip_subnet_mask:=255.255.255.0 \
force_ip_gateway:=192.168.1.1 \
net_device_ip:=192.168.1.50 \
net_device_port:=8090
```
