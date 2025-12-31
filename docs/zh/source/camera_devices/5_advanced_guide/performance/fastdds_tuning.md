# 针对Orbbec相机与ROS2的Fast DDS优化

使用默认配置时，Fast DDS表现出次优的传输效率，导致在ROS2中使用Orbbec相机时图像传输延迟显著。本文档提供了优化Fast DDS以提高图像传输效率的指导。

## 调整系统参数

**IP分片时间**

- **路径**: `/proc/sys/net/ipv4/ipfrag_time`（默认值：30秒）
- **目的**: 定义IP分片保留在内存中的持续时间。
- **调整**: 减少此值以缩短未接收到分片的时间窗口，这有助于减少延迟。考虑到您环境的具体需求，因为此设置会影响所有传入的分片。

  **示例**: 设置为3秒。

  ```bash
  sudo sysctl net.ipv4.ipfrag_time=3
  ```

**IP分片内存阈值**

- **路径**: `/proc/sys/net/ipv4/ipfrag_high_thresh`（默认值：262144字节）
- **目的**: 设置用于重组IP分片的最大内存。
- **调整**: 增加此值以允许更多内存用于分片重组，这可以改善较大数据包的处理。

  **示例**: 增加到128 MB。

  ```bash
  sudo sysctl net.ipv4.ipfrag_high_thresh=134217728
  ```

**最大缓冲区大小**

- **目的**: 配置接收和发送数据的最大缓冲区大小，这对高吞吐量数据传输至关重要。
- **调整**: 设置接收和发送操作的最大缓冲区大小。

  **命令**:

  ```bash
  sudo sysctl -w net.core.rmem_max=2147483647
  sudo sysctl -w net.core.rmem_default=2147483647
  sudo sysctl -w net.core.wmem_max=2147483647
  sudo sysctl -w net.core.wmem_default=2147483647
  ```

或者，通过将这些设置添加到 `/etc/sysctl.d/10-fastrtps-max.conf` 文件中使其永久生效。

```bash
sudo gedit /etc/sysctl.d/10-fastrtps-max.conf
```

将以下行添加到文件中：

```bash
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
net.core.wmem_max=2147483647
net.core.wmem_default=2147483647
```

然后保存并退出文件。运行 `sudo sysctl -p` 以应用更改。

有关详细指导，请参考 [ROS 2 DDS调优文档](https://docs.ros.org/en/foxy/How-To-Guides/DDS-tuning.html)。

## Fast DDS配置

以下是针对Orbbec相机与ROS2使用而优化的Fast DDS配置文件示例。此配置通过调整缓冲区大小和传输设置来增强整体数据传输。

**配置文件:** `shm_fastdds.xml`

将此文件放在 `$HOME` 目录中。

```xml
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UDP_transport</transport_id>
            <type>UDPv4</type>
            <maxInitialPeersRange>10</maxInitialPeersRange>
            <maxMessageSize>65000</maxMessageSize>
            <sendBufferSize>1048576</sendBufferSize>
            <receiveBufferSize>1048576</receiveBufferSize>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="participant_profile_ros2" is_default_profile="true">
        <rtps>
            <name>profile_for_ros2_context</name>
            <userTransports>
                <transport_id>UDP_transport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
            <sendSocketBufferSize>1048576</sendSocketBufferSize>
            <listenSocketBufferSize>1048576</listenSocketBufferSize>
            <builtin>
                <initialPeersList>
                    <locator>
                        <udpv4>
                            <address>127.0.0.1</address>
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
    <data_writer profile_name="default publisher profile" is_default_profile="true">
        <qos>
            <publishMode>
                <kind>ASYNCHRONOUS</kind>
            </publishMode>
            <latencyBudget>
                <duration>
                    <sec>0</sec>
                    <nanosec>1000000</nanosec>
                </duration>
            </latencyBudget>
        </qos>
        <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    </data_writer>
    <data_reader profile_name="default subscription profile" is_default_profile="true">
        <qos>
            <data_sharing>
                <kind>AUTOMATIC</kind>
            </data_sharing>
            <latencyBudget>
                <duration>
                    <sec>0</sec>
                    <nanosec>1000000</nanosec>
                </duration>
            </latencyBudget>
        </qos>
        <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    </data_reader>
</profiles>
```

**环境变量**

设置以下环境变量以使用自定义Fast DDS配置文件：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
```

此配置旨在优化数据流并减少传输延迟，提高Orbbec相机系统在ROS2环境中的响应性和可靠性。
