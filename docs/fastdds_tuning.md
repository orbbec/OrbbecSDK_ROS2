# Fast DDS Optimization for Orbbec Camera with ROS2

When operating with the default configuration, Fast DDS exhibits suboptimal transmission efficiency, resulting in
significant image transmission delays when used with the Orbbec camera in ROS2. This document provides guidance on
optimizing Fast DDS to enhance image transfer efficiency.

## 1. Adjusting System Parameters

### IP Fragmentation Time

- **Path**: `/proc/sys/net/ipv4/ipfrag_time` (default: 30 seconds)
- **Purpose**: Defines the duration that IP fragments are kept in memory.
- **Adjustment**: Decrease this value to reduce the time window where no fragments are received, which can help reduce
  delays. Consider the specific needs of your environment as this setting affects all incoming fragments.

  **Example**: Set to 3 seconds.

  ```bash
  sudo sysctl net.ipv4.ipfrag_time=3
  ```

### IP Fragmentation Memory Threshold

- **Path**: `/proc/sys/net/ipv4/ipfrag_high_thresh` (default: 262144 bytes)
- **Purpose**: Sets the maximum memory used to reassemble IP fragments.
- **Adjustment**: Increase this value to allow more memory for fragment reassembly, which can improve handling of larger
  data packets.

  **Example**: Increase to 128 MB.

  ```bash
  sudo sysctl net.ipv4.ipfrag_high_thresh=134217728
  ```

### Maximum Buffer Sizes

- **Purpose**: Configures the maximum buffer sizes for receiving and sending data, which is critical for high-throughput
  data transmission.
- **Adjustment**: Set the maximum buffer sizes for both receiving and sending operations.

  **Commands**:

  ```bash
  sudo sysctl -w net.core.rmem_max=2147483647
  sudo sysctl -w net.core.rmem_default=2147483647
  sudo sysctl -w net.core.wmem_max=2147483647
  sudo sysctl -w net.core.wmem_default=2147483647
  ```

Alternatively, make these settings permanent by adding them to the `/etc/sysctl.d/10-fastrtps-max.conf` file.

```bash
sudo gedit /etc/sysctl.d/10-fastrtps-max.conf
```

add blow lines to the file:

```bash
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
net.core.wmem_max=2147483647
net.core.wmem_default=2147483647
```

then save and exit the file. run `sudo sysctl -p` to apply the changes.

For detailed guidance, refer
to [ROS 2 DDS Tuning Documentation](https://docs.ros.org/en/foxy/How-To-Guides/DDS-tuning.html).

## 2. Fast DDS Configuration

Below is an example of a Fast DDS configuration file optimized for ROS2 usage with the Orbbec camera. This configuration
enhances the overall data transmission by adjusting buffer sizes and transport settings.

### Configuration File: `shm_fastdds.xml`

Place this file in the `$HOME` directory.

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

### Environment Variables

Set the following environment variables to use the custom Fast DDS profile:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
```

This configuration aims to optimize the data flow and reduce transmission delays, improving the responsiveness and
reliability of the Orbbec camera system in a ROS2 environment.
