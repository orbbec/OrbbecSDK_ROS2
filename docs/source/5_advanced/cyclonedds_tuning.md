# CycloneDDS tuning

● Edit cyclonedds configuration file

```bash
sudo gedit /etc/cyclonedds/config.xml
```

Add

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/confighttps://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Internal>
            <MinimumSocketReceiveBufferSize>16MB</MinimumSocketReceiveBufferSize>
        </Internal>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
            <Peers>
                <Peer address="localhost"/>
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
```

● Set the environment variables, add to `.zshrc` or `.bashrc`

```bash
export ROS_DOMAIN_ID=42 # Numbers from 0 to 232
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml
```

Tips:to understand why the maximum ROS_DOMAIN_ID is 232, please visit [The ROS DOMAIN ID](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)

● Increase UDP receive buffer size
Edit

```bash
/etc/sysctl.d/10-cyclone-max.conf
```

Add

```bash
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
```
