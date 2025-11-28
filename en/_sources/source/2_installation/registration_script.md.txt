## Registration Script (Required)

To allow the Orbbec cameras to be recognized correctly on Linux, install the udev rules.

### Binary Installation

```bash
sudo cp /opt/ros/$ROS_DISTRO/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Build from Source

```bash
cd  ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

This step is **mandatory** for Linux users.

`Notes:` If this script is not executed, open the device will fail due to permission issues. You need to run the sample with sudo (administrator privileges).
