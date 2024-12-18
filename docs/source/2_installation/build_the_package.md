<!-- docs/source/2_installation/build_the_package.md -->

# Build the package

This section provides a comprehensive guide to installing, compiling, and running the OrbbecSDK_ROS2, covering all necessary steps for setup.

- Table of contents
  - [Get source code of OrbbecSDK_ROS2](#get-source-code-of-orbbecsdk-ros2)
  - [Install environment](#install-environment)
  - [Build project](#build-project)
  - [Performance Optimization Suggestions](#Performance Optimization Suggestions)

## Get source code of OrbbecSDK_ROS2

**Get source code from github:**    [https://github.com/orbbec/OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2)

```bash
mkdir -p ~/ros2_ws/src        # Create colcon workspace on your local disk
cd ~/ros2_ws/src
git clone -b v2-main https://github.com/orbbec/OrbbecSDK_ROS2.git        #Get source code
```


## Install environment

**Install ROS 2 environment**, refer to the official documentation: [ROS2 installation guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

```bash
# Tips: If your ROS2 command does not auto-complete, put the following two lines into your `.bashrc` or `.zshrc`

eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

**Install deb dependencies:**

```bash
# assume you have sourced ROS environment, same blow
sudo apt install libgflags-dev nlohmann-json3-dev \
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
ros-$ROS_DISTRO-backward-ros libdw-dev ros-$ROS_DISTRO-image-transport \
ros-$ROS_DISTRO-image-transport-plugins ros-$ROS_DISTRO-compressed-image-transport \
ros-$ROS_DISTRO-rqt-tf-tree -y
```

**Install udev rules:**

```bash
tar -zxvf OrbbecSDK_ROS2_xxx.tar.gz -C ~/ros2_ws/src
cd  ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Build project

```bash
cd ~/ros2_ws/
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Verify the build results.
[start single camera](../3_start_single_camera/start_single_camera.md)

## Performance Optimization Suggestions

### Optimization of usbfs_memory Parameters in USB Camera

**Increase usbfs_memory_mb Value**

- Increase the `usbfs_memory_mb` value to 128MB (this is a reference value and can be adjusted based on your system’s needs)
  by running the following command:

```bash
echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

- To make this change permanent, check [this link](https://github.com/OpenKinect/libfreenect2/issues/807).
  There are two ways to persist the configuration: by modifying GRUB or by adding a systemd service.

**by modifying GRUB**

Open /etc/default/grub file,Find and replace

```bash
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
```

  with this

```bash
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=128"
```

  Update grub

```bash
$ sudo update-grub
```

  Reboot and check

```bash
$ cat /sys/module/usbcore/parameters/usbfs_memory_mb
```

**by adding a systemd service**

  Create the `/etc/systemd/system/usbfs-memory.service` file

```bash
sudo vi /etc/systemd/system/usbfs-memory.service
```

  Paste the following content into the file:

```bash
[Unit]
Description=Set USBFS memory limit
After=multi-user.target

[Service]
ExecStart=/bin/bash -c 'echo 128 | tee /sys/module/usbcore/parameters/usbfs_memory_mb'
ExecStartPost=/bin/bash -c 'echo "USBFS memory limit set to 128 MB"'

[Install]
WantedBy=multi-user.target
```

  Reload the systemd configuration to apply the new service

```bash
sudo systemctl daemon-reload
sudo systemctl enable usbfs-memory.service
sudo systemctl start usbfs-memory.service
```

  Verify the service status

```bash
sudo systemctl status usbfs-memory.service
cat /sys/module/usbcore/parameters/usbfs_memory_mb
```

### Optimizing ROS DDS Configuration

**CycloneDDS Tuning**

If you use CycloneDDS, please refer to the [CycloneDDS Tuning](../6_advanced/cyclonedds_tuning.md) file.

The default DDS settings may not be optimal for data transmission. Different DDS settings can have varying performance. For more detailed information, please refer to the [CycloneDDS official website](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html).

**FastDDS Tuning**

If you use FastDDS, please refer to the [FastDDS Tuning](../6_advanced/fastdds_tuning.md) file.
