# Building a Debian Package

## Preparing the Environment

Before starting, install the required tools:

```bash
sudo apt install debhelper fakeroot python3-bloom
```

## Configuring ROS Dependencies

Add the following YAML file to your system at `/etc/ros/rosdep/sources.list.d/00-orbbec.yaml`. Make sure to
replace `focal` with the codename of your Ubuntu version and `humble` with your ROS2 distribution name:

```yaml
orbbec_camera_msgs:
  ubuntu:
    focal: [ ros-humble-orbbec-camera-msgs ]
```

Next, create a new file `/etc/ros/rosdep/sources.list.d/50-orbbec.list` and add this line to specify the path to the
YAML file:

```bash
yaml file:///etc/ros/rosdep/sources.list.d/00-orbbec.yaml
```

Update the rosdep database to reflect these changes:

```bash
rosdep update
```

## Building the Package

Navigate to your workspace and build the project:

```bash
cd ~/ros2_ws/
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
cd src/OrbbecSDK_ROS2/
bash .make_deb.sh
```
