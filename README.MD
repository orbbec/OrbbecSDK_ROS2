# orbbec_camera

---
ORBBEC_SDK ROS2 wrapper for orbbec 3d camera 

## Installation Instructions

---
Install ROS2

* Add ROS2 gpg key

```bash
sudo apt install curl gnupg2
sudo curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

* Add tuna ROS2 mirror

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
 
sudo apt update
```

* Install ROS2

```bash
sudo apt-get  install ros-galactic-desktop
sudo apt-get install python3-colcon-common-extensions

```
>>If your ROS2 command does not auto-complete, put the following two lines into your `.bashrc`
or `.zshrc`

```bash
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

Create `colcon` workspace Ubuntu

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone source code 

```bash
git clone gitlab@code.orbbec.com.cn:OrbbecSDK/orbbecros2sdk.git
```
Install `magic_enum`

```bash
cd orbbec_camera/dependencies
tar -xzvf magic_enum-0.8.0.tar.gz
cd magic_enum-0.8.0
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig
```

Install `ORBBEC_SDK`

```bash
cd orbbec_camera/dependencies
tar -xzvf sensor_sdk_v1.2.8_linux.tar.gz
cd SDK/lib
sudo cp libOrbbecSDK.so.1.2.8  libpostfilter.so /usr/local/lib
sudo ln -s /usr/local/lib/libOrbbecSDK.so.1.2.8 /usr/local/lib/libOrbbecSDK.so.1.2
sudo ln -s /usr/local/lib/libOrbbecSDK.so.1.2.8 /usr/local/lib/libOrbbecSDK.so
cd ../include 
sudo cp -frd libobsensor /usr/local/include
sudo ldconfig
```
Build

```bash
cd ~/ros2_ws/
colcon build --event-handlers console_direct+
```
Launch camera node 
* On terminal 1
```bash 
. ./install/setup.bash 
ros2 launch orbbec_camera orbbec.launch.py
```
* On terminal 2

```bash
. ./install/setup.bash 
rviz2 -d src/orbbec_camera/rviz/pointcloud.rviz
```
* List topics / services/ parameters ( on terminal 3)

```bash
ros2 topic list
ros2 service list
ros2 param list
```

---

## Camera control && Information service

---

* Show depth to color extrinsic

```bash
ros2 topic echo --qos-durability=transient_local /camera/extrinsic/depth_to_color  --qos-profile=services_default
```
* Get device info 

```bash 
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo "{}"
```
* Get SDK version

```bash
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString "{}"

```

* Get exposure

```bash
ros2 service call /camera/get_color_exposure orbbec_camera_msgs/srv/GetInt32 "{}"
```
> If your check `ir` or `depth`, please change ` /camera/get_color_exposure`
to  `/camera/get_ir_exposure` or `/camera/get_depth_exposure`, Same below.

* Get gain

```bash
ros2 service call /camera/get_color_gain orbbec_camera_msgs/srv/GetInt32 "{}"
```
* Get white balance 

```bash
ros2 service call /camera/get_white_balance orbbec_camera_msgs/srv/GetInt32 "{}"
```

* Set auto exposure 

```bash
ros2 service call /camera/set_color_auto_exposure std_srvs/srv/SetBool '{data: false}'
```

* Set white balance 
```bash
ros2 service call /camera/set_white_balance orbbec_camera_msgs/srv/SetInt32 '{data: 4600}'
```
