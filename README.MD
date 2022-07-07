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

Install deb dependencies

```bash
sudo apt install libgflags-dev nlohmann-json3-dev \
ros-galactic-image-transport ros-galactic-image-publisher
```

Install glog

```bash
cd orbbec_camera/dependencies
tar -xzvf glog-0.6.0.tar.gz
cd glog-0.6.0
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig
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

---

## Getting start

---

```bash
cd ~/ros2_ws/
# build release, Default is Debug
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
```
Launch camera node 
* On terminal 1
```bash 
. ./install/setup.bash 
ros2 launch orbbec_camera ob_camera.launch.py
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

* Show depth to color extrinsic

```bash
ros2 topic echo --qos-durability=transient_local /camera/extrinsic/depth_to_color  --qos-profile=services_default
```
* Get device info 

```bash 
ros2 service call /camera/get_device_info orbbec_camera_msgs/srv/GetDeviceInfo '{}'
```
* Get SDK version

```bash
ros2 service call /camera/get_sdk_version orbbec_camera_msgs/srv/GetString '{}'

```

* Get exposure

```bash
ros2 service call /camera/get_color_exposure orbbec_camera_msgs/srv/GetInt32 '{}'
```
> If your check `ir` or `depth`, please change ` /camera/get_color_exposure`
to  `/camera/get_ir_exposure` or `/camera/get_depth_exposure`, Same below.

* Get gain

```bash
ros2 service call /camera/get_color_gain orbbec_camera_msgs/srv/GetInt32 '{}'
```
* Get white balance 

```bash
ros2 service call /camera/get_white_balance orbbec_camera_msgs/srv/GetInt32 '{}'
```

* Set auto exposure 

```bash
ros2 service call /camera/set_color_auto_exposure std_srvs/srv/SetBool '{data: false}'
```

* Set white balance

```bash
ros2 service call /camera/set_white_balance orbbec_camera_msgs/srv/SetInt32 '{data: 4600}'
```
* Set laser enable

```bash
ros2 service call  /camera/set_laser_enable std_srvs/srv/SetBool "{data: true}" 
* ```

* toggle sensor 

```bash
ros2 service call  /camera/toggle_ir std_srvs/srv/SetBool "{data : true}"
* ```
---
## Known issues
* TODO:
