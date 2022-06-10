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
>>If your ROS2 command can not auto complete, please put blow 2 lines in your `.bashrc`
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
