#!/bin/bash

source /opt/ros/galactic/setup.bash

ros2 service call /camera_01/save_images std_srvs/srv/Empty "{}"
ros2 service call /camera_02/save_images std_srvs/srv/Empty "{}"
ros2 service call /camera_03/save_images std_srvs/srv/Empty "{}"
ros2 service call /camera_04/save_images std_srvs/srv/Empty "{}"
ros2 service call /camera_05/save_images std_srvs/srv/Empty "{}"

wait
