#!/usr/bin/env bash
source /opt/ros/galactic/setup.bash

ros2 topic echo --qos-durability=transient_local /camera/extrinsic/depth_to_color  --qos-profile=services_default
