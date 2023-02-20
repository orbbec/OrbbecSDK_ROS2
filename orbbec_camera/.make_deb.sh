#!/usr/bin/bash
set -e
CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
rm -fr debian obj-x86_64-linux-gnu
export DEBIAN_INC=1
bloom-generate rosdebian --os-name ubuntu --ros-distro $ROS_DISTRO -i ${DEBIAN_INC}.$(date +%Y%m%d.%H%M%S)
export DEB_BUILD_OPTIONS="parallel=$(($(nproc) - 1))"
fakeroot debian/rules binary
