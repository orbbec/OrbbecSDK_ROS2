#!/usr/bin/bash
set -e
CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
rm -fr debian .obj-x86_64-linux-gnu
bloom-generate rosdebian --os-name ubuntu --ros-distro $ROS_DISTRO  
sed -i 's/$@/$@ --parallel/g' debian/rules
PKG_NAME=$(cat package.xml| grep "<name>" | awk -F '>' '{print $2}' | awk -F '<'  '{print $1}')
PKG_NAME=$(echo "ros-$ROS_DISTRO-${PKG_NAME}" | tr '_' '-')
VERSION=$(cat package.xml| grep "<version>" | awk -F '>' '{print $2}' | awk -F '<'  '{print $1}')
STAMP=$(date  +'%Y%m%d.%H%M%S')
sed -i "1 s/.*/${PKG_NAME} (${VERSION}-1focal.${STAMP}) focal; urgency=high/g" debian/changelog
export DEB_BUILD_OPTIONS="parallel=$(($(nproc) - 1))"
fakeroot debian/rules binary
