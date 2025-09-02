#!/usr/bin/bash
set -e
CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
rm -fr debian obj-x86_64-linux-gnu .obj-aarch64-linux-gnu || true
export DEBIAN_INC=1
export BUILDING_PACKAGE=1
bloom-generate rosdebian --os-name ubuntu --ros-distro $ROS_DISTRO -i ${DEBIAN_INC}.$(date +%Y%m%d.%H%M%S)
sed -i 's/dh $@/dh $@ --parallel/g' debian/rules

sed -i '/^override_dh_shlibdeps:/,/^$/d' debian/rules

cat >> debian/rules << 'EOF'

override_dh_shlibdeps:
	dh_shlibdeps --dpkg-shlibdeps-params=--ignore-missing-info
EOF

PARALLEL=$(nproc)
if [ "$PARALLEL" -gt 3 ]; then PARALLEL=3; fi
export DEB_BUILD_OPTIONS="parallel=$PARALLEL"
fakeroot debian/rules binary