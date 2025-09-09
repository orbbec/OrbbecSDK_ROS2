#!/usr/bin/bash
set -e
CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
sub_dir=(
  orbbec_camera_msgs
  orbbec_camera
)

for pkg in "${sub_dir[@]}"; do
  cd "${pkg}"
  bash .make_deb.sh
  cd "${CURR_DIR}"
done
