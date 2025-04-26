#!/bin/bash

VID="2bc5"

for dev in /sys/bus/usb/devices/*; do
  if [ -e "$dev/idVendor" ]; then
    vid=$(cat "$dev/idVendor")
    if [ "$vid" == "${VID}" ]; then
      port=$(basename $dev)
      product=$(cat "$dev/product" 2>/dev/null) # product name
      serial=$(cat "$dev/serial" 2>/dev/null) # serial number
      echo "Found Orbbec device $product, usb port $port, serial number $serial"
    fi
  fi
done
