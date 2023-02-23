#!/bin/sh

set -e

# Check if script is running as root or with sudo
if [ "$(id -u)" -ne 0 ]; then
    echo "Please run this script with sudo"
    exit 1
fi

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
if [ "$(uname -s)" != "Darwin" ]; then
    # Install UDEV rules for USB device
    cp "$SCRIPT_DIR/99-obsensor-libusb.rules" /etc/udev/rules.d/99-obsensor-libusb.rules
    echo "USB rules file installed at /etc/udev/rules.d/99-obsensor-libusb.rules"
fi

echo "Done"
