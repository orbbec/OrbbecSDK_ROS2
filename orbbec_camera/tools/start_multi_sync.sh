#!/bin/bash

# Function to reset timestamp
reset_timestamp() {
    if ! ros2 service call /G330_0/set_reset_timestamp std_srvs/srv/SetBool '{data: true}'; then
        printf "Error: Failed to publish start_capture message\n" >&2
        return 1
    fi
    echo "Timestamp reset successfully."
}

# Function to sync interleaver laser
sync_interleaverlaser() {
    if ! ros2 service call /G330_0/set_sync_interleaverlaser orbbec_camera_msgs/srv/SetInt32 '{data: 0}'; then
        printf "Error: Failed to launch Orbbec camera\n" >&2
        return 1
    fi
}

# Function to save image
save_image() {
    printf "Executing save_image.sh...\n"
    if ! ros2 topic pub --once /start_capture std_msgs/msg/Int32 "{data: 100}"; then
        printf "Error: Failed to publish start_capture message\n" >&2
        return 1
    fi
    echo "Image saved successfully."
}

# Function to source the setup script
setup_environment() {
    local setup_script="install/setup.bash"
    if [[ -f "$setup_script" ]]; then
        source "$setup_script"
        printf "Environment setup sourced successfully.\n"
    else
        printf "Error: Setup script %s not found\n" "$setup_script" >&2
        return 1
    fi
}
# Function to adjust USB memory settings
set_usb_memory() {
    local usb_memory_value=700
    if ! sudo sh -c "echo $usb_memory_value > /sys/module/usbcore/parameters/usbfs_memory_mb"; then
        printf "Error: Failed to set USB memory value\n" >&2
        return 1
    fi
    printf "USB memory set to %d MB.\n" "$usb_memory_value"
}
# Function to run the Orbbec multi_save_rgbir_node command in the background
run_multi_save_rgbir_node() {
    ros2 run orbbec_camera multi_save_rgbir_node &
    local pid=$!
    if ! kill -0 "$pid" 2>/dev/null; then
        printf "Error: Failed to run multi_save_rgbir_node\n" >&2
        return 1
    fi
    printf "multi_save_rgbir_node is running in the background with PID %d.\n" "$pid"
}
# Function to launch ROS2 Orbbec camera with multi camera sync
launch_orbbec_camera() {
    if ! ros2 launch orbbec_camera multi_camera_synced.launch.py; then
        printf "Error: Failed to launch Orbbec camera\n" >&2
        return 1
    fi
    printf "Orbbec camera launched successfully.\n"
}
# Function to run camera
run_camera() {
    setup_environment || return 1
    set_usb_memory || return 1
    # Run multi_save_rgbir_node in the background and delay for 2 seconds before launching the camera
    run_multi_save_rgbir_node || return 1
    sleep 2
    launch_orbbec_camera || return 1
}


# Function to display the menu
display_menu() {
    printf "\nSelect a function to run:\n"
    printf "1. run_camera\n"
    printf "2. reset_timestamp\n"
    printf "3. sync_interleaverlaser\n"
    printf "4. save_image\n"
    printf "0. Exit\n"
}

# Function to execute the selected function
run_function() {
    case $1 in
        1)
            run_camera
            ;;
        2)
            reset_timestamp
            ;;
        3)
            sync_interleaverlaser
            ;;
        4)
            save_image
            ;;
        0)
            printf "Exiting the script...\n"
            return 0
            ;;
        *)
            printf "Invalid choice. Please select a valid option.\n" >&2
            return 1
            ;;
    esac
}

# Main function to run the menu system
main() {
    setup_environment || return 1
    while true; do
        display_menu
        read -rp "Enter your choice: " choice
        if ! run_function "$choice"; then
            continue
        fi
        if [ "$choice" -eq 0 ]; then
            break
        fi
    done
}

# Call the main function
main
