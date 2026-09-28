# AE/AWB Lock Test

The source file is in [ae_awb_lock](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples/ae_awb_lock).

This sample exposes a test-only ROS 2 action that verifies the AE/AWB capture and manual
lock-in flow through the camera driver's services and color-frame metadata.

Run the camera driver and this sample in the same namespace:

```bash
ros2 run orbbec_camera ae_awb_lock_test_node --ros-args -r __ns:=/camera
```

Send a goal and print feedback:

```bash
ros2 action send_goal \
  /camera/run_ae_awb_lock_test \
  orbbec_camera_msgs/action/RunAeAwbLockTest \
  "{timeout_ms: 10000}" \
  --feedback
```

The sample subscribes to the relative `color/metadata` topic. It enables auto exposure and auto
white balance, waits until the SDK status equals `1`, captures exposure, color gain, and color
temperature from the latest color-frame metadata, and reads AWB R/B/G gains through the structured
property service. It then disables the auto controls and writes the captured values back in this
order:

1. Color exposure
2. Color gain
3. AWB R/B/G gains
4. Color temperature

The final AWB gain readback must exactly match the captured value. Other readback differences are
reported as warnings because the device may quantize those controls. On failure or cancellation,
the sample restores auto exposure and auto white balance.

Every feedback phase contains a fresh status value read from the camera service. The
`waiting_for_services` feedback is published after all required services become available, because
the status cannot be read before its service is ready.

The color stream must be enabled, and `/camera/color/metadata` must be available when using the
`/camera` namespace. The action fails instead of writing default values if no metadata arrives
before the goal timeout or if `exposure`, `gain`, or `white_balance` is missing. The goal timeout
covers the main workflow, including service discovery, service calls, convergence, capture,
writeback, and verification. Restoring auto exposure and auto white balance after a failure uses a
separate best-effort timeout.
