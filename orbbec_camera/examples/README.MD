# OrbbecSDK ROS2 Examples

This directory contains practical launch and tool examples. Most example READMEs
contain local setup and run commands; their full usage, configuration, and
troubleshooting are maintained on the official documentation site. The Gemini
435Le example keeps its complete instructions next to its source code.

For command-line maintenance and diagnostic utilities, see the official
[Tool Index](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/6_benchmark/tools.html).

## Example Index

| Source | Purpose | Experience Level | Usage Guide |
| :---: | --- | :---: | --- |
| [Multi-camera network](./multi_net_camera) | Launch multiple supported Orbbec network cameras. | ⭐️ | [EN](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/configuration/net_camera.html) / [中文](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/camera_devices/5_advanced_guide/configuration/net_camera.html) |
| [GigE Action Command](./gige_action_command) | Configure and trigger a Gemini 335Le camera group over GVCP. | ⭐️⭐️ | [EN](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/multi_camera/gige_action_command.html) / [中文](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/camera_devices/5_advanced_guide/multi_camera/gige_action_command.html) |
| [GMSL camera](./gmsl_multi_camera_sync) | Synchronize two GMSL-connected Gemini 330 Series cameras. | ⭐️⭐️ | [EN](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/multi_camera/gmsl_camera.html) / [中文](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/camera_devices/5_advanced_guide/multi_camera/gmsl_camera.html) |
| [Shared component container](./multi_camera_shared_container) | Load multiple Gemini 330 Series cameras into one component container. | ⭐️⭐️ | [EN](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/performance/efficient_intra_process_communication.html) / [中文](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/camera_devices/5_advanced_guide/performance/efficient_intra_process_communication.html) |
| [AE/AWB lock test](./ae_awb_lock) | Verify AE/AWB convergence, frame-metadata capture, and manual lock-in through ROS 2. | ⭐️⭐️ | [EN](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/4_application_guide/examples/ae_awb_lock.html) / [中文](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/camera_devices/4_application_guide/examples/ae_awb_lock.html) |
| [Gemini 435Le example node](./Gemini_435Le_example_node) | Try camera services and inspect device status through an interactive menu. | ⭐️⭐️ | [Example README](./Gemini_435Le_example_node/README.MD) |
| [Benchmark](./benchmark) | Benchmark the performance of different camera configurations. | ⭐️⭐️ | [EN](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/6_benchmark/benchmark_tools.html) / [中文](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/camera_devices/6_benchmark/benchmark_tools.html) |
| [Multi-camera synchronization verification](./multi_camera_synced_verification_tool) | Verify synchronization accuracy across multiple cameras. | ⭐️⭐️⭐️ | [EN](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/multi_camera/multi_camera_synced_verification_tool.html) / [中文](https://orbbec.github.io/OrbbecSDK_ROS2/zh/source/camera_devices/5_advanced_guide/multi_camera/multi_camera_synced_verification_tool.html) |
