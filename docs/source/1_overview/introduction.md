# Introduction

OrbbecSDK ROS2 Wrapper provides seamless integration of Orbbec cameras with ROS 2 environment. It supports ROS2 Foxy, Humble, and Jazzy distributions.

With a major update in October 2024, we release the [OrbbecSDK ROS2 Wrapper v2](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main) connected to the open source [OrbbecSDK v2](https://github.com/orbbec/OrbbecSDK_v2/releases) with enhanced flexibility and extensibility. This update ensures compatibility with all Orbbec USB products adhering to UVC standard. However, it no longer supports Orbbec's traditional OpenNI protocol devices. We strongly encourage you to use the v2-main branch if your device is supported.

If you are a user in China, it is recommended to use [gitee Repo](https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS2).

## Support Hardware Products

The following devices are supported by the OrbbecSDK ROS2 Wrapper v2-main branch.  More devices support will be added in the near future. If you can not find your device in the table below, try the [main](https://github.com/orbbec/OrbbecSDK_ROS2)  branch.

For optimal performance, we strongly recommend updating to the latest firmware version. This ensures that you benefit from the most recent enhancements and bug fixes.

| Product List   | Minimal Firmware Version | **Launch File**             |
| :------------- | :----------------------- | :-------------------------- |
| Gemini 435Le   | 1.2.04                   | gemini435_le.launch.py      |
| Gemini 335     | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 336     | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 335L    | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 336L    | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 335Lg   | 1.3.46                   | gemini_330_series.launch.py |
| Gemini 335Le   | 1.5.31                   | gemini_330_series.launch.py |
| Gemini 330     | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 330L    | 1.2.20                   | gemini_330_series.launch.py |
| Gemini 2       | 1.4.92                   | gemini2.launch.py           |
| Gemini 2 L     | 1.4.53                   | gemini2L.launch.py          |
| Femto Bolt     | 1.1.2                    | femto_bolt.launch.py        |
| Femto Mega     | 1.3.0                    | femto_mega.launch.py        |
| Astra 2        | 2.8.20                   | astra2.launch.py            |
| Astra Mini Pro | 2.0.01                   | astra.launch.py             |

All launch files are essentially similar, with the primary difference being the default values of the parameters set
for different models within the same series. Differences in USB standards, such as USB 2.0 versus USB 3.0, may require adjustments to these parameters. If you encounter a startup failure, please carefully review the specification manual. Pay special attention to the resolution settings in the launch file, as well as other parameters, to ensure compatibility and optimal performance.

## Support Platforms

- Linux x64: tested on Ubuntu 20.04
- Linux ARM64: tested on NVIDIA Jetson AGX Orin , NVIDIA Jetson Orin NX , NVIDIA Jetson Orin Nano , NVIDIA Jetson AGX Xavier , NVIDIA Jetson Xavier NX
