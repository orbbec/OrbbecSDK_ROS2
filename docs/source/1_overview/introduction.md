# Introduction

OrbbecSDK ROS2 Wrapper provides seamless integration of Orbbec cameras with ROS 2 environment. It supports ROS2 Foxy, Humble, and Jazzy distributions.

With a major update in October 2024, we release the [OrbbecSDK ROS2 Wrapper v2](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main) connected to the open source [OrbbecSDK v2](https://github.com/orbbec/OrbbecSDK_v2/releases) with enhanced flexibility and extensibility. This update ensures compatibility with all Orbbec USB products adhering to UVC standard. However, it no longer supports Orbbec's traditional OpenNI protocol devices. We strongly encourage you to use the v2-main branch if your device is supported.

If you are a user in China, it is recommended to use [gitee Repo](https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS2).

## Support Hardware Products

This document is based on the v2-main branch code, which is a Python Wrapper built on Orbbec SDK v2. It supports the following devices.
If your device is included in this supported list, we recommend using the v2-main branch. If not, you can use the main branch instead.

| **Products List** | **Minimal Firmware Version** |
|-------------------|------------------------------|
| Gemini 435Le        | 1.2.04                       |
| Gemini 335Le        | 1.5.31                       |
| Gemini 330        | 1.2.20                       |
| Gemini 330L       | 1.2.20                       |
| Gemini 335        | 1.2.20                       |
| Gemini 335L       | 1.2.20                       |
| Gemini 336        | 1.2.20                       |
| Gemini 336L       | 1.2.20                       |
| Gemini 335Lg      | 1.3.46                       |
| Femto Bolt        | 1.1.2                  |
| Femto Mega        | 1.3.0                  |
| Femto Mega I        | 2.0.4                  |
| Astra 2           | 2.8.20                       |
| Gemini 2 L        | 1.4.53                       |
| Gemini 2          | 1.4.92               |
| Gemini 215        | 1.0.9                        |
| Gemini 210        | 1.0.9                        |

the main branch supports the following devices:


| **Products List** | **Minimal Firmware Version**        |
|-------------------|-----------------------------|
| Gemini 330        | 1.2.20                   |
| Gemini 330L        | 1.2.20                    |
| Gemini 335        | 1.2.20                   |
| Gemini 335L        | 1.2.20                    |
| Gemini 336        | 1.2.20                      |
| Gemini 336L        | 1.2.20                    |
| Femto Bolt        | 1.0.6                |
| Femto Mega        | 1.1.7                 |
| Femto Mega I      | 2.0.2                       |
| Gemini 2 XL       | Obox: V1.2.5  VL:1.4.54     |
| Astra 2           | 2.8.20                      |
| Gemini 2 L        | 1.4.32                      |
| Gemini 2          | 1.4.60               |
| Astra+            | 1.0.19 |
| Femto             | 1.6.7                       |
| Femto W           | 1.1.8                       |
| DaBai             | 2436                        |
| DaBai DCW         | 2460                        |
| DaBai DW          | 2606                        |
| Astra Mini Pro    | 1007                        |
| Gemini E          | 3460                        |
| Gemini E Lite     | 3606                        |
| Gemini            | 3018                      |
| Astra Mini S Pro  | 1005                      |


## Support Platforms

- Linux: 18.04/20.04/22.04/24.04 (x64)
- Arm64: Ubuntu18.04/20.04/22.04
