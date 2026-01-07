# Introduction

OrbbecSDK ROS2 Wrapper provides seamless integration of Orbbec cameras with ROS 2 environment. It supports ROS2 Foxy, Humble, and Jazzy distributions.

By default, we recommend using the **v2-main** branch. For older OpenNI devices not supported by v2-main, please use the **main** branch. Device models that are only supported by the main branch are listed in the table below.

If you are a user in China, it is recommended to use [gitee Repo](https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS2).

Here is the device support list of main branch (v1.x) and v2-main branch (v2.x):

<table border="1" style="border-collapse: collapse; text-align: left; width: 100%;">
  <thead>
    <tr style="background-color: #1f4e78; color: white; text-align: center;">
      <th>Product Series</th>
      <th>Product</th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS2/tree/main" style="color: black; text-decoration: none;">Branch main</a></th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main" style="color: black; text-decoration: none;">Branch v2-main</a></th>
    </tr>
  </thead>
  <tbody>
      <tr>
      <td style="text-align: center; font-weight: bold;">Gemini 435Le</td>
      <td>Gemini 435Le</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="8" style="text-align: center; font-weight: bold;">Gemini 330</td>
      <td>Gemini 335Le</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 336</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 330</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 336L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 330L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335Lg</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="5" style="text-align: center; font-weight: bold;">Gemini 2</td>
      <td>Gemini 2</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 2 L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 2 XL</td>
      <td>recommended for new designs</td>
      <td>to be supported</td>
    </tr>
    <tr>
      <td>Gemini 215</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 210</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Femto</td>
      <td>Femto Bolt</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Femto Mega</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Femto Mega I</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Astra</td>
      <td>Astra 2</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Astra+</td>
      <td>limited maintenance</td>
      <td>not supported</td>
    </tr>
    <tr>
      <td>Astra Pro Plus</td>
      <td>limited maintenance</td>
      <td>not supported</td>
    </tr>
    <tr>
      <td style="text-align: center; font-weight: bold;">Astra Mini</td>
      <td>Astra Mini (S) Pro</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="2" style="text-align: center; font-weight: bold;">LiDAR</td>
      <td>Pulsar ME450</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Pulsar SL450</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
  </tbody>
</table>

**Note**: If you do not find your device, please contact our FAE or sales representative for help.

**Definition**:

1. Recommended for new designs: we will provide full supports with new features,  bug fix and performance optimization;
2. Full maintenance: we will provide bug fix support;
3. Limited maintenance: we will provide critical bug fix support;
4. Not supported: we will not support specific device in this version;
5. To be supported: we will add support in the near future.

## Support Hardware Products

The following devices are supported by the OrbbecSDK ROS2 Wrapper v2-main branch.  More devices support will be added in the near future. If you can not find your device in the table below, try the [main](https://github.com/orbbec/OrbbecSDK_ROS2)  branch.

For optimal performance, we strongly recommend updating to the latest firmware version. This ensures that you benefit from the most recent enhancements and bug fixes.

To learn how to obtain and upgrade the latest firmware, [please click here](../3_quickstarts/orbbecviewer.md).

| **Products List** | **Recommended FW Version**                                                             | Launch File                 |
| ----------------------- | -------------------------------------------------------------------------------------------- | --------------------------- |
| Astra Mini Pro          | [2.0.03](https://github.com/orbbec/OrbbecFirmware/releases/tag/Astra-Mini-Pro)                  | astra.launch.py             |
| Astra Mini S Pro        | [2.0.03](https://github.com/orbbec/OrbbecFirmware/releases/tag/Astra-Mini-S-Pro)                | astra.launch.py             |
| Gemini 435Le            | [1.3.6](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemin435Le-Firmware)              | gemini435_le.launch.py      |
| Gemini 330 series       | [1.6.00](https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.6.00.zip) | gemini_330_series.launch.py |
| Gemini 215              | [1.0.9](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini215-Firmware)               | gemini210.launch.py         |
| Gemini 210              | [1.0.9](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini210-Firmware)               | gemini210.launch.py         |
| Gemini 2                | [1.4.98](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini2-Firmware)                | gemini2.launch.py           |
| Gemini 2 L              | [1.5.2](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini2L-Firmware)                | gemini2L.launch.py          |
| Femto Bolt              | [1.1.2](https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Bolt-Firmware)              | femto_bolt.launch.py        |
| Femto Mega              | [1.3.1](https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Mega-Firmware)              | femto_mega.launch.py        |
| Femto Mega I            | [2.0.4](https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Mega-I-Firmware)            | femto_mega.launch.py        |
| Astra 2                 | [2.8.20](https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Astra2_Release_2.8.20.zip)    | astra2.launch.py            |
| Pulsar SL450            | 2.2.4.5 | lidar.launch.py |
| Pulsar ME450            | 1.0.0.6 | lidar.launch.py |

All launch files are essentially similar, with the primary difference being the default values of the parameters set
for different models within the same series. Differences in USB standards, such as USB 2.0 versus USB 3.0, may require adjustments to these parameters. If you encounter a startup failure, please carefully review the specification manual. Pay special attention to the resolution settings in the launch file, as well as other parameters, to ensure compatibility and optimal performance.

## Orbbec camera datasheet

Refer to the camera datasheet for more information.

<style>
table {
  border-collapse: collapse;
  width: 100%;
}
th, td {
  border: 1px solid #ccc;
  padding: 8px;
  text-align: left;
  vertical-align: middle;
}
thead th {
  background-color: #1f4e78;
  color: white;
  text-align: center;
  vertical-align: middle;
}
</style>

<table>
  <thead>
    <tr>
      <th>Product Series</th>
      <th>Product</th>
      <th>Datasheet</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="text-align: center;">Gemini 435Le</td>
      <td>Gemini 435Le</td>
      <td><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2025/06/04011158/Orbbec-Gemini-435Le-Datasheet-V1.pdf">Orbbec Gemini 435Le Datasheet</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="6">Gemini 330</td>
      <td>Gemini 335</td>
      <td rowspan="4"><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2025/04/22062452/Gemini-330-series-Datasheet-V1.6.pdf">Gemini 330 Series Datasheet for USB Devices</a></td>
    </tr>
    <tr><td>Gemini 336</td></tr>
    <tr><td>Gemini 335L</td></tr>
    <tr><td>Gemini 336L</td></tr>
    <tr>
      <td>Gemini 335Lg</td>
      <td><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2024/10/22030914/Gemini-335Lg-Datasheet-V1.0-241022.pdf">Gemini 330 Series Datasheet for GMSL Devices</a></td>
    </tr>
    <tr>
      <td>Gemini 335Le</td>
      <td><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2025/03/24023151/Orbbec-Gemini-335Le-Datasheet-V1-2.pdf">Gemini 330 Series Datasheet for Ethernet Devices</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="3">Gemini 2</td>
      <td>Gemini 2</td>
      <td rowspan="2"><a href="https://xm917ch2uk.feishu.cn/file/Khxfb2vdioUghexIMqJcAyL3nXf">Orbbec Gemini 2 Series Datasheet</a></td>
    </tr>
    <tr><td>Gemini 2 L</td></tr>
    <tr>
      <td>Gemini 2 XL</td>
      <td><a href="https://xm917ch2uk.feishu.cn/file/QW2vbNvwxoocRIxSL6Zcvut2npS">Orbbec Gemini 2 XL Datasheet</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="3">Femto</td>
      <td>Femto Bolt</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2024/08/ORBBEC_Datasheet_Femto-Bolt-v1.0.pdf">Orbbec Femto Bolt Datasheet</a></td>
    </tr>
    <tr>
      <td>Femto Mega</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/04/ORBBEC_Datasheet_Femto-Mega1.pdf">Orbbec Femto Mega Datasheet</a></td>
    </tr>
    <tr>
      <td>Femto Mega I</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/08/ORBBEC_Datasheet_Femto-Mega-I.pdf">Orbbec Femto Mega I Datasheet</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="3">Astra</td>
      <td>Astra 2</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/04/ORBBEC_Datasheet_Astra-2_V1.2.pdf">Orbbec Astra 2 Datasheet</a></td>
    </tr>
    <tr>
      <td>Astra+</td>
      <td><a href="https://xm917ch2uk.feishu.cn/file/Qk0zbx26Doh8XMxw0rIcOgQYnff">Orbbec Astra+ Datasheet</a></td>
    </tr>
    <tr>
      <td>Astra Mini Pro</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/04/ORBBEC_Datasheet_Astra-Mini-Pro-1.pdf">Orbbec Astra Mini Pro Datasheet</a></td>
    </tr>
  </tbody>
</table>

  ---

## Support Platforms

- Linux x64: tested on Ubuntu 22.04
- Linux ARM64: tested on NVIDIA Jetson AGX Orin , NVIDIA Jetson Orin NX , NVIDIA Jetson Orin Nano , NVIDIA Jetson AGX Xavier , NVIDIA Jetson Xavier NX
