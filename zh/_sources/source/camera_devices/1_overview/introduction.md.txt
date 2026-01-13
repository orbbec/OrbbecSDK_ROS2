# 引言

OrbbecSDK ROS2 封装为 Orbbec 相机与 ROS 2 环境提供无缝集成。它支持 ROS2 Foxy、Humble 和 Jazzy 发行版。

默认推荐使用 **v2-main** 分支。对于 v2-main 尚不支持的旧 OpenNI 设备，请使用 **main** 分支。仅由 main 分支支持的设备型号列在下表中。

如果您是中国用户，推荐使用 [gitee 仓库](https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS2)。

下面是 main 分支 (v1.x) 与 v2-main 分支 (v2.x) 的设备支持列表：

<table border="1" style="border-collapse: collapse; text-align: left; width: 100%;">
  <thead>
    <tr style="background-color: #1f4e78; color: white; text-align: center;">
      <th>产品系列</th>
      <th>产品</th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS2/tree/main" style="color: black; text-decoration: none;">main 分支</a></th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main" style="color: black; text-decoration: none;">v2-main 分支</a></th>
    </tr>
  </thead>
  <tbody>
      <tr>
      <td rowspan="2" style="text-align: center; font-weight: bold;">Gemini 340</td>
      <td>Gemini 345</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 345Lg</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
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

**注意**: 如果未找到您的设备，请联系我们的 FAE 或销售代表获取帮助。

**术语定义**:

1. 建议用于新设计：我们将提供完整支持，包括新特性、缺陷修复与性能优化；
2. 完整维护：我们将提供缺陷修复支持；
3. 限制维护：我们仅提供关键缺陷修复支持；
4. 不支持：当前版本不支持该设备；
5. 即将支持：近期将添加支持。

## 支持的硬件产品

以下设备由 OrbbecSDK ROS2 封装 v2-main 分支支持。后续将增加更多设备支持。如果您在下表中未找到设备，请尝试 [main](https://github.com/orbbec/OrbbecSDK_ROS2) 分支。

为获得最佳性能，强烈建议升级到最新固件版本，以确保获得最新的改进与缺陷修复。

关于如何获取与升级最新固件，[请点击这里](../3_quickstarts/orbbecviewer.md)。

| **产品列表** | **推荐固件版本**                                                             |  **启动文件**                                          |
| ----------------------- | -------------------------------------------------------------------------------------------- | --------------------------------------------- |
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

所有启动文件本质上是类似的，主要区别在于同系列不同型号设置的参数默认值。USB 标准差异（如 USB 2.0 与 USB 3.0）可能需要调整这些参数。如果遇到启动失败，请仔细查看规格说明书，特别关注启动文件中的分辨率设置以及其他参数，以确保兼容与最佳性能。

## Orbbec 相机规格书

更多信息请参考相机规格书。

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
      <th>产品系列</th>
      <th>产品</th>
      <th>规格书</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="text-align: center;">Gemini 435Le</td>
      <td>Gemini 435Le</td>
      <td><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2025/06/04011158/Orbbec-Gemini-435Le-Datasheet-V1.pdf">Orbbec Gemini 435Le 规格书</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="6">Gemini 330</td>
      <td>Gemini 335</td>
      <td rowspan="4"><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2025/04/22062452/Gemini-330-series-Datasheet-V1.6.pdf">Gemini 330 系列 USB 设备规格书</a></td>
    </tr>
    <tr><td>Gemini 336</td></tr>
    <tr><td>Gemini 335L</td></tr>
    <tr><td>Gemini 336L</td></tr>
    <tr>
      <td>Gemini 335Lg</td>
      <td><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2024/10/22030914/Gemini-335Lg-Datasheet-V1.0-241022.pdf">Gemini 330 系列 GMSL 设备规格书</a></td>
    </tr>
    <tr>
      <td>Gemini 335Le</td>
      <td><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2025/03/24023151/Orbbec-Gemini-335Le-Datasheet-V1-2.pdf">Gemini 330 系列 以太网设备规格书</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="3">Gemini 2</td>
      <td>Gemini 2</td>
      <td rowspan="2"><a href="https://xm917ch2uk.feishu.cn/file/Khxfb2vdioUghexIMqJcAyL3nXf">Orbbec Gemini 2 系列规格书</a></td>
    </tr>
    <tr><td>Gemini 2 L</td></tr>
    <tr>
      <td>Gemini 2 XL</td>
      <td><a href="https://xm917ch2uk.feishu.cn/file/QW2vbNvwxoocRIxSL6Zcvut2npS">Orbbec Gemini 2 XL 规格书</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="3">Femto</td>
      <td>Femto Bolt</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2024/08/ORBBEC_Datasheet_Femto-Bolt-v1.0.pdf">Orbbec Femto Bolt 规格书</a></td>
    </tr>
    <tr>
      <td>Femto Mega</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/04/ORBBEC_Datasheet_Femto-Mega1.pdf">Orbbec Femto Mega 规格书</a></td>
    </tr>
    <tr>
      <td>Femto Mega I</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/08/ORBBEC_Datasheet_Femto-Mega-I.pdf">Orbbec Femto Mega I 规格书</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="3">Astra</td>
      <td>Astra 2</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/04/ORBBEC_Datasheet_Astra-2_V1.2.pdf">Orbbec Astra 2 规格书</a></td>
    </tr>
    <tr>
      <td>Astra+</td>
      <td><a href="https://xm917ch2uk.feishu.cn/file/Qk0zbx26Doh8XMxw0rIcOgQYnff">Orbbec Astra+ 规格书</a></td>
    </tr>
    <tr>
      <td>Astra Mini Pro</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/04/ORBBEC_Datasheet_Astra-Mini-Pro-1.pdf">Orbbec Astra Mini Pro 规格书</a></td>
    </tr>
  </tbody>
</table>

---

## 支持的平台

- Linux x64：已在 Ubuntu 22.04 上测试
- Linux ARM64：已在 NVIDIA Jetson AGX Orin、NVIDIA Jetson Orin NX、NVIDIA Jetson Orin Nano、NVIDIA Jetson AGX Xavier、NVIDIA Jetson Xavier NX 上测试
