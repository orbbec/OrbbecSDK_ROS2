# Disparity_search_offset

> This section describes how to use the disparity_search_offset function in the Gemini330 series cameras (minimum camera firmware version [1.4.60](https://www.orbbec.com/docs/g330-firmware-release/)).Disparity_search_offset is effective only for1280×720, 1280×800 and 640×400 resolutions of depth stream.

## Function Introduction

The definition of disparity search range: For any pixel *(u_l, v)* in the left image, by default, the corresponding disparity search range in the right image is *[ (u_l - 255, v)*, *(u_l, v) ]*, where the disparity search length is 256 and the maximum integer disparity is 255. If the starting point of the search is adjusted to *[ (u_l - 255 - offset, v)*, *(u_l - offset, v) ]*, the offset is defined as the disparity shift. Therefore, our disparity search range configuration includes both the disparity search length and the search position offset (which can also be referred to as the disparity shift).

![Depth Point Cloud Visualization](../image/disparity_search_offset/search_offset0.png)

## Parameter Introduction

The disparity_search_offset related parameters are set in [gemini_330_series.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/launch/gemini_330_series.launch.py)

* `disparity_range_mode` : Disparity search length,can only be set to 64, 128 and 256.

* `disparity_search_offset` : Disparity search offset value,Disparity search offset value, can be set from 0 to 127.

* `disparity_offset_config` : Disparity search offset interleave frames.

* `offset_index0` : Frame 0 disparity search offset value.

* `offset_index1` : Frame 1 disparity search offset value.

| disparity range mode | disparity search offset | Minimum depth of inclined wall (mm) |
| :------------------: | :---------------------: | :---------------------------------: |
|          64          |           85           |        Gemini 335L  388-406        |
|          64          |           127           |        Gemini 335L  302-317        |

| disparity range mode | disparity search offset |       Minimum depth of inclined wall (mm)       |
| :------------------: | :---------------------: | :---------------------------------------------: |
|         128         |            0            | Gemini 335  233-249<br />Gemini 335L  453-475 |
|         128         |           45           | Gemini 335  172-184<br />Gemini 335L  334-349 |
|         128         |           127           | Gemini 335  117-125<br />Gemini 335L  226-236 |

| disparity range mode | disparity search offset | Minimum depth of inclined wall (mm) |
| :------------------: | :---------------------: | :---------------------------------: |
|         256         |           85           |        Gemini 335L  169-178        |
|         256         |           127           |        Gemini 335L  151-158        |

## Run the launch

Setting the disparity_search_offset parameter,`colcon build` again and run launch

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```
