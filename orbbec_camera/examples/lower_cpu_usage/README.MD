## Using the Orbbec ROS package to get lower CPU usage

This section shows you how to reduce CPU usage for one or more cameras in a ROS 2 environment. This method only works with Gemini 330 series cameras, and the firmware version needs to be above 1.4.10(`device_preset `is set to Default).

The example launch files used in this section are `gemini_330_series_lower_cpu_usage.launch.py` and `multi_camera_lower_cpu_usage.launch.py`.

### Parameters that affect CPU usage

* `uvc_backend`:Global UVC Backend select on Linux,optional values: libuvc, v4l2.(v4l2 is lower)
* `color_format`:Color stream coding format.(RGB is lower)

* `depth_registration`: Enables alignment of the depth frame to the color frame.(align_mode is set to HW lower)
* `enable_point_cloud`:Enables the point cloud

* `enable_colored_point_cloud`:Enables the RGB point cloud
* `filter`:decimation_filter,hdr_merge,sequenced_filter,threshold_filter,hardware_noise_removal_filter,noise_removal_filter,spatial_filter,temporal_filter,hole_filling_filter.(hardware_noise_removal_filter is lower)

### Data comparison

> Since different machines and different software environments will result in different CPU usage, the CPU improvement rate is calculated based on the device's CPU benchmark.
> A comparing B:
> `CPU improvement rate=（CPUA-CPUB）/CPUB*100`

In this example, we used two different video stream resolutions as test benchmarks to provide a performance comparison between lower and higher resolutions.(Currently, data is only provided for `uvc_backend`, `color_format`, and some `filter`)

#### Depth&Left_ir&Right_ir 424 * 266 15fps,Color 848 * 480 15fps

##### uvc_backend

libuvc comparing v4l2 In the same environment, CPU improvement rate is ***13.1%***

* libuvc is ***72.3%***
* v4l2 is ***62.8%***

##### color_format

ROS2 color stream sets MJPG encoding format and decodes it to RGB comparing RGB encoding format (including YUYV format to RGB conversion operation cost).The CPU improvement rate under the libuvc protocol is ***14.2%*** the CPU improvement rate under the v4l2 protocol is ***30.8%***

* libuvc:RGB encoding format is ***63.3%***，MJPG encoding format and decoded into RGB is ***72.3%***
* v4l2:RGB encoding format is ***48%***，MJPG encoding format and decoded into RGB is ***62.8%***

##### filter

The color stream setting RGB encoding and MJPG have no effect on filtering and are basically the same. The CPU usage of the depth stream is basically the same under the configuration of 424*266 15fps.

|             Filter Configuration             | libuvc(CPU improvement rate) | v4l2(CPU improvement rate) |
| :------------------------------------------: | :--------------------------: | :------------------------: |
|                  No filter                  |          63.3%(0%)          |         48.0%(0%)         |
|        hardware_noise_removal_filter        |         61.2%(-3.3%)         |        47.9%(-0.2%)        |
| hardware_noise_removal_filter+spatial_filter |         65.3%(3.2%)         |        51.2%(6.7%)        |
|             noise_removal_filter             |         77.0%(21.6%)         |        53.3%(11.0%)        |
|     noise_removal_filter+spatial_filter     |         78.4%(23.9%)         |        54.8%(14.2%)        |

#### Depth&Left_ir&Right_ir 848 * 480 30fps,Color 848 * 480 30fps

##### uvc_backend

libuvc comparing v4l2 In the same environment, CPU improvement rate is ***53.9%***

* libuvc is ***182.8%***
* v4l2 is ***118.8%***

##### color_format

ROS2 color stream sets MJPG encoding format and decodes it to RGB comparing RGB encoding format (including YUYV format to RGB conversion operation cost).The CPU improvement rate under the libuvc protocol is ***90.2%*** the CPU improvement rate under the v4l2 protocol is ***43.1%***

* libuvc:RGB encoding format is ***182.8%***，MJPG encoding format and decoded into RGB is ***347.7%***
* v4l2:RGB encoding format is ***118.8%***，MJPG encoding format and decoded into RGB is ***170.0%***

##### filter

When the depth stream is configured at 848*480 30fps, the CPU usage is significantly different.

|             Filter Configuration             | libuvc(CPU improvement rate) | v4l2(CPU improvement rate) |
| :------------------------------------------: | :--------------------------: | :------------------------: |
|                  No filter                  |          182.8%(0%)          |         118.8%(0%)         |
|        hardware_noise_removal_filter        |         186.3%(1.9%)         |       115.4%(-2.9%)       |
| hardware_noise_removal_filter+spatial_filter |        251.3%(37.5%)        |       152.5%(28.4%)       |
|             noise_removal_filter             |        218.0%(16.1%)        |        128.5%(8.2%)        |
|     noise_removal_filter+spatial_filter     |        469.6%(156.9%)        |       336.7%(183.4%)       |

### Example launch

This is the current example file for running the camera to achieve the lowest CPU usage, excluding the impact of filters.

Use the following command to start the single-camera configuration:

```bash
ros2 launch orbbec_camera gemini_330_series_lower_cpu_usage.launch.py
```

Use the following command to start the multi-camera configuration:

```bash
roslaunch orbbec_camera multi_camera_lower_cpu_usage.launch.py
```
