# Predefined presets

| Preset         | Features                                                                                                                                                                                                           | Recommended use cases                                                                                                                                              |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Default        | - Best visual perception``- Overall good performance in accuracy, fill rate, tiny objects, etc.                                                                                                                    | - Generic `<br>`- Robotics                                                                                                                                       |
| Hand           | - Clear hand and finger edges                                                                                                                                                                                      | - Gesture recognition                                                                                                                                              |
| High Accuracy  | - Depth of high confidence `<br>`- Barely noise depth values `<br>`- Lower fill rate                                                                                                                           | - Collision avoidance `<br>`- Object scanning                                                                                                                    |
| High Density   | - Higher fill rate `<br>`- More tiny objects `<br>`- May suffer from noise depth values                                                                                                                        | - Object recognition `<br>`- Pick & place `<br>`- Foreground & background animation                                                                            |
| Medium Density | - Balanced performance in fill rate and accuracy `<br>`- In comparison to Default: lower fill rate, better edge quality                                                                                          | - Generic and alternative to Default                                                                                                                               |
| Custom         | - User defined Preset `<br>`- Derived from Presets above, with customized modifications, e.g. a new configuration for the post-processing pipeline, modified mean intensity set point of depth AE function, etc. | - Better depth performance achieved using customized configurations in comparison to using predefined presets `<br>`- For well-established custom configurations |

Choose the appropriate preset name based on your specific use case and set it as the value for the `device_preset`
parameter.

## Optional depth preset

> You can pass the firmware path of the Optional preset into the `preset_firmware_path` launch param

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py preset_firmware_path:=/home/orbbec/G336X_Dimensioning_Accurate_0.0.1_78C743B9.bin,/home/orbbec/G336X_Dimensioning_Dense_0.0.1_4E70D227.bin device_preset:=G336X Dimensioning Dense
```
