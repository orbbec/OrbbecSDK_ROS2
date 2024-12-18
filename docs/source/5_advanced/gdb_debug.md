# GDB debug

Debugging ROS 2 programs with GDB involves several steps:

## Config debug

Set `CMAKE_BUILD_TYPE` to `Debug`  in ` orbbec_camera/CMakeLists.txt`

```
set(CMAKE_BUILD_TYPE Debug)
```

## Use xterm terminal to open gdb debugging

Install xterm

```bash
sudo apt install xterm
```

Take gemini_330_series.launch.py as an example to use xterm terminal to open gdb

![Multi_camera1](../image/gdb_1.png)
