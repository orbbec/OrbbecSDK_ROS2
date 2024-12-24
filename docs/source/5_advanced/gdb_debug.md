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

Take orbbec_camera.launch.py as an example to use xterm terminal to open gdb

```python
    def create_composable_node(camera_name, params, use_intra_process):
        common_arguments = [{'use_intra_process_comms': use_intra_process}]
        composable_node = ComposableNode(
                namespace=camera_name,
                name=camera_name,
                package=default_package_name,
                plugin='orbbec_camera::OBCameraNodeDriver',
                parameters=params,
                #extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("use_intra_process_comms")}],
                extra_arguments=common_arguments,
                prefix=['xterm -e gdb -ex run --args'],
            )
        return composable_node
```
