# 其他工具

## Ob_benchmark 工具

> 此工具的目标是对各种 OrbbecSDK_ROS2 相机配置的性能进行基准测试。基准测试结果取决于使用的相机和设置。（目前仅适用于 ROS2 Humble）

您可以在 [example](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples) 中找到示例使用代码。

### 工具配置 ([start_benchmark_params.json](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/config/tools/startbenchmark/start_benchmark_params.json))

```json
{
    "start_benchmark_params": {
        "camera_name": [
            "camera_01",
            "camera_02",
            "camera_03",
            "camera_04"
        ],
        "process_name": "component_conta",
        "switch_cycle": 300,
        "test_cycle": 1,
        "skip_number": 30
    }
}
```

- `camera_name`：要配置的相机名称。例如：`"camera_01"`、`"camera_02"` 等。
- `process_name`：要监控的进程名称。例如，`"component_conta"` 将监控容器进程的数据。
- `switch_cycle`：切换配置的周期时间，以秒为单位。例如，设置为 `300` 意味着配置将每 300 秒切换一次。
- `test_cycle`：测试周期，以秒为单位。例如，设置为 `1` 意味着工具将每 1 秒收集一次监控进程的数据。
- `skip_number`：要跳过的数据点数量。例如，设置为 `30` 意味着将忽略前 30 个数据点。

### 相机配置（启动文件）

在 launch 文件夹中，有多个 .launch.py 文件（`ob_benchmark_0.launch.py`、`ob_benchmark_1.launch.py`、...、`ob_benchmark_19.launch.py`）。每个文件对应不同的相机配置。

### 运行 ob_benchmark 工具

要运行该工具，请使用以下命令：

```bash
source install/setup.bash
ros2 run orbbec_camera ob_benchmark_node
```

### 输出数据文件

输出数据文件将存储在 ob_benchmark 文件夹中，文件名如 `0.csv`、`1.csv`、...、19.csv。例如：

- `0.csv` 包含来自 `ob_benchmark_0.launch.py` 配置的数据。
- `1.csv` 包含来自 `ob_benchmark_1.launch.py` 配置的数据。
