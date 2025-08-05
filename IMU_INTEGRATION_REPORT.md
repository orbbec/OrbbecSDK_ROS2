# IMU集成测试报告

## 修改摘要
本次修改将原来分离的加速度计和陀螺仪控制整合成统一的IMU控制，主要修改包括：

### 1. Launch文件修改 (lidar.launch.py)
**之前的参数:**
- `enable_sync_output_accel_gyro`: 控制同步输出
- `enable_accel`: 控制加速度计
- `enable_gyro`: 控制陀螺仪
- `accel_rate`: 加速度计频率
- `gyro_rate`: 陀螺仪频率

**现在的参数:**
- `enable_imu`: 统一控制IMU(加速度计+陀螺仪)
- `imu_rate`: 统一的IMU频率
- `accel_range`: 加速度计量程
- `gyro_range`: 陀螺仪量程

### 2. 话题变化
**之前的话题:**
- `/camera/accel/sample`: 单独的加速度计数据
- `/camera/gyro/sample`: 单独的陀螺仪数据
- `/camera/gyro_accel/sample`: 同步的加速度计+陀螺仪数据

**现在的话题:**
- `/lidar/imu/sample`: 统一的IMU话题，包含同步的加速度计和陀螺仪数据

### 3. 外部参考话题变化
**之前的话题:**
- `/lidar/lidar_to_accel`: LiDAR到加速度计的外部参考
- `/lidar/lidar_to_gyro`: LiDAR到陀螺仪的外部参考

**现在的话题:**
- `/lidar/lidar_to_imu`: LiDAR到IMU的统一外部参考

### 4. 测试验证

#### 启动命令测试
```bash
# 新的启动命令
ros2 launch orbbec_camera lidar.launch.py enable_imu:=true imu_rate:=50hz

# 验证日志输出
# 应该看到: "Started IMU stream with accel range: 2g, gyro range: 1000dps, rate: 50hz"
```

#### 话题验证
```bash
# 检查IMU话题
ros2 topic list | grep -E "(imu|accel|gyro)"
# 预期输出:
# /lidar/imu/sample
# /lidar/lidar_to_imu

# 检查IMU数据
ros2 topic echo /lidar/imu/sample --once
# 预期: 包含angular_velocity和linear_acceleration的同步数据
```

### 5. 代码修改文件列表
- `orbbec_camera/launch/lidar.launch.py`
- `orbbec_camera/src/ob_lidar_node.cpp`
- `orbbec_camera/include/orbbec_camera/ob_lidar_node.h`
- `docs/launch_parameters.md`
- `docs/all_available_topics.md`

### 6. 修改优势
1. **简化配置**: 用户只需要设置一个`enable_imu`参数即可启用整个IMU功能
2. **统一频率**: 确保加速度计和陀螺仪使用相同的采样频率，提高数据同步性
3. **统一话题**: 所有IMU数据在一个话题中发布，便于下游节点使用
4. **减少复杂性**: 去除了复杂的同步选项，默认就是同步的
5. **更好的语义**: IMU作为一个整体概念更符合机器人领域的常见用法

### 7. 向后兼容性
此修改不向后兼容，使用旧参数的launch文件需要更新参数名称。但这是一个有意的设计决策，旨在简化和改进用户体验。
