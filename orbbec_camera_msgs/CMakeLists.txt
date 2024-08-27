cmake_minimum_required(VERSION 3.8)
project(orbbec_camera_msgs)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeviceInfo.msg"
  "msg/Extrinsics.msg"
  "msg/Metadata.msg"
  "msg/IMUInfo.msg"
  "msg/RGBD.msg"
  "srv/GetBool.srv"
  "srv/GetDeviceInfo.srv"
  "srv/GetCameraInfo.srv"
  "srv/GetInt32.srv"
  "srv/GetString.srv"
  "srv/SetInt32.srv"
  "srv/SetString.srv"
  DEPENDENCIES
  sensor_msgs
  std_msgs)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
