cmake_minimum_required(VERSION 3.13)
project(orbbec_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -fPIC -g3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC -g3")
option(USE_RK_HW_DECODER "Use Rockchip hardware decoder" OFF)
option(USE_NV_HW_DECODER "Use Nvidia hardware decoder" OFF)
option(INSTALL_UDEV_RULES "Install udev rule for Orbbec cameras" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Werror -Wno-pedantic -Wno-array-bounds)
    add_compile_options(-Wno-error=deprecated-declarations)
    add_link_options("-Wl,-z,relro,-z,now,-z,defs")
endif()

# find dependencies
set(dependencies
    ament_cmake
    ament_index_cpp
    Eigen3
    builtin_interfaces
    cv_bridge
    backward_ros
    camera_info_manager
    geometry_msgs
    image_transport
    image_publisher
    message_filters
    OpenCV
    orbbec_camera_msgs
    rcl_interfaces
    rclcpp
    rclcpp_components
    rmw
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_msgs
    tf2_ros
    tf2_sensor_msgs
    Threads
    diagnostic_updater
    diagnostic_msgs
    statistics_msgs
    yaml-cpp
    camera_calibration_parsers
)

foreach(dep IN LISTS dependencies)
    find_package(${dep} REQUIRED)
endforeach()

find_package(PkgConfig REQUIRED)
find_package(OpenSSL REQUIRED)

set(ORBBEC_MESSAGE_FILTERS_USES_RCLCPP_QOS OFF)
if(DEFINED message_filters_VERSION AND message_filters_VERSION VERSION_GREATER_EQUAL 5.0)
    set(ORBBEC_MESSAGE_FILTERS_USES_RCLCPP_QOS ON)
endif()

set(ORBBEC_IMAGE_TRANSPORT_USES_REQUIRED_INTERFACES OFF)
if(DEFINED image_transport_VERSION AND image_transport_VERSION VERSION_GREATER_EQUAL 6.4)
    set(ORBBEC_IMAGE_TRANSPORT_USES_REQUIRED_INTERFACES ON)
endif()

set(ORBBEC_CAMERA_INFO_MANAGER_USES_NODE_INTERFACES OFF)
if(DEFINED camera_info_manager_VERSION AND
   camera_info_manager_VERSION VERSION_GREATER_EQUAL 4.1.0)
    set(ORBBEC_CAMERA_INFO_MANAGER_USES_NODE_INTERFACES ON)
endif()

set(ORBBEC_CAMERA_INFO_MANAGER_USES_RCLCPP_QOS OFF)
if(DEFINED camera_info_manager_VERSION AND
   camera_info_manager_VERSION VERSION_GREATER_EQUAL 7.0.0)
    set(ORBBEC_CAMERA_INFO_MANAGER_USES_RCLCPP_QOS ON)
endif()

function(orbbec_target_dependencies target)
    if(COMMAND ament_target_dependencies)
        ament_target_dependencies(${target} ${ARGN})
        return()
    endif()

    set(dependency_targets)
    foreach(dependency IN LISTS ARGN)
        if(dependency STREQUAL "rclcpp_components")
            list(APPEND dependency_targets rclcpp_components::component)
        elseif(dependency STREQUAL "sensor_msgs")
            list(APPEND dependency_targets sensor_msgs::sensor_msgs_library)
        elseif(TARGET "${dependency}::${dependency}")
            list(APPEND dependency_targets "${dependency}::${dependency}")
        else()
            message(FATAL_ERROR "No CMake target exported by ${dependency}")
        endif()
    endforeach()
    target_link_libraries(${target} ${dependency_targets})
endfunction()

if(USE_RK_HW_DECODER)
    pkg_search_module(RK_MPP REQUIRED rockchip_mpp)
    if(NOT RK_MPP_FOUND)
        message(FATAL_ERROR "rockchip_mpp is not found")
    endif()
    pkg_search_module(RGA librga)
    if(NOT RGA_FOUND)
        add_definitions(-DUSE_LIBYUV)
        message("librga is not found, use libyuv instead")
    endif()
endif()

execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINES)
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE MACHINES_BIT)
message(STATUS "ORRBEC Machine : ${MACHINES}")
message(STATUS "ORRBEC Machine Bits : ${MACHINES_BIT}")

if((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64"))
    set(HOST_PLATFORM "x64")
elseif(${MACHINES} MATCHES "arm" OR (${MACHINES} MATCHES "aarch64" AND ${MACHINES_BIT} MATCHES "32"))
    message(FATAL_ERROR "ORBBEC SDK does not support arm32")
    set(HOST_PLATFORM "arm32")
elseif((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64"))
    set(HOST_PLATFORM "arm64")
endif()

set(ORBBEC_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/lib/${HOST_PLATFORM})
set(ORBBEC_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/include/)

set(CMAKE_BUILD_RPATH "${CMAKE_BUILD_RPATH}:${ORBBEC_LIBS_DIR}")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${ORBBEC_LIBS_DIR}")
if(USE_NV_HW_DECODER)
    set(JETSON_MULTI_MEDIA_API_DIR /usr/src/jetson_multimedia_api)
    set(JETSON_MULTI_MEDIA_API_CLASS_DIR ${JETSON_MULTI_MEDIA_API_DIR}/samples/common/classes)
    set(JETSON_MULTI_MEDIA_API_INCLUDE_DIR ${JETSON_MULTI_MEDIA_API_DIR}/include/)
    set(LIBJPEG8B_INCLUDE_DIR ${JETSON_MULTI_MEDIA_API_INCLUDE_DIR}/libjpeg-8b)
    set(TEGRA_ARMABI /usr/lib/aarch64-linux-gnu/)
    add_definitions(-DUSE_NV_HW_DECODER)
    add_compile_options(-Wno-missing-field-initializers -Wno-unused-parameter)
    set(NV_LIBRARIES -lnvjpeg -lnvbufsurface -lnvbufsurftransform -lyuv -lv4l2)
    list(APPEND NV_LIBRARIES -L${TEGRA_ARMABI} -L${TEGRA_ARMABI}/tegra)
endif()

set(COMMON_INCLUDE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> ${ORBBEC_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/tools
)

set(EXTRA_TARGETS
)

set(SOURCE_FILES
  src/d2c_viewer.cpp
  src/dynamic_params.cpp
  src/image_publisher.cpp
  src/frame_timestamp_csv_logger.cpp
  src/ob_camera_node_driver.cpp
  src/ob_camera_node.cpp
  src/ob_lidar_node.cpp
  src/ros_param_backend.cpp
  src/ros_service.cpp
  src/synced_imu_publisher.cpp
  src/utils.cpp
  src/jpeg_decoder.cpp
)

if(USE_RK_HW_DECODER)
    add_definitions(-DUSE_RK_HW_DECODER)
    list(APPEND SOURCE_FILES src/rk_mpp_decoder.cpp)
    list(APPEND COMMON_INCLUDE_DIRS ${RK_MPP_INCLUDE_DIRS} ${RGA_INCLUDE_DIRS})
    list(APPEND EXTRA_TARGETS ${RGA_LIBRARIES} ${RK_MPP_LIBRARIES})
    if(NOT RGA_FOUND)
        list(APPEND EXTRA_TARGETS yuv)
    endif()
endif()

if(USE_NV_HW_DECODER)
    list(APPEND SOURCE_FILES src/jetson_nv_decoder.cpp)
    list(APPEND COMMON_INCLUDE_DIRS ${JETSON_MULTI_MEDIA_API_INCLUDE_DIR} ${LIBJPEG8B_INCLUDE_DIR})
    list(APPEND EXTRA_TARGETS ${NV_LIBRARIES})
    # append jetson_multimedia_api source files
    list(
        APPEND
        SOURCE_FILES
        ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvBuffer.cpp
        ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvElement.cpp
        ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvElementProfiler.cpp
        ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvJpegDecoder.cpp
        ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvJpegEncoder.cpp
        ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvLogging.cpp
        ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvUtils.cpp
        ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvV4l2Element.cpp
        ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvV4l2ElementPlane.cpp
        ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvVideoDecoder.cpp
        ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvVideoEncoder.cpp
        ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvBufSurface.cpp
    )
endif()

macro(add_orbbec_executable TARGET SOURCE)
    add_executable(${TARGET} ${SOURCE})
    target_include_directories(${TARGET} PUBLIC ${COMMON_INCLUDE_DIRS})
    target_link_directories(${TARGET} PRIVATE
        ${ORBBEC_LIBS_DIR}
    )
    target_link_libraries(${TARGET}
        ${OpenCV_LIBS}
        ${PROJECT_NAME}
        OrbbecSDK
        yaml-cpp
    )
endmacro()

# Define library and nodes
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
if(ORBBEC_MESSAGE_FILTERS_USES_RCLCPP_QOS)
    target_compile_definitions(${PROJECT_NAME} PRIVATE ORBBEC_MESSAGE_FILTERS_USES_RCLCPP_QOS)
endif()
if(ORBBEC_IMAGE_TRANSPORT_USES_REQUIRED_INTERFACES)
    target_compile_definitions(${PROJECT_NAME} PRIVATE ORBBEC_IMAGE_TRANSPORT_USES_REQUIRED_INTERFACES)
endif()
if(ORBBEC_CAMERA_INFO_MANAGER_USES_NODE_INTERFACES)
    target_compile_definitions(${PROJECT_NAME}
        PRIVATE ORBBEC_CAMERA_INFO_MANAGER_USES_NODE_INTERFACES)
endif()
if(ORBBEC_CAMERA_INFO_MANAGER_USES_RCLCPP_QOS)
    target_compile_definitions(${PROJECT_NAME}
        PRIVATE ORBBEC_CAMERA_INFO_MANAGER_USES_RCLCPP_QOS)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC ${COMMON_INCLUDE_DIRS} ${image_publisher_INCLUDE_DIRS})
target_link_directories(${PROJECT_NAME} PRIVATE ${ORBBEC_LIBS_DIR})
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${orbbec_camera_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ament_index_cpp::ament_index_cpp
    Eigen3::Eigen
    OpenSSL::Crypto
    OrbbecSDK
    Threads::Threads
    ${BACKWARD_LIBRARIES}
    rclcpp::rclcpp
    rt
    tf2::tf2
    tf2_ros::tf2_ros
    ${EXTRA_TARGETS}
)
orbbec_target_dependencies(${PROJECT_NAME}
    camera_info_manager
    cv_bridge
    diagnostic_updater
    geometry_msgs
    image_transport
    message_filters
    rcl_interfaces
    rclcpp_components
    rmw
    sensor_msgs
)

rclcpp_components_register_node(
    ${PROJECT_NAME} PLUGIN "orbbec_camera::OBCameraNodeDriver" EXECUTABLE orbbec_camera_node
)
# Add nodes using the macro
add_orbbec_executable(list_devices_node tools/list_devices_node.cpp)
add_orbbec_executable(list_depth_work_mode_node tools/list_depth_work_mode.cpp)
add_orbbec_executable(list_camera_profile_mode_node tools/list_camera_profile.cpp)
add_orbbec_executable(firmware_update_tool tools/firmware_update_tool.cpp)
add_orbbec_executable(ip_config_tool tools/ip_config_tool.cpp)
add_orbbec_executable(set_device_ip tools/ip_config_tool.cpp)
add_orbbec_executable(topic_statistics_node tools/topic_statistics.cpp)
add_orbbec_executable(ob_benchmark_node tools/ob_benchmark.cpp)
add_orbbec_executable(435le_example_node examples/Gemini_435Le_example_node/camera_example_node.cpp)
add_orbbec_executable(service_benchmark_node scripts/service_benchmark_node.cpp)
add_orbbec_executable(image_sync_example_node examples/multi_camera_time_sync/image_sync_example_node.cpp)
orbbec_target_dependencies(topic_statistics_node statistics_msgs)
if(ORBBEC_MESSAGE_FILTERS_USES_RCLCPP_QOS)
    target_compile_definitions(image_sync_example_node PRIVATE ORBBEC_MESSAGE_FILTERS_USES_RCLCPP_QOS)
endif()

install(
  PROGRAMS
    scripts/common_benchmark_node.py
    scripts/service_benchmark_node.py
  DESTINATION lib/${PROJECT_NAME}
)

add_library(frame_latency SHARED tools/frame_latency.cpp)
target_include_directories(frame_latency PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(frame_latency
    ${orbbec_camera_msgs_TARGETS}
    ${tf2_msgs_TARGETS}
    rclcpp::rclcpp
)
orbbec_target_dependencies(frame_latency
    diagnostic_updater
    geometry_msgs
    rclcpp_components
    sensor_msgs
)

rclcpp_components_register_node(frame_latency PLUGIN "orbbec_camera::FrameLatencyNode" EXECUTABLE frame_latency_node)

add_library(start_benchmark SHARED tools/start_benchmark.cpp)
target_include_directories(start_benchmark PUBLIC ${COMMON_INCLUDE_DIRS} ${image_publisher_INCLUDE_DIRS})
target_link_libraries(start_benchmark
    ${OpenCV_LIBS}
    ${orbbec_camera_msgs_TARGETS}
    ${std_srvs_TARGETS}
    Eigen3::Eigen
    rclcpp::rclcpp
    tf2_ros::tf2_ros
)
orbbec_target_dependencies(start_benchmark
    camera_info_manager
    diagnostic_updater
    image_transport
    rclcpp_components
    sensor_msgs
)

rclcpp_components_register_node(
    start_benchmark PLUGIN "orbbec_camera::tools::StartBenchmark" EXECUTABLE start_benchmark_node
)

add_library(multi_save_rgbir SHARED tools/multi_save_rgbir.cpp src/utils.cpp)
target_include_directories(multi_save_rgbir PUBLIC ${COMMON_INCLUDE_DIRS} ${image_publisher_INCLUDE_DIRS})
target_link_directories(multi_save_rgbir PRIVATE ${ORBBEC_LIBS_DIR})
target_link_libraries(multi_save_rgbir
    ${OpenCV_LIBS}
    ${orbbec_camera_msgs_TARGETS}
    ${std_srvs_TARGETS}
    Eigen3::Eigen
    OpenSSL::Crypto
    OrbbecSDK
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros
)
orbbec_target_dependencies(multi_save_rgbir
    camera_info_manager
    cv_bridge
    diagnostic_updater
    image_transport
    rclcpp_components
    sensor_msgs
)

rclcpp_components_register_node(
    multi_save_rgbir PLUGIN "orbbec_camera::tools::MultiCameraSubscriber" EXECUTABLE multi_save_rgbir_node
)

# Install rules
install(TARGETS ${PROJECT_NAME} frame_latency start_benchmark multi_save_rgbir ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib RUNTIME DESTINATION bin
)

install(DIRECTORY include/orbbec_camera DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY examples DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY SDK/licenses/ DESTINATION share/${PROJECT_NAME}/SDK/licenses)
install(DIRECTORY ${ORBBEC_INCLUDE_DIR} DESTINATION include)
install(DIRECTORY ${ORBBEC_LIBS_DIR}/ DESTINATION lib/ FILES_MATCHING PATTERN "*.so*")
# Install extensions directory with explicit file copying to avoid symlink issues
file(GLOB_RECURSE EXTENSION_FILES "${ORBBEC_LIBS_DIR}/extensions/*")
if(EXTENSION_FILES)
    install(FILES ${EXTENSION_FILES} DESTINATION lib/extensions/)
endif()
if(INSTALL_UDEV_RULES)
    # Install udev rule into package share so user can manually copy with sudo if desired
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/99-obsensor-libusb.rules
            DESTINATION share/${PROJECT_NAME}/udev)
endif()
if(DEFINED ENV{BUILDING_PACKAGE})
    # Install udev rules
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/99-obsensor-libusb.rules DESTINATION /etc/udev/rules.d)
endif()

install(TARGETS list_devices_node list_depth_work_mode_node list_camera_profile_mode_node firmware_update_tool topic_statistics_node  service_benchmark_node ob_benchmark_node 435le_example_node ip_config_tool set_device_ip image_sync_example_node DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies} ${ORBBEC_LIBS})

ament_package()
