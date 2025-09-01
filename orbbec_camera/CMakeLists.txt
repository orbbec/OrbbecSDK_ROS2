cmake_minimum_required(VERSION 3.8)
project(orbbec_camera)

# 표준/빌드 옵션
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC -g3")
set(CMAKE_BUILD_TYPE "Release")

option(USE_RK_HW_DECODER "Use Rockchip hardware decoder" OFF)
option(USE_NV_HW_DECODER "Use Nvidia hardware decoder" OFF)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror -Wno-pedantic -Wno-array-bounds)
endif()

# ament & deps
find_package(ament_cmake REQUIRED)
find_package(gaemi_base_libs REQUIRED)

set(dependencies
  ament_cmake
  ament_index_cpp
  Eigen3
  builtin_interfaces
  cv_bridge
  camera_info_manager
  image_transport
  image_publisher
  OpenCV
  orbbec_camera_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_msgs
  tf2_ros
  tf2_sensor_msgs
  Threads
  diagnostic_updater
  diagnostic_msgs
  statistics_msgs
  backward_ros
  gaemi_base_libs
)
foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()
find_package(PkgConfig REQUIRED)

# HW decoder (선택)
if (USE_RK_HW_DECODER)
  pkg_search_module(RK_MPP REQUIRED rockchip_mpp)
  if (NOT RK_MPP_FOUND)
    message(FATAL_ERROR "rockchip_mpp is not found")
  endif()
  pkg_search_module(RGA librga)
  if (NOT RGA_FOUND)
    add_definitions(-DUSE_LIBYUV)
    message("librga is not found, use libyuv instead")
  endif()
endif()

# 플랫폼 감지
execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINES OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE MACHINES_BIT OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "ORRBEC Machine : ${MACHINES}")
message(STATUS "ORRBEC Machine Bits : ${MACHINES_BIT}")

if ((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "x64")
elseif (${MACHINES} MATCHES "arm" OR (${MACHINES} MATCHES "aarch64" AND ${MACHINES_BIT} MATCHES "32"))
  set(HOST_PLATFORM "arm32")
elseif ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "arm64")
else()
  message(FATAL_ERROR "Unsupported platform: ${MACHINES} ${MACHINES_BIT}")
endif()

# SDK 경로
set(ORBBEC_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/SDK)
set(ORBBEC_INCLUDE_DIR ${ORBBEC_SDK_PATH}/include)
set(ORBBEC_LIBS_DIR ${ORBBEC_SDK_PATH}/lib/${HOST_PLATFORM})

# RPATH: 설치 후 실행 시 같은 lib/에서 .so 탐색
# set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH
    "${CMAKE_INSTALL_PREFIX}/lib"    # /opt/ros/humble/lib
    "$ORIGIN/../"                    # 실행파일 기준 상위 디렉토리
)

# NV decoder (선택)
if (USE_NV_HW_DECODER)
  set(JETSON_MULTI_MEDIA_API_DIR /usr/src/jetson_multimedia_api)
  set(JETSON_MULTI_MEDIA_API_CLASS_DIR ${JETSON_MULTI_MEDIA_API_DIR}/samples/common/classes)
  set(JETSON_MULTI_MEDIA_API_INCLUDE_DIR ${JETSON_MULTI_MEDIA_API_DIR}/include/)
  set(LIBJPEG8B_INCLUDE_DIR ${JETSON_MULTI_MEDIA_API_INCLUDE_DIR}/libjpeg-8b)
  set(TEGRA_ARMABI /usr/lib/aarch64-linux-gnu/)
  add_definitions(-DUSE_NV_HW_DECODER)
  add_compile_options(-Wno-missing-field-initializers -Wno-unused-parameter)
  set(NV_LIBRARIES -lnvjpeg -lnvbufsurface -lnvbufsurftransform -lyuv -lv4l2
      -L${TEGRA_ARMABI} -L${TEGRA_ARMABI}/tegra)
endif()

# include 경로
set(COMMON_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${ORBBEC_INCLUDE_DIR}>
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tools>
)

# 소스
set(SOURCE_FILES
  # src/d2c_viewer.cpp
  # src/dynamic_params.cpp
  # src/image_publisher.cpp
  src/ob_camera_node.cpp
  # src/synced_imu_publisher.cpp
  src/utils.cpp
  src/jpeg_decoder.cpp
)
if (USE_RK_HW_DECODER)
  add_definitions(-DUSE_RK_HW_DECODER)
  list(APPEND SOURCE_FILES src/rk_mpp_decoder.cpp)
endif()
if (USE_NV_HW_DECODER)
  list(APPEND SOURCE_FILES
    src/jetson_nv_decoder.cpp
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

# 메인 라이브러리
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC ${COMMON_INCLUDE_DIRS})

# # 공통 라이브러리 링크 (SDK는 PRIVATE로 링크해 소비자에 경로 노출 방지)
# target_link_libraries(${PROJECT_NAME}
#   PUBLIC
#     ${OpenCV_LIBS}
#     Eigen3::Eigen
#     Threads::Threads
#     -lrt
#     -ldw
#   PRIVATE
#     "${ORBBEC_LIBS_DIR}/libOrbbecSDK.so"
# )

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  Threads::Threads
  -lrt
  -ldw
  "${ORBBEC_LIBS_DIR}/libOrbbecSDK.so"
)

# # SDK는 별도 비키워드 호출로 추가
# target_link_libraries(${PROJECT_NAME}
# )

# 컴포넌트 등록
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "orbbec_camera::OBCameraNodeDriver"
  EXECUTABLE orbbec_camera_node
)

# 도구/부가 노드
# function(add_orbbec_executable TARGET SOURCE)
#   add_executable(${TARGET} ${SOURCE})
#   target_include_directories(${TARGET} PUBLIC ${COMMON_INCLUDE_DIRS})
#   target_link_libraries(${TARGET} ${PROJECT_NAME})
#   ament_target_dependencies(${TARGET} ${dependencies})
# endfunction()
# add_orbbec_executable(list_depth_work_mode_node tools/list_depth_work_mode.cpp)
# add_orbbec_executable(topic_statistics_node tools/topic_statistics.cpp)

add_library(frame_latency SHARED tools/frame_latency.cpp)
target_include_directories(frame_latency PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(frame_latency ${PROJECT_NAME})
ament_target_dependencies(frame_latency ${dependencies})
rclcpp_components_register_node(frame_latency
  PLUGIN "orbbec_camera::FrameLatencyNode"
  EXECUTABLE frame_latency_node
)

# 설치
install(TARGETS ${PROJECT_NAME} frame_latency
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# SDK 헤더/라이브러리 동봉 설치
install(DIRECTORY ${ORBBEC_INCLUDE_DIR}/ DESTINATION include)
install(DIRECTORY ${ORBBEC_LIBS_DIR}/ DESTINATION lib/ FILES_MATCHING PATTERN "*.so*")

# 툴 실행파일 설치
# install(TARGETS
#   list_depth_work_mode_node
#   topic_statistics_node
#   DESTINATION lib/${PROJECT_NAME}/
# )

# 내보내기
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
