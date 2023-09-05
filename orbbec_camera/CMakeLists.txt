cmake_minimum_required(VERSION 3.8)
project(orbbec_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -fPIC -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC -g")
set(CMAKE_BUILD_TYPE "Release")
option(USE_RK_HW_DECODER "Use Rockchip hardware decoder" OFF)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror -Wpedantic)
endif ()

# find dependencies
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
)

foreach (dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach ()

find_package(PkgConfig REQUIRED)
pkg_search_module(GLOG REQUIRED libglog)

if (NOT GLOG_FOUND)
  message(FATAL_ERROR "glog is not found")
endif ()

if (USE_RK_HW_DECODER)
  pkg_search_module(RK_MPP REQUIRED rockchip_mpp)
  if (NOT RK_MPP_FOUND)
    message(FATAL_ERROR "rockchip_mpp is not found")
  endif ()
  pkg_search_module(RGA librga)
  if (NOT RGA_FOUND)
    add_definitions(-DUSE_LIBYUV)
    message("librga is not found, use libyuv instead")
  endif ()
endif ()


execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINES)
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE MACHINES_BIT)
message(STATUS "ORRBEC Machine : ${MACHINES}")
message(STATUS "ORRBEC Machine Bits : ${MACHINES_BIT}")

if ((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "x64")
elseif (${MACHINES} MATCHES "arm" OR (${MACHINES} MATCHES "aarch64" AND ${MACHINES_BIT} MATCHES "32"))
  set(HOST_PLATFORM "arm32")
elseif ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "arm64")
endif ()

set(ORBBEC_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/lib/${HOST_PLATFORM})
set(ORBBEC_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/include/)

set(CMAKE_BUILD_RPATH "${CMAKE_BUILD_RPATH}:${ORBBEC_LIBS_DIR}")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${ORBBEC_LIBS_DIR}")

set(COMMON_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ORBBEC_INCLUDE_DIR}
  ${OpenCV_INCLUDED_DIRS}
  ${GLOG_INCLUDED_DIRS}
)

set(COMMON_LIBRARIES
  ${ORBBEC_SDK_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${GLOG_LIBRARIES}
  -lOrbbecSDK
  -L${ORBBEC_LIBS_DIR}
  ${RK_MPP_LIBRARIES}
)

set(SOURCE_FILES
  src/d2c_viewer.cpp
  src/dynamic_params.cpp
  src/ob_camera_node_driver.cpp
  src/ob_camera_node.cpp
  src/ros_param_backend.cpp
  src/ros_service.cpp
  src/synced_imu_publisher.cpp
  src/utils.cpp
  src/mjpeg_decoder.cpp
)

if (USE_RK_HW_DECODER)
  add_definitions(-DUSE_RK_HW_DECODER)
  list(APPEND SOURCE_FILES src/rk_mpp_decoder.cpp)
  list(APPEND COMMON_INCLUDE_DIRS
    ${RK_MPP_INCLUDE_DIRS}
    ${RGA_INCLUDE_DIRS}
  )
  list(APPEND COMMON_LIBRARIES
    ${RGA_LIBRARIES}
    ${RK_MPP_LIBRARIES}
  )
  if (NOT RGA_FOUND)
    list(APPEND COMMON_LIBRARIES
      -lyuv
    )
  endif ()
endif ()

macro(add_orbbec_executable TARGET SOURCE)
  add_executable(${TARGET} ${SOURCE})
  target_include_directories(${TARGET} PUBLIC ${COMMON_INCLUDE_DIRS})
  target_link_libraries(${TARGET} ${COMMON_LIBRARIES} ${PROJECT_NAME})
  ament_target_dependencies(${TARGET} ${dependencies})
endmacro()

# Define library and nodes
add_library(${PROJECT_NAME} SHARED
  ${SOURCE_FILES}
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${COMMON_LIBRARIES})


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "orbbec_camera::OBCameraNodeDriver"
  EXECUTABLE orbbec_camera_node
)
# Add nodes using the macro
add_orbbec_executable(list_devices_node src/list_devices_node.cpp)
add_orbbec_executable(ob_cleanup_shm_node src/ob_cleanup_shm.cpp)
add_orbbec_executable(list_depth_work_mode_node src/list_depth_work_mode.cpp)
add_orbbec_executable(list_camera_profile_mode_node src/list_camera_profile.cpp)

# Install rules
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY ${ORBBEC_INCLUDE_DIR} DESTINATION include)
install(DIRECTORY ${ORBBEC_LIBS_DIR}/ DESTINATION lib/ FILES_MATCHING PATTERN "*.so*")
install(TARGETS list_devices_node
  ob_cleanup_shm_node
  list_depth_work_mode_node
  list_camera_profile_mode_node
  DESTINATION lib/${PROJECT_NAME}/)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_export_include_directories(include ${ORBBEC_INCLUDE_DIR})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies} ${ORBBEC_LIBS})

ament_package()