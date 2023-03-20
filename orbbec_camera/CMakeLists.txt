cmake_minimum_required(VERSION 3.8)
project(orbbec_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -fPIC -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC -g")
set(CMAKE_BUILD_TYPE "Debug")

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_publisher REQUIRED)
find_package(OpenCV REQUIRED)
find_package(orbbec_camera_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLOG REQUIRED libglog)

if (NOT GLOG_FOUND)
  message(FATAL_ERROR "glog is not found")
endif ()
execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINES)
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE MACHINES_BIT)
message(STATUS "ORRBEC Machine : ${MACHINES}")
message(STATUS "ORRBEC Machine Bits : ${MACHINES_BIT}")
if ((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "x64")
elseif (${MACHINES} MATCHES "arm")
  set(HOST_PLATFORM "arm")
elseif ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "arm64")
elseif ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "32"))
  set(HOST_PLATFORM "arm")
endif ()

set(ORBBEC_LIBS ${CMAKE_CURRENT_SOURCE_DIR}/SDK/lib/${HOST_PLATFORM})
set(ORBBEC_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/include/)

add_library(${PROJECT_NAME} SHARED
  src/d2c_viewer.cpp
  src/dynamic_params.cpp
  src/ob_camera_node_driver.cpp
  src/ob_camera_node.cpp
  src/ros_param_backend.cpp
  src/ros_service.cpp
  src/utils.cpp
  )

set(dependencies
  ament_index_cpp
  builtin_interfaces
  cv_bridge
  camera_info_manager
  Eigen3
  image_transport
  image_publisher
  orbbec_camera_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_msgs
  tf2_ros
  tf2_sensor_msgs
  )

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
  )

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ORBBEC_INCLUDE_DIR}
  ${OpenCV_INCLUDED_DIRS}
  ${GLOG_INCLUDED_DIRS}
  )


target_link_libraries(${PROJECT_NAME}
  ${ORBBEC_SDK_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${GLOG_LIBRARIES}
  -lOrbbecSDK
  -L${ORBBEC_LIBS}
  )

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  )

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ORBBEC_INCLUDE_DIR}
  ${OpenCV_INCLUDED_DIRS}
  ${GLOG_INCLUDED_DIRS}
  )


target_link_libraries(${PROJECT_NAME}_node
  ${ORBBEC_SDK_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${GLOG_LIBRARIES}
  -lOrbbecSDK
  -L${ORBBEC_LIBS}
  ${PROJECT_NAME}
  )

ament_target_dependencies(${PROJECT_NAME}_node
  ${dependencies}
  )
add_executable(list_devices_node
  src/list_devices_node.cpp
  )

target_include_directories(list_devices_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ORBBEC_INCLUDE_DIR}
  ${OpenCV_INCLUDED_DIRS}
  ${GLOG_INCLUDED_DIRS}
  )


target_link_libraries(list_devices_node
  ${ORBBEC_SDK_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${GLOG_LIBRARIES}
  -lOrbbecSDK
  -L${ORBBEC_LIBS}
  ${PROJECT_NAME}
  )

ament_target_dependencies(list_devices_node
  ${dependencies}
  )

add_executable(ob_cleanup_shm_node
  src/ob_cleanup_shm.cpp
  )

target_include_directories(ob_cleanup_shm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ORBBEC_INCLUDE_DIR}
  ${OpenCV_INCLUDED_DIRS}
  ${GLOG_INCLUDED_DIRS}
  )


target_link_libraries(ob_cleanup_shm_node
  ${ORBBEC_SDK_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${GLOG_LIBRARIES}
  -lOrbbecSDK
  -L${ORBBEC_LIBS}
  ${PROJECT_NAME}
  )

ament_target_dependencies(ob_cleanup_shm_node
  ${dependencies}
  )

add_executable(list_depth_work_mode_node
  src/list_depth_work_mode.cpp
  )

target_include_directories(list_depth_work_mode_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ORBBEC_INCLUDE_DIR}
  ${OpenCV_INCLUDED_DIRS}
  ${GLOG_INCLUDED_DIRS}
  )

target_link_libraries(list_depth_work_mode_node
  ${ORBBEC_SDK_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${GLOG_LIBRARIES}
  -lOrbbecSDK
  -L${ORBBEC_LIBS}
  ${PROJECT_NAME}
  )

ament_target_dependencies(list_depth_work_mode_node
  ${dependencies}
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  )

install(DIRECTORY
  ${ORBBEC_INCLUDE_DIR}
  DESTINATION include
  )

install(DIRECTORY
  ${ORBBEC_LIBS}/
  DESTINATION lib/
  FILES_MATCHING PATTERN "*.so"
  PATTERN "*.so.*"
  )


install(TARGETS list_devices_node
  ob_cleanup_shm_node
  list_depth_work_mode_node
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}/
  )
if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_export_include_directories(include ${ORBBEC_INCLUDE_DIR})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies} ${ORBBEC_LIBS})

ament_package()
