cmake_minimum_required(VERSION 3.8)
project(orbbec_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_BUILD_TYPE "Debug")

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-variable -Wno-unused-parameter -Werror)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(glog REQUIRED)
find_package(image_transport REQUIRED)
find_package(magic_enum REQUIRED)
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


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(ORBBEC_SDK REQUIRED)

if (NOT ORBBEC_SDK_FOUND)
  message("Could not find Orbbec SDK!")
endif ()

add_library(${PROJECT_NAME} SHARED
  src/dynamic_params.cpp
  src/ob_camera_node_factory.cpp
  src/ob_camera_node.cpp
  src/ros_param_backend.cpp
  src/ros_service.cpp
  src/utils.cpp
  )

set(dependencies
  ament_index_cpp
  builtin_interfaces
  cv_bridge
  Eigen3
  image_transport
  orbbec_camera_msgs
  ORBBEC_SDK
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
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "orbbec_camera::OBCameraNodeFactory"
  EXECUTABLE orbbec_camera_node
  )
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ORBBEC_SDK_INCLUDE_DIR}
  ${OpenCV_INCLUDED_DIRS}
  )


target_link_libraries(${PROJECT_NAME}
  ${ORBBEC_SDK_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  glog::glog
  magic_enum::magic_enum
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
  params
  DESTINATION share/${PROJECT_NAME}/
  )

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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
