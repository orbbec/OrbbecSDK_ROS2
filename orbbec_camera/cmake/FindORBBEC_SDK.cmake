message("***************************************************")
message("*                                                 *")
message("*                 Find Orbbec SDK                 *")
message("*                                                 *")
message("***************************************************")


find_path(ORBBEC_SDK_INCLUDE_DIR "libobsensor/ObSensor.hpp" "/usr/local/include" "/usr/include")
message("include:\n${ORBBEC_SDK_INCLUDE_DIR}")

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" TARGET_PROCESSOR)
message(STATUS "ORRBEC Target processor: ${TARGET_PROCESSOR}")
if(TARGET_PROCESSOR MATCHES "^(x86_64|amd64)$")
  set(HOST_PLATFORM "x64")
elseif(TARGET_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(HOST_PLATFORM "arm64")
elseif(TARGET_PROCESSOR MATCHES "^(arm|armv[5-8].*)$")
  message(FATAL_ERROR "ORBBEC SDK does not support arm32")
else()
  message(FATAL_ERROR
    "Unsupported Orbbec target architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

message(STATUS "ORRBEC : ${HOST_PLATFORM}")

set(ORBBEC_LIB_PATH "/usr/local/lib" "/usr/lib")

message("Orbbec lib path: ${ORBBEC_LIB_PATH}")

find_library(ORBBEC_SDK_LIBRARY NAMES OrbbecSDK PATHS ${ORBBEC_LIB_PATH} REQUIRED)
find_library(POSTFILTER_LIBRARY NAMES postfilter PATHS ${ORBBEC_LIB_PATH} REQUIRED)

message("OrbbecSDK path: ${ORBBEC_SDK_LIBRARY}")
message("postfilter path: ${POSTFILTER_LIBRARY}")

set(ORBBEC_SDK_LIBRARIES ${ORBBEC_SDK_LIBRARY} ${POSTFILTER_LIBRARY})
message("libraries:\n${ORBBEC_SDK_LIBRARIES}")

set(ORBBEC_SDK_FOUND TRUE)
