message("***************************************************")
message("*                                                 *")
message("*                 Find Orbbec SDK                 *")
message("*                                                 *")
message("***************************************************")


find_path(ORBBEC_SDK_INCLUDE_DIR "libobsensor/ObSensor.hpp" "/usr/local/include" "/usr/include")
message("include:\n${ORBBEC_SDK_INCLUDE_DIR}")

execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINES)
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE MACHINES_BIT)
MESSAGE(STATUS "ORRBEC Machine : ${MACHINES}")
MESSAGE(STATUS "ORRBEC Machine Bits : ${MACHINES_BIT}")
if ((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "x64")
elseif ((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "32"))
  set(HOST_PLATFORM "x86")
elseif (${MACHINES} MATCHES "x86")
elseif (${MACHINES} MATCHES "x86")
  set(HOST_PLATFORM "x86")
elseif (${MACHINES} MATCHES "i686")
  set(HOST_PLATFORM "x86")
elseif (${MACHINES} MATCHES "i386")
  set(HOST_PLATFORM "x86")
elseif (${MACHINES} MATCHES "arm")
  set(HOST_PLATFORM "arm")
elseif ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "arm64")
elseif ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "32"))
  set(HOST_PLATFORM "arm")
endif ()

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
