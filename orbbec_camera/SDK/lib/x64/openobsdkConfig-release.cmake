#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ob::openobsdk" for configuration "Release"
set_property(TARGET ob::openobsdk APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ob::openobsdk PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libopenobsdk.so"
  IMPORTED_SONAME_RELEASE "libopenobsdk.so"
  )

list(APPEND _cmake_import_check_targets ob::openobsdk )
list(APPEND _cmake_import_check_files_for_ob::openobsdk "${_IMPORT_PREFIX}/lib/libopenobsdk.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
