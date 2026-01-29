#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ndi_aurora_node::ndi_api" for configuration ""
set_property(TARGET ndi_aurora_node::ndi_api APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ndi_aurora_node::ndi_api PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libndi_api.a"
  )

list(APPEND _cmake_import_check_targets ndi_aurora_node::ndi_api )
list(APPEND _cmake_import_check_files_for_ndi_aurora_node::ndi_api "${_IMPORT_PREFIX}/lib/libndi_api.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
