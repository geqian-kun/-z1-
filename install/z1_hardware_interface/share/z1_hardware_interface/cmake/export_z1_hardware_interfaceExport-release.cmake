#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "z1_hardware_interface::z1_hardware_interface" for configuration "Release"
set_property(TARGET z1_hardware_interface::z1_hardware_interface APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(z1_hardware_interface::z1_hardware_interface PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "fmt::fmt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libz1_hardware_interface.so"
  IMPORTED_SONAME_RELEASE "libz1_hardware_interface.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS z1_hardware_interface::z1_hardware_interface )
list(APPEND _IMPORT_CHECK_FILES_FOR_z1_hardware_interface::z1_hardware_interface "${_IMPORT_PREFIX}/lib/libz1_hardware_interface.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
