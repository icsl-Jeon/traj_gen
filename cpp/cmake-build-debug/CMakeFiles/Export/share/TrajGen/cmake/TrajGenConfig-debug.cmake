#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "traj_gen" for configuration "Debug"
set_property(TARGET traj_gen APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(traj_gen PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_DEBUG "qpOASES"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libtraj_gen.so.2.0.0"
  IMPORTED_SONAME_DEBUG "libtraj_gen.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS traj_gen )
list(APPEND _IMPORT_CHECK_FILES_FOR_traj_gen "${_IMPORT_PREFIX}/lib/libtraj_gen.so.2.0.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
