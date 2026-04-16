#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pure_pursuit::pure_pursuit_lib" for configuration ""
set_property(TARGET pure_pursuit::pure_pursuit_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(pure_pursuit::pure_pursuit_lib PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libpure_pursuit_lib.dylib"
  IMPORTED_SONAME_NOCONFIG "@rpath/libpure_pursuit_lib.dylib"
  )

list(APPEND _cmake_import_check_targets pure_pursuit::pure_pursuit_lib )
list(APPEND _cmake_import_check_files_for_pure_pursuit::pure_pursuit_lib "${_IMPORT_PREFIX}/lib/libpure_pursuit_lib.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
