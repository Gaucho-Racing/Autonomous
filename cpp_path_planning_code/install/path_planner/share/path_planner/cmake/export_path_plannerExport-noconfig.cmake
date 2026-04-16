#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "path_planner::delaunay_planner" for configuration ""
set_property(TARGET path_planner::delaunay_planner APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(path_planner::delaunay_planner PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdelaunay_planner.dylib"
  IMPORTED_SONAME_NOCONFIG "@rpath/libdelaunay_planner.dylib"
  )

list(APPEND _cmake_import_check_targets path_planner::delaunay_planner )
list(APPEND _cmake_import_check_files_for_path_planner::delaunay_planner "${_IMPORT_PREFIX}/lib/libdelaunay_planner.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
