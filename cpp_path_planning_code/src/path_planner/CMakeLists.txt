cmake_minimum_required(VERSION 3.8)
project(path_planner)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# =============================================================================
# CORE LIBRARY (Algorithm only, no ROS dependencies)
# =============================================================================

add_library(delaunay_planner SHARED
  src/delaunay_planner.cpp
)

target_include_directories(delaunay_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# =============================================================================
# ROS 2 NODE
# =============================================================================

add_executable(planner_node
  src/planner_node.cpp
)

target_link_libraries(planner_node
  delaunay_planner
)

ament_target_dependencies(planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
)

# =============================================================================
# INSTALLATION
# =============================================================================

install(TARGETS delaunay_planner
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# =============================================================================
# TESTING
# =============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Add unit tests here
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(test_planner test/test_planner.cpp)
  # target_link_libraries(test_planner delaunay_planner)
endif()

# =============================================================================
# EXPORT
# =============================================================================

ament_export_include_directories(include)
ament_export_libraries(delaunay_planner)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
