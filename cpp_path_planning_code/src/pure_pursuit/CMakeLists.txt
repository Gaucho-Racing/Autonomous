cmake_minimum_required(VERSION 3.8)
project(pure_pursuit)

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
find_package(ackermann_msgs REQUIRED)

# =============================================================================
# CORE LIBRARY (Algorithm only, no ROS dependencies)
# =============================================================================

add_library(pure_pursuit_lib SHARED
  src/pure_pursuit.cpp
)

target_include_directories(pure_pursuit_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# =============================================================================
# ROS 2 NODE
# =============================================================================

add_executable(controller_node
  src/controller_node.cpp
)

target_link_libraries(controller_node
  pure_pursuit_lib
)

ament_target_dependencies(controller_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  ackermann_msgs
)

# =============================================================================
# INSTALLATION
# =============================================================================

install(TARGETS pure_pursuit_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS controller_node
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
endif()

# =============================================================================
# EXPORT
# =============================================================================

ament_export_include_directories(include)
ament_export_libraries(pure_pursuit_lib)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
