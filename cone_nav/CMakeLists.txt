cmake_minimum_required(VERSION 3.8)
project(cone_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDAToolkit REQUIRED)

find_path(NVINFER_INCLUDE_DIR NvInfer.h
  HINTS
    /usr/include
    /usr/include/aarch64-linux-gnu
    /usr/include/x86_64-linux-gnu
    /usr/local/cuda/include
)
find_library(NVINFER_LIBRARY nvinfer
  HINTS
    /usr/lib
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/cuda/lib64
)
find_library(NVINFER_PLUGIN_LIBRARY nvinfer_plugin
  HINTS
    /usr/lib
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/cuda/lib64
)

if(NOT NVINFER_INCLUDE_DIR OR NOT NVINFER_LIBRARY OR NOT NVINFER_PLUGIN_LIBRARY)
  message(FATAL_ERROR "TensorRT headers/libraries were not found. Install TensorRT nvinfer and nvinfer_plugin.")
endif()

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${NVINFER_INCLUDE_DIR}
)

add_executable(cone_detector_node src/cone_detector_node.cpp)
ament_target_dependencies(cone_detector_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  vision_msgs
  cv_bridge
  image_transport
)
target_link_libraries(cone_detector_node
  ${OpenCV_LIBS}
  CUDA::cudart
  ${NVINFER_LIBRARY}
  ${NVINFER_PLUGIN_LIBRARY}
)

add_executable(cone_localizer_node src/cone_localizer_node.cpp)
ament_target_dependencies(cone_localizer_node
  rclcpp
  sensor_msgs
  vision_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
)
target_link_libraries(cone_localizer_node ${OpenCV_LIBS})

add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
ament_target_dependencies(pure_pursuit_node
  rclcpp
  nav_msgs
  ackermann_msgs
)

add_executable(depth_obstacle_node src/depth_obstacle_node.cpp)
ament_target_dependencies(depth_obstacle_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(depth_obstacle_node ${OpenCV_LIBS})

add_executable(stereo_depth_node src/stereo_depth_node.cpp)
ament_target_dependencies(stereo_depth_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
)
target_link_libraries(stereo_depth_node ${OpenCV_LIBS})

add_executable(drive_safety_node src/drive_safety_node.cpp)
ament_target_dependencies(drive_safety_node
  rclcpp
  nav_msgs
  ackermann_msgs
)

add_executable(ackermann_to_twist_node src/ackermann_to_twist_node.cpp)
ament_target_dependencies(ackermann_to_twist_node
  rclcpp
  geometry_msgs
  ackermann_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  cone_nav/path_planner_node.py
  cone_nav/obstacle_avoidance_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  cone_detector_node
  cone_localizer_node
  pure_pursuit_node
  depth_obstacle_node
  stereo_depth_node
  drive_safety_node
  ackermann_to_twist_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  models
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(isaac_contract test/test_isaac_contract.py)
  ament_add_pytest_test(avoidance_algorithms test/test_avoidance_algorithms.py)
endif()

ament_package()
