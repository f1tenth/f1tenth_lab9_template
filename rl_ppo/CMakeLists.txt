cmake_minimum_required(VERSION 3.5)
project(rl_ppo)

# Default to C++17 (required for structured bindings)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE
  ament_cmake
  ament_cmake_python
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  Eigen3
  tf2
  tf2_ros)

  find_package(${PACKAGE} REQUIRED)

endforeach()

find_package(nlohmann_json REQUIRED)

# ONNX Runtime does not ship a standard CMake find-module, so locate it
# manually. Install via `sudo apt install libonnxruntime-dev` if your distro
# packages it, or point CMAKE_PREFIX_PATH at an extracted release from
# https://github.com/microsoft/onnxruntime/releases
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATH_SUFFIXES onnxruntime)
find_library(ONNXRUNTIME_LIB onnxruntime)
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR "onnxruntime not found - install libonnxruntime-dev, or "
                       "extract a release from https://github.com/microsoft/"
                       "onnxruntime/releases and add it to CMAKE_PREFIX_PATH")
endif()

# Include Cpp "include" directory
include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
)

# Create Cpp executable
add_executable(ppo_deploy_node src/ppo_deploy_node.cpp)

target_include_directories(ppo_deploy_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
)

ament_target_dependencies(ppo_deploy_node
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2 tf2_ros
)

target_link_libraries(ppo_deploy_node ${ONNXRUNTIME_LIB} nlohmann_json::nlohmann_json)

# Install Cpp executables
install(TARGETS
  ppo_deploy_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/ppo_deploy_node.py
  scripts/export_ppo_onnx.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install scripts directory (export tool, README, and the onnx model +
# config once you generate them with export_ppo_onnx.py) so the launch
# file's default onnx_path/config_path can find them under share/rl_ppo/scripts
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
