cmake_minimum_required(VERSION 3.8)

# Set CUDA paths BEFORE project() call
set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.6/bin/nvcc")
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.6")
set(ENV{PATH} "/usr/local/cuda-12.6/bin:$ENV{PATH}")

project(nvblox_fastlio_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set CUDA architecture before enabling CUDA
set(CMAKE_CUDA_ARCHITECTURES "87")  # Orin is SM 8.7

# Find CUDA using modern CMake approach
find_package(CUDAToolkit REQUIRED)
enable_language(CUDA)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PCL REQUIRED)
find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)

# nvblox paths
set(NVBLOX_DIR "/home/robot/workspaces/nvblox/nvblox")
set(NVBLOX_BUILD_DIR "/home/robot/workspaces/nvblox/build")
set(NVBLOX_INCLUDE_DIR "${NVBLOX_DIR}/include")
set(NVBLOX_LIB_DIR "${NVBLOX_BUILD_DIR}/nvblox")

# Check if nvblox library exists
if(NOT EXISTS "${NVBLOX_LIB_DIR}/libnvblox_lib.so")
  message(FATAL_ERROR "nvblox library not found at ${NVBLOX_LIB_DIR}/libnvblox_lib.so. Please build nvblox first.")
endif()

# Import nvblox library
add_library(nvblox_lib SHARED IMPORTED)
set_target_properties(nvblox_lib PROPERTIES
  IMPORTED_LOCATION "${NVBLOX_LIB_DIR}/libnvblox_lib.so"
  INTERFACE_INCLUDE_DIRECTORIES "${NVBLOX_INCLUDE_DIR}"
)


# Find stdgpu (nvblox dependency)
find_path(STDGPU_INCLUDE_DIR stdgpu/memory.h
  HINTS 
    ${NVBLOX_BUILD_DIR}/_deps/ext_stdgpu-src/src
    /usr/local/include
    /usr/include
)

# Include directories
include_directories(
  include
  ${NVBLOX_INCLUDE_DIR}
  ${NVBLOX_BUILD_DIR}/_deps/ext_stdgpu-src/src
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

# Build the bridge node
add_executable(m1_nvblox_br
  src/nvblox_fastlio_node.cpp
)

# Build ESDF to Grid converter
add_executable(esdf_to_grid
  src/esdf_to_grid.cpp
)
target_link_libraries(esdf_to_grid
  ${PCL_LIBRARIES}
)
ament_target_dependencies(esdf_to_grid
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
)

# Set RPATH so the executable can find nvblox library at runtime
set_target_properties(m1_nvblox_br PROPERTIES
  BUILD_RPATH "${NVBLOX_LIB_DIR}"
  INSTALL_RPATH "${NVBLOX_LIB_DIR}"
)

target_link_libraries(m1_nvblox_br
  nvblox_lib
  glog::glog
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}
  CUDA::cudart
)

ament_target_dependencies(m1_nvblox_br
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
  pcl_ros
  message_filters
  std_srvs
)

# Install targets
install(TARGETS
  m1_nvblox_br
  esdf_to_grid
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
