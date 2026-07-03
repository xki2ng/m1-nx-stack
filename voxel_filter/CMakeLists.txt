cmake_minimum_required(VERSION 3.8)
project(m1_voxel_filter)

# Default to Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# CUDA
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
  set(CMAKE_CUDA_ARCHITECTURES "72;87" CACHE STRING "CUDA architectures" FORCE)
endif()
enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
add_definitions(-DFASTLIO_USE_CUDA)

# ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# PCL
find_package(PCL REQUIRED COMPONENTS common io)

set(dependencies
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

# Build shared library for the GPU filter
add_library(m1_voxel_filter_lib SHARED
  src/gpu_voxel_filter.cu
)
target_include_directories(m1_voxel_filter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(m1_voxel_filter_lib
  CUDA::cudart
  ${PCL_LIBRARIES}
)
target_include_directories(m1_voxel_filter_lib PRIVATE ${CUDAToolkit_INCLUDE_DIRS})

# ROS2 node executable
add_executable(m1_voxel_flt
  src/voxel_filter_node.cpp
)
target_include_directories(m1_voxel_flt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(m1_voxel_flt
  m1_voxel_filter_lib
  ${PCL_LIBRARIES}
)
ament_target_dependencies(m1_voxel_flt ${dependencies})

# Install
install(TARGETS
  m1_voxel_filter_lib
  m1_voxel_flt
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

# Lidar merger (no GPU, no PCL — plain C++)
add_executable(lidar_merger
  src/lidar_merger.cpp
)
target_include_directories(lidar_merger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(lidar_merger rclcpp sensor_msgs)

install(TARGETS lidar_merger
  DESTINATION lib/${PROJECT_NAME}
)

# Nav2 merger (TF transform, no CUDA)
add_executable(nav2_merger src/nav2_merger.cpp)
ament_target_dependencies(nav2_merger rclcpp sensor_msgs)
install(TARGETS nav2_merger DESTINATION lib/${PROJECT_NAME})
