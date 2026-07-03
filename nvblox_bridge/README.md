# nvblox_fastlio_bridge

> GPU-ready ROS 2 bridge that feeds FAST-LIO state estimation directly into NVBlox for real-time TSDF, mesh, and ESDF map generation on Jetson Orin.

![Live mapping demo](docs/live_mapping_demo.gif)

## Overview
This package provides a high-performance bridge between **FAST-LIO** (LiDAR-inertial odometry) and **nvblox** (GPU-accelerated volumetric mapping). It consumes the registered point clouds and odometry from FAST-LIO to generate dense 3D reconstructions in real-time on NVIDIA Jetson platforms.

### Key Features
- **Seamless Integration**: Directly subscribes to `/cloud_registered` and `/Odometry` from FAST-LIO.
- **Jetson Optimized**: Tuned for NVIDIA Jetson Orin (SM 8.7) with CUDA 12.8.
- **Real-time Reconstruction**: Generates TSDF, ESDF, and Mesh layers at LiDAR frame rates (10-20 Hz).
- **Rich Visualization**: Pre-configured RViz setup for viewing meshes, voxel grids, and distance fields.
- **Voxel Export**: Includes functionality to export the reconstructed map to PCD files.

## Repository Layout
| Path | Purpose |
| --- | --- |
| `launch/` | ROS 2 launch files (`nvblox_fastlio.launch.py`). |
| `src/` | Core bridge source code. |
| `config/` | Configuration files for nvblox and FAST-LIO. |
| `rviz/` | Visualization configuration (`nvblox_mapper.rviz`). |

## Getting Started

### Prerequisites
- **Hardware**: NVIDIA Jetson Orin (or other CUDA-capable GPU).
- **OS**: Ubuntu 22.04 (ROS 2 Humble).
- **Dependencies**:
  - ROS 2 Humble
  - CUDA Toolkit (12.8+ recommended for Orin)
  - [nvblox](https://github.com/nvidia-isaac/nvblox) (installed and built)
  - [FAST_LIO](https://github.com/OmerMersin/FAST_LIO_GPU) (GPU accelerated version)

### Installation

1.  **Clone the repository** into your ROS 2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone git@github.com:OmerMersin/fastlio_nvblox_mapper.git nvblox_fastlio_bridge
    ```

2.  **Build the package**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select nvblox_fastlio_bridge
    source install/setup.bash
    ```

## Usage

### Launching the Mapper
To launch FAST-LIO and the nvblox bridge together with RViz visualization:

```bash
ros2 launch nvblox_fastlio_bridge nvblox_fastlio.launch.py \
  fastlio_config:=/path/to/fast_lio/config/ouster32.yaml \
  bridge_config:=/path/to/nvblox_fastlio_bridge/config/nvblox_params.yaml
```

**Arguments:**
- `fastlio_config`: Absolute path to your FAST-LIO sensor configuration file (default: `fast_lio/config/ouster32.yaml`).
- `bridge_config`: Path to the nvblox parameter file (default: `config/nvblox_params.yaml`).
- `launch_rviz`: Set to `true` or `false` to enable/disable RViz (default: `true`).

### Example Command
```bash
ros2 launch nvblox_fastlio_bridge nvblox_fastlio.launch.py \
  fastlio_config:=/home/orin/LIONav/ros2_ws/src/FAST_LIO/config/ouster32.yaml \
  bridge_config:=/home/orin/LIONav/ros2_ws/src/nvblox_fastlio_bridge/config/nvblox_params.yaml
```

## Configuration

### nvblox Parameters
The mapping behavior is controlled by `config/nvblox_params.yaml`. Key parameters include:
- `voxel_size`: Resolution of the map (default: 0.10m).
- `truncation_distance_vox`: Truncation distance in voxel units.
- `esdf_update_rate_hz`: Frequency of ESDF updates.

### FAST-LIO Parameters
Refer to the [FAST-LIO repository](https://github.com/OmerMersin/FAST_LIO_GPU) for details on sensor configuration (LiDAR type, extrinsics, etc.).

## Visualization
The launch file automatically opens RViz with a pre-configured view (`rviz/nvblox_mapper.rviz`) showing:
- **Point Cloud**: The raw registered cloud from FAST-LIO.
- **Mesh**: The reconstructed mesh from nvblox.
- **Odometry**: The path and pose of the sensor.
| `max_integration_distance` | `15.0` | Clip lidar rays beyond this range. |
| `pointcloud_topic` | `/cloud_registered` | FAST-LIO registered cloud input. |
| `odom_topic` | `/Odometry` | FAST-LIO pose stream used for TSDF poses. |
| `publish_voxels` | `true` | Toggle voxel `CUBE_LIST` marker publisher. |
| `publish_voxel_points` | `false` | Publish sampled voxel point cloud for analytics. |
| `voxel_export_*` | see `nvblox_fastlio_node.cpp` | Filters for visualization and PCD exports. |

All parameters are declared on the node, so you can override them via launch files, YAML, or `ros2 param set` at runtime.

## Runtime interfaces
- **Subscriptions**: `/cloud_registered` (`sensor_msgs/PointCloud2`), `/Odometry` (`nav_msgs/Odometry`).
- **Publishers**:
  - `nvblox/mesh` (`visualization_msgs/Marker` TRIANGLE_LIST)
  - `nvblox/voxels` (`visualization_msgs/Marker` CUBE_LIST)
  - `nvblox/voxel_points` (`sensor_msgs/PointCloud2`, optional)
  - `nvblox/esdf_slice`, `nvblox/tsdf_slice` (slice point clouds)
  - `nvblox/occupancy_grid` (`nav_msgs/OccupancyGrid`)
- **Services**: `nvblox/save_voxels` (`std_srvs/Trigger`) → writes filtered TSDF voxels to PCD.

## Tips for best results
1. **Clock alignment**: make sure FAST-LIO and NVBlox run on synchronized clocks (use `/use_sim_time` or PTP if needed).
2. **TF availability**: keep `map -> body/lidar` transforms continuous; the node falls back to raw odometry, but TF avoids drift during temporary dropouts.
3. **Denoising toggles**: `lidar_denoise_*`, `lidar_smooth_*`, and `lidar_hole_filling_radius` help when Ouster returns sparse walls or Livox has motion blur.
4. **Jetson thermals**: pin clocks with `jetson_clocks` and watch GPU temps when integrating dense indoor scans.

## License
Released under the [MIT License](./LICENSE).
