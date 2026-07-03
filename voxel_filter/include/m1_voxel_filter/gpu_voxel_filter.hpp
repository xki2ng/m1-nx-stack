#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace m1
{

/**
 * @brief GPU-accelerated voxel downsampling filter.
 *
 * Downsamples a point cloud using fixed-size voxels on the GPU,
 * preserving per-point ring and timestamp metadata.
 * Uses Thrust sort_by_key + unique_by_key for O(N log N) filtering.
 */
class GpuVoxelFilter
{
public:
  using PCLPoint = pcl::PointXYZINormal;
  using PCLCloud = pcl::PointCloud<PCLPoint>;

  GpuVoxelFilter();
  ~GpuVoxelFilter();

  GpuVoxelFilter(const GpuVoxelFilter &) = delete;
  GpuVoxelFilter &operator=(const GpuVoxelFilter &) = delete;

  /** @brief Set voxel leaf size in meters. Must be > 0. */
  void set_leaf_size(float leaf_m);

  /** @brief Returns true if GPU resources are available. */
  bool available() const;

  /**
   * @brief Filter a point cloud with per-point metadata.
   *
   * @param cloud      Input point cloud (x, y, z, intensity used)
   * @param rings      Per-point ring/channel ID (size == cloud.size())
   * @param timestamps Per-point timestamp in seconds (size == cloud.size())
   * @param filtered   Output filtered point cloud
   * @param rings_out  Filtered ring values (1:1 with filtered points)
   * @param stamps_out Filtered timestamp values (1:1 with filtered points)
   * @return true if GPU path succeeded, false on fallback
   */
  bool filter(const PCLCloud &cloud,
              const std::vector<uint16_t> &rings,
              const std::vector<double> &timestamps,
              PCLCloud &filtered,
              std::vector<uint16_t> &rings_out,
              std::vector<double> &stamps_out);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace m1
