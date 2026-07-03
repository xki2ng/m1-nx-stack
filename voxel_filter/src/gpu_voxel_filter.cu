#include "m1_voxel_filter/gpu_voxel_filter.hpp"

#include <cuda_runtime.h>

#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>
#include <thrust/gather.h>
#include <thrust/sort.h>
#include <thrust/unique.h>

#include <cmath>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace m1
{
namespace
{

// ---- CUDA error checking ----
inline void check_cuda(cudaError_t err, const char *expr, const char *file, int line)
{
  if (err == cudaSuccess) return;
  std::ostringstream oss;
  oss << "CUDA call '" << expr << "' failed at " << file << ":" << line
      << " with error '" << cudaGetErrorString(err) << "'";
  throw std::runtime_error(oss.str());
}
#define CUDA_CHECK(expr) ::m1::check_cuda((expr), #expr, __FILE__, __LINE__)

// ---- Packed point struct (GPU-compatible layout) ----
struct PackedPoint
{
  float x;
  float y;
  float z;
  float intensity;
  uint16_t ring;
  double timestamp;
};

// ---- Voxel key encoding ----
__device__ inline uint64_t encode_key(int ix, int iy, int iz)
{
  const int bias = 1 << 20;
  const uint64_t mask = (1ULL << 21) - 1ULL;
  const uint64_t ux = (static_cast<uint64_t>(ix + bias) & mask);
  const uint64_t uy = (static_cast<uint64_t>(iy + bias) & mask);
  const uint64_t uz = (static_cast<uint64_t>(iz + bias) & mask);
  return (ux << 42) | (uy << 21) | uz;
}

// ---- CUDA kernels ----
__global__ void compute_keys_kernel(const PackedPoint *points,
                                    uint64_t *keys,
                                    int *indices,
                                    size_t total_points,
                                    float inv_leaf)
{
  const size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= total_points) return;

  const PackedPoint p = points[idx];
  const int ix = static_cast<int>(floorf(p.x * inv_leaf));
  const int iy = static_cast<int>(floorf(p.y * inv_leaf));
  const int iz = static_cast<int>(floorf(p.z * inv_leaf));
  keys[idx] = encode_key(ix, iy, iz);
  indices[idx] = static_cast<int>(idx);
}

__global__ void gather_points_kernel(const PackedPoint *input,
                                     const int *indices,
                                     PackedPoint *output,
                                     size_t count)
{
  const size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= count) return;
  output[idx] = input[indices[idx]];
}

}  // anonymous namespace

// ---- PIMPL ----
struct GpuVoxelFilter::Impl
{
  cudaStream_t stream_{nullptr};
  bool available_{true};
  float leaf_size_{0.15f};
  float inv_leaf_size_{1.0f / 0.15f};
  size_t capacity_{0};

  PackedPoint *d_points_{nullptr};
  PackedPoint *d_compacted_{nullptr};
  uint64_t *d_keys_{nullptr};
  int *d_indices_{nullptr};

  std::vector<PackedPoint> host_points_;
  std::vector<PackedPoint> host_compacted_;

  Impl()
  {
    if (cudaStreamCreate(&stream_) != cudaSuccess)
    {
      available_ = false;
    }
  }

  ~Impl()
  {
    release();
    if (stream_)
    {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
  }

  void set_leaf(float leaf)
  {
    if (leaf <= 0.0f) leaf = 0.1f;
    leaf_size_ = leaf;
    inv_leaf_size_ = 1.0f / leaf;
  }

  bool available() const { return available_; }

  bool filter(const PCLCloud &cloud,
              const std::vector<uint16_t> &rings,
              const std::vector<double> &timestamps,
              PCLCloud &filtered,
              std::vector<uint16_t> &rings_out,
              std::vector<double> &stamps_out)
  {
    if (!available_) return false;

    const size_t total_points = cloud.size();
    if (total_points == 0)
    {
      filtered.clear();
      rings_out.clear();
      stamps_out.clear();
      return true;
    }

    try { ensure_capacity(total_points); }
    catch (const std::exception &ex)
    {
      available_ = false;
      return false;
    }

    // Pack host data
    host_points_.resize(total_points);
    for (size_t i = 0; i < total_points; ++i)
    {
      const auto &pt = cloud.points[i];
      host_points_[i] = PackedPoint{
        pt.x, pt.y, pt.z, pt.intensity,
        rings[i],
        timestamps[i]
      };
    }

    // Upload to GPU
    CUDA_CHECK(cudaMemcpyAsync(d_points_, host_points_.data(),
                               total_points * sizeof(PackedPoint),
                               cudaMemcpyHostToDevice, stream_));

    // Compute voxel keys
    const int threads = 256;
    const int blocks = static_cast<int>((total_points + threads - 1) / threads);
    compute_keys_kernel<<<blocks, threads, 0, stream_>>>(
        d_points_, d_keys_, d_indices_, total_points, inv_leaf_size_);
    CUDA_CHECK(cudaGetLastError());

    // Sort + unique by key (Thrust)
    auto exec = thrust::cuda::par.on(stream_);
    auto keys_begin = thrust::device_pointer_cast(d_keys_);
    auto idx_begin = thrust::device_pointer_cast(d_indices_);
    thrust::sort_by_key(exec, keys_begin, keys_begin + total_points, idx_begin);
    auto new_end = thrust::unique_by_key(exec, keys_begin, keys_begin + total_points, idx_begin);
    size_t unique_count = new_end.first - keys_begin;

    if (unique_count == 0)
    {
      CUDA_CHECK(cudaStreamSynchronize(stream_));
      filtered.clear();
      rings_out.clear();
      stamps_out.clear();
      return true;
    }

    // Gather unique points
    const int gather_blocks = static_cast<int>((unique_count + threads - 1) / threads);
    gather_points_kernel<<<gather_blocks, threads, 0, stream_>>>(
        d_points_, d_indices_, d_compacted_, unique_count);
    CUDA_CHECK(cudaGetLastError());

    // Download results
    host_compacted_.resize(unique_count);
    CUDA_CHECK(cudaMemcpyAsync(host_compacted_.data(), d_compacted_,
                               unique_count * sizeof(PackedPoint),
                               cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaStreamSynchronize(stream_));

    // Unpack to output
    filtered.clear();
    filtered.reserve(unique_count);
    rings_out.clear();
    rings_out.reserve(unique_count);
    stamps_out.clear();
    stamps_out.reserve(unique_count);

    for (size_t i = 0; i < unique_count; ++i)
    {
      PCLPoint pt;
      pt.x = host_compacted_[i].x;
      pt.y = host_compacted_[i].y;
      pt.z = host_compacted_[i].z;
      pt.intensity = host_compacted_[i].intensity;
      pt.curvature = 0.0f;
      pt.normal_x = pt.normal_y = pt.normal_z = 0.0f;
      filtered.push_back(pt);
      rings_out.push_back(host_compacted_[i].ring);
      stamps_out.push_back(host_compacted_[i].timestamp);
    }

    return true;
  }

private:
  void ensure_capacity(size_t desired)
  {
    if (desired <= capacity_) return;
    release();
    capacity_ = desired;
    CUDA_CHECK(cudaMalloc(&d_points_, capacity_ * sizeof(PackedPoint)));
    CUDA_CHECK(cudaMalloc(&d_compacted_, capacity_ * sizeof(PackedPoint)));
    CUDA_CHECK(cudaMalloc(&d_keys_, capacity_ * sizeof(uint64_t)));
    CUDA_CHECK(cudaMalloc(&d_indices_, capacity_ * sizeof(int)));
  }

  void release()
  {
    if (d_points_)     { cudaFree(d_points_);     d_points_     = nullptr; }
    if (d_compacted_)  { cudaFree(d_compacted_);  d_compacted_  = nullptr; }
    if (d_keys_)       { cudaFree(d_keys_);       d_keys_       = nullptr; }
    if (d_indices_)    { cudaFree(d_indices_);    d_indices_    = nullptr; }
    capacity_ = 0;
  }
};

// ---- Public API ----
GpuVoxelFilter::GpuVoxelFilter() : impl_(std::make_unique<Impl>()) {}
GpuVoxelFilter::~GpuVoxelFilter() = default;

void GpuVoxelFilter::set_leaf_size(float leaf_m)
{
  if (impl_) impl_->set_leaf(leaf_m);
}

bool GpuVoxelFilter::available() const
{
  return impl_ && impl_->available();
}

bool GpuVoxelFilter::filter(const PCLCloud &cloud,
                            const std::vector<uint16_t> &rings,
                            const std::vector<double> &timestamps,
                            PCLCloud &filtered,
                            std::vector<uint16_t> &rings_out,
                            std::vector<double> &stamps_out)
{
  if (!impl_) return false;
  return impl_->filter(cloud, rings, timestamps, filtered, rings_out, stamps_out);
}

}  // namespace m1
