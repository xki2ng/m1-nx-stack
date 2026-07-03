#include "m1_voxel_filter/gpu_voxel_filter.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

class VoxelFilterNode : public rclcpp::Node
{
public:
  VoxelFilterNode()
    : Node("voxel_filter_node")
  {
    this->declare_parameter("leaf_size", 0.2);
    this->declare_parameter("input_topic", "/livox/lidar");
    this->declare_parameter("output_topic", "/livox/lidar_filtered");

    const float leaf = static_cast<float>(this->get_parameter("leaf_size").as_double());
    const std::string in_topic  = this->get_parameter("input_topic").as_string();
    const std::string out_topic = this->get_parameter("output_topic").as_string();

    filter_ = std::make_unique<m1::GpuVoxelFilter>();
    if (!filter_->available())
    {
      RCLCPP_ERROR(this->get_logger(), "GPU VoxelFilter unavailable — CUDA init failed");
      return;
    }
    filter_->set_leaf_size(leaf);
    RCLCPP_INFO(this->get_logger(),
                "GPU VoxelFilter ready — leaf=%.3fm, in='%s', out='%s'",
                leaf, in_topic.c_str(), out_topic.c_str());

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        in_topic, rclcpp::SensorDataQoS(),
        std::bind(&VoxelFilterNode::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        out_topic, rclcpp::SensorDataQoS());
  }

private:
  // Check if a field exists in the PointCloud2
  static bool has_field(const sensor_msgs::msg::PointCloud2 &msg, const std::string &name)
  {
    for (const auto &f : msg.fields)
    {
      if (f.name == name) return true;
    }
    return false;
  }

  void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const size_t total = msg->width * msg->height;
    if (total == 0) return;

    // --- 1. Extract ring & timestamp fields ---
    std::vector<uint16_t> rings(total, 0);
    std::vector<double> timestamps(total, 0.0);

    bool has_ring = has_field(*msg, "ring");
    if (has_ring)
    {
      sensor_msgs::PointCloud2ConstIterator<uint16_t> it(*msg, "ring");
      size_t i = 0;
      for (; it != it.end() && i < total; ++it, ++i)
      {
        rings[i] = *it;
      }
    }
    else
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Input cloud lacks 'ring' field — output will have ring=0");
    }

    // Time field: try "timestamp", "t", "time"
    std::string time_field;
    for (const auto &candidate : {"timestamp", "t", "time"})
    {
      if (has_field(*msg, candidate))
      {
        time_field = candidate;
        break;
      }
    }

    bool has_time = false;
    if (!time_field.empty())
    {
      // Determine datatype of the time field
      uint8_t dt = sensor_msgs::msg::PointField::FLOAT64;
      for (const auto &f : msg->fields)
      {
        if (f.name == time_field) { dt = f.datatype; break; }
      }

      if (dt == sensor_msgs::msg::PointField::FLOAT64)
      {
        sensor_msgs::PointCloud2ConstIterator<double> it(*msg, time_field);
        size_t i = 0;
        for (; it != it.end() && i < total; ++it, ++i)
        {
          timestamps[i] = *it;
        }
        has_time = true;
      }
      else if (dt == sensor_msgs::msg::PointField::UINT32)
      {
        sensor_msgs::PointCloud2ConstIterator<uint32_t> it(*msg, time_field);
        size_t i = 0;
        for (; it != it.end() && i < total; ++it, ++i)
        {
          timestamps[i] = static_cast<double>(*it);
        }
        has_time = true;
      }
    }

    if (!has_time)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Input cloud lacks timestamp field — output will have time=0.0");
    }

    // --- 2. Convert PointCloud2 → PCL (suppress PCL field warnings) ---
    m1::GpuVoxelFilter::PCLCloud pcl_in;
    {
      // Temporarily suppress PCL console output for missing fields
      pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
      pcl::fromROSMsg(*msg, pcl_in);
    }

    if (pcl_in.size() != total)
    {
      RCLCPP_WARN(this->get_logger(),
                  "PCL conversion size mismatch: %zu vs %zu", pcl_in.size(), total);
    }

    // --- 3. GPU downsampling ---
    m1::GpuVoxelFilter::PCLCloud pcl_out;
    std::vector<uint16_t> rings_out;
    std::vector<double> stamps_out;

    if (!filter_->filter(pcl_in, rings, timestamps, pcl_out, rings_out, stamps_out))
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "GPU filter failed — dropping frame");
      return;
    }

    if (pcl_out.empty()) return;

    const size_t num_points = pcl_out.size();

    // --- 4. Build output PointCloud2 ---
    auto out_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out_msg->header = msg->header;
    out_msg->height = 1;
    out_msg->width = num_points;
    out_msg->is_dense = true;

    // Define output layout: x(f32), y(f32), z(f32), intensity(f32), ring(u16), pad, timestamp(f64)
    int offset = 0;
    out_msg->fields.clear();

    auto add_field = [&](const std::string &name, uint8_t dtype, int count, int byte_size) {
      sensor_msgs::msg::PointField f;
      f.name = name; f.offset = offset; f.datatype = dtype; f.count = count;
      out_msg->fields.push_back(f);
      offset += count * byte_size;
    };

    add_field("x",         sensor_msgs::msg::PointField::FLOAT32, 1, 4);
    add_field("y",         sensor_msgs::msg::PointField::FLOAT32, 1, 4);
    add_field("z",         sensor_msgs::msg::PointField::FLOAT32, 1, 4);
    add_field("intensity", sensor_msgs::msg::PointField::FLOAT32, 1, 4);
    add_field("ring",      sensor_msgs::msg::PointField::UINT16,  1, 2);

    // Align to 8-byte boundary for timestamp (float64)
    int pad = (8 - (offset % 8)) % 8;
    if (pad > 0)
    {
      add_field("_pad", sensor_msgs::msg::PointField::UINT8, pad, 1);
    }

    add_field("timestamp", sensor_msgs::msg::PointField::FLOAT64, 1, 8);

    out_msg->point_step = offset;
    out_msg->row_step = offset * num_points;
    out_msg->data.resize(num_points * offset, 0);

    // Fill data buffer
    for (size_t i = 0; i < num_points; ++i)
    {
      uint8_t *ptr = out_msg->data.data() + i * offset;
      float fx = pcl_out.points[i].x;
      float fy = pcl_out.points[i].y;
      float fz = pcl_out.points[i].z;
      float fi = pcl_out.points[i].intensity;
      uint16_t ring = rings_out[i];
      double ts = stamps_out[i];

      memcpy(ptr + 0,  &fx,   4);
      memcpy(ptr + 4,  &fy,   4);
      memcpy(ptr + 8,  &fz,   4);
      memcpy(ptr + 12, &fi,   4);
      memcpy(ptr + 16, &ring, 2);
      // pad bytes stay 0
      memcpy(ptr + 18 + pad, &ts, 8);
    }

    // --- 5. Publish ---
    pub_->publish(std::move(out_msg));

    // Log stats every 10s
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                         "Voxel filter: %zu → %zu points (leaf=%.3fm, %.1f%% reduction)",
                         total, num_points,
                         static_cast<float>(this->get_parameter("leaf_size").as_double()),
                         100.0 * (1.0 - static_cast<double>(num_points) / total));
  }

  std::unique_ptr<m1::GpuVoxelFilter> filter_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoxelFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
