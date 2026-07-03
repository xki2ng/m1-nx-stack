#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstring>
#include <memory>
#include <cmath>

// Airy96 LiDAR frame (rslidar_head): X=down, Y=left, Z=forward
// Rear lidar: R_x(180°) + z=-0.8086 → approximately same orientation
// NOTE: height filter removed — all points pass through to Nav2 costmap

class Nav2Merger : public rclcpp::Node
{
public:
  Nav2Merger() : Node("nav2_merger")
  {
    // Rear→Front TF
    this->declare_parameter("rear_tz", -0.8086);
    rear_tz_ = this->get_parameter("rear_tz").as_double();

    sub_front_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/front_lidar/filtered", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){ latest_front_ = msg; try_publish(); });
    sub_rear_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rear_lidar/filtered", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){ latest_rear_ = msg; });

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/merged_cloud", rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(),
      "Nav2Merger: rear_tz=%.2f (no height filter) → /merged_cloud",
      rear_tz_);
  }

private:
  void try_publish()
  {
    if (!latest_front_ || !latest_rear_) return;
    const auto &fc = *latest_front_, &rc = *latest_rear_;
    uint32_t nf = fc.width, nr = rc.width;
    if (nf == 0 && nr == 0) return;

    uint32_t ps = fc.point_step;

    uint32_t total = nf + nr;
    if (total == 0) return;

    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out->header = fc.header;
    out->height = 1; out->width = total; out->is_dense = true;
    out->fields = fc.fields;
    out->point_step = ps;
    out->row_step = ps * total;
    out->data.resize(out->row_step);

    // Front: direct copy
    std::memcpy(out->data.data(), fc.data.data(), fc.data.size());

    // Rear: R_x(180°) + translation, then copy
    uint32_t dst = nf;
    for (uint32_t i = 0; i < nr; ++i) {
      uint32_t src_off = i * ps;
      float rx, ry, rz;
      std::memcpy(&rx, rc.data.data() + src_off + 0, 4);
      std::memcpy(&ry, rc.data.data() + src_off + 4, 4);
      std::memcpy(&rz, rc.data.data() + src_off + 8, 4);

      if (std::isnan(rx) || std::isnan(ry) || std::isnan(rz)) {
        out->is_dense = false;
      }

      // R_x(180°): x stays, y=-y, z=-z + translation
      float fx = rx;
      float fy = -ry;
      float fz = -rz + rear_tz_;

      std::memcpy(out->data.data() + dst * ps, rc.data.data() + src_off, ps);
      std::memcpy(out->data.data() + dst * ps + 0, &fx, 4);
      std::memcpy(out->data.data() + dst * ps + 4, &fy, 4);
      std::memcpy(out->data.data() + dst * ps + 8, &fz, 4);
      dst++;
    }

    pub_->publish(std::move(out));
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                         "merged: front %u + rear %u = %u pts (no height filter)",
                         nf, nr, nf + nr);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_front_, sub_rear_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_front_, latest_rear_;
  double rear_tz_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2Merger>());
  rclcpp::shutdown();
  return 0;
}
