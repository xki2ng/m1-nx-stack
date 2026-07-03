#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstring>
#include <memory>

class LidarMerger : public rclcpp::Node
{
public:
  LidarMerger() : Node("lidar_merger")
  {
    this->declare_parameter("rear_tx", 0.0);
    this->declare_parameter("rear_ty", 0.0);
    this->declare_parameter("rear_tz", -0.8086);

    rear_tx_ = this->get_parameter("rear_tx").as_double();
    rear_ty_ = this->get_parameter("rear_ty").as_double();
    rear_tz_ = this->get_parameter("rear_tz").as_double();

    sub_front_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/front_lidar/filtered", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          latest_front_ = msg; try_publish(); });
    sub_rear_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rear_lidar/filtered", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          latest_rear_ = msg; });
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/merged_lidar/filtered", rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(),
        "Merger: R_x(180) + t=[%.3f,%.3f,%.3f]", rear_tx_, rear_ty_, rear_tz_);
  }

private:
  void try_publish()
  {
    if (!latest_front_ || !latest_rear_) return;
    const auto &fc = *latest_front_;
    const auto &rc = *latest_rear_;
    const uint32_t nf = fc.width, nr = rc.width;
    if (nf == 0 || nr == 0) return;

    auto merged = std::make_unique<sensor_msgs::msg::PointCloud2>();
    merged->header = fc.header;
    merged->header.frame_id = "rslidar_head";
    merged->height = 1; merged->width = nf + nr;
    merged->is_dense = true;
    merged->fields = fc.fields;
    merged->point_step = fc.point_step;
    merged->row_step = merged->point_step * merged->width;
    merged->data.resize(merged->row_step);

    std::memcpy(merged->data.data(), fc.data.data(), fc.data.size());

    const uint32_t rear_start = fc.data.size();
    const uint32_t ps = fc.point_step;
    static bool debug_done = false;

    for (uint32_t i = 0; i < nr; ++i)
    {
      const uint32_t src = i * ps;
      const uint32_t dst = rear_start + src;

      float rx, ry, rz;
      std::memcpy(&rx, rc.data.data() + src + 0, 4);
      std::memcpy(&ry, rc.data.data() + src + 4, 4);
      std::memcpy(&rz, rc.data.data() + src + 8, 4);

      // R_x(180°): x stays, y→-y, z→-z + translation
      float fx = rx + rear_tx_;
      float fy = -ry + rear_ty_;
      float fz = -rz + rear_tz_;

      if (!debug_done) {
        debug_done = true;
        RCLCPP_INFO(this->get_logger(),
            "DEBUG rear[0]: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)", rx, ry, rz, fx, fy, fz);
      }

      std::memcpy(merged->data.data() + dst, rc.data.data() + src, ps);
      std::memcpy(merged->data.data() + dst + 0, &fx, 4);
      std::memcpy(merged->data.data() + dst + 4, &fy, 4);
      std::memcpy(merged->data.data() + dst + 8, &fz, 4);

      uint16_t ring;
      std::memcpy(&ring, merged->data.data() + dst + 16, sizeof(ring));
      ring += 96;
      std::memcpy(merged->data.data() + dst + 16, &ring, sizeof(ring));
    }

    pub_->publish(std::move(merged));
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "merged: %u + %u = %u pts", nf, nr, nf + nr);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_front_, sub_rear_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_front_, latest_rear_;
  double rear_tx_, rear_ty_, rear_tz_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarMerger>());
  rclcpp::shutdown();
  return 0;
}
