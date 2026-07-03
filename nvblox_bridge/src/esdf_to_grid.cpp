#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>

class EsdfToGridNode : public rclcpp::Node {
public:
    EsdfToGridNode() : Node("esdf_to_grid") {
        // Parameters
        this->declare_parameter("resolution", 0.1);
        this->declare_parameter("max_dist", 1.5); // Distance at which cost becomes 0
        
        resolution_ = this->get_parameter("resolution").as_double();
        max_dist_ = this->get_parameter("max_dist").as_double();

        pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("esdf_map", 1);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/nvblox/esdf_slice", 10,
            std::bind(&EsdfToGridNode::cb, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "ESDF to Grid Converter started. Res: %.2f, MaxDist: %.2f", resolution_, max_dist_);
    }

private:
    void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        if (cloud.empty()) return;

        // 1. Find bounds of the map
        float min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6;
        for (const auto& p : cloud.points) {
            if (p.x < min_x) min_x = p.x;
            if (p.x > max_x) max_x = p.x;
            if (p.y < min_y) min_y = p.y;
            if (p.y > max_y) max_y = p.y;
        }

        // Add small padding
        min_x -= resolution_; max_x += resolution_;
        min_y -= resolution_; max_y += resolution_;

        // Snap origin to grid resolution to avoid aliasing
        min_x = std::floor(min_x / resolution_) * resolution_;
        min_y = std::floor(min_y / resolution_) * resolution_;

        // 2. Initialize Grid
        int width = std::ceil((max_x - min_x) / resolution_);
        int height = std::ceil((max_y - min_y) / resolution_);

        nav_msgs::msg::OccupancyGrid grid;
        grid.header = msg->header;
        grid.info.resolution = resolution_;
        grid.info.width = width;
        grid.info.height = height;
        grid.info.origin.position.x = min_x;
        grid.info.origin.position.y = min_y;
        grid.info.origin.orientation.w = 1.0;
        grid.data.resize(width * height, -1); // Default to unknown

        // 3. Fill Grid
        // We map ESDF distance to Cost [0-100]
        // Distance <= 0 -> 100 (Lethal)
        // Distance >= max_dist -> 0 (Free)
        // 0 < Distance < max_dist -> Gradient
        
        for (const auto& p : cloud.points) {
            int ix = std::floor((p.x - min_x) / resolution_);
            int iy = std::floor((p.y - min_y) / resolution_);
            
            if (ix >= 0 && ix < width && iy >= 0 && iy < height) {
                float dist = p.intensity;
                int8_t cost = 0;
                
                if (dist <= 0.0) {
                    cost = 100;
                } else if (dist >= max_dist_) {
                    cost = 0;
                } else {
                    // Scale non-lethal costs to be max 60 to ensure traversability.
                    // This creates a gradient for centering but ensures doorways (narrow spaces)
                    // are not marked as lethal (100) or near-lethal.
                    cost = static_cast<int8_t>(60.0 * (1.0 - dist / max_dist_));
                }
                
                // Take max cost if multiple points fall in same cell (conservative)
                int idx = iy * width + ix;
                if (grid.data[idx] == -1 || cost > grid.data[idx]) {
                    grid.data[idx] = cost;
                }
            }
        }
        
        // 4. Gap Filling (Post-processing)
        // Fix "vertical lines" or empty spots by interpolating from neighbors
        std::vector<int8_t> filled_data = grid.data;
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;
                
                if (grid.data[idx] == -1) {
                    // Check neighbors
                    int valid_neighbors = 0;
                    int max_neighbor_cost = -1;
                    
                    // 4-connectivity
                    const int dx[] = {0, 0, -1, 1};
                    const int dy[] = {-1, 1, 0, 0};
                    
                    for (int k = 0; k < 4; ++k) {
                        int nx = x + dx[k];
                        int ny = y + dy[k];
                        
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            int nidx = ny * width + nx;
                            if (grid.data[nidx] != -1) {
                                valid_neighbors++;
                                if (grid.data[nidx] > max_neighbor_cost) {
                                    max_neighbor_cost = grid.data[nidx];
                                }
                            }
                        }
                    }
                    
                    // If we have neighbors, fill the gap
                    if (valid_neighbors > 0) {
                        filled_data[idx] = max_neighbor_cost; 
                    }
                }
            }
        }
        grid.data = filled_data;
        
        pub_->publish(grid);
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    double resolution_;
    double max_dist_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EsdfToGridNode>());
    rclcpp::shutdown();
    return 0;
}
