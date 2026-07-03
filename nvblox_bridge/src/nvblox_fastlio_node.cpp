/**
 * @file nvblox_fastlio_node.cpp
 * @brief ROS2 node bridging FAST-LIO with nvblox for real-time voxel reconstruction
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>
#include <deque>
#include <cstring>  // for memset
#include <algorithm>
#include <filesystem>
#include <limits>
#include <unordered_map>
#include <vector>
#include <cctype>
#include <sstream>
#include <cmath>
#include <initializer_list>
#include <cuda_runtime.h>

#include <rclcpp/parameter.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// nvblox includes
#include <nvblox/nvblox.h>
#include <nvblox/serialization/mesh_serializer_gpu.h>
#include <nvblox/map/accessors.h>

using namespace std::chrono_literals;

class NvbloxFastlioNode : public rclcpp::Node
{
public:
    explicit NvbloxFastlioNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("nvblox_fastlio_node", options)
    {
        // Declare parameters (guarded to allow overrides pre-declare)
        declareIfNotDeclared<double>("voxel_size", 0.1);
        declareIfNotDeclared<double>("truncation_distance_vox", 4.0);
        declareIfNotDeclared<double>("max_integration_distance", 15.0);
        declareIfNotDeclared<double>("lidar_vertical_fov_deg", 45.0);
        declareIfNotDeclared<int>("lidar_num_azimuth", 1024);
        declareIfNotDeclared<int>("lidar_num_elevation", 32);
        declareIfNotDeclared<double>("mesh_update_rate_hz", 2.0);
        declareIfNotDeclared<double>("esdf_update_rate_hz", 2.0);
        declareIfNotDeclared<std::string>("pointcloud_topic", "/cloud_registered");
        declareIfNotDeclared<std::string>("odom_topic", "/Odometry");
        declareIfNotDeclared<std::string>("frame_id", "map");
        declareIfNotDeclared<bool>("publish_mesh", true);
        declareIfNotDeclared<bool>("publish_esdf_slice", true);
        declareIfNotDeclared<bool>("publish_voxels", true);
        declareIfNotDeclared<bool>("publish_voxel_points", false);
        declareIfNotDeclared<bool>("publish_occupancy_grid", true);
        declareIfNotDeclared<std::string>("esdf_mode", "2d");
        declareIfNotDeclared<double>("esdf_slice_height", 0.5);
        declareIfNotDeclared<std::string>("voxel_export_path", "/tmp/nvblox_voxels.pcd");
        declareIfNotDeclared<double>("voxel_export_surface_band", 0.08);
        declareIfNotDeclared<double>("voxel_export_min_weight", 0.2);
        declareIfNotDeclared<bool>("voxel_export_surface_only", true);
        declareIfNotDeclared<int>("voxel_export_points_per_side", 1);
        declareIfNotDeclared<double>("voxel_marker_alpha", 1.0);
        declareIfNotDeclared<bool>("use_tf", true);
        declareIfNotDeclared<double>("tf_timeout_sec", 0.1);
        declareIfNotDeclared<int>("lidar_hole_filling_radius", 1);
        declareIfNotDeclared<std::string>("voxel_color_mode", "flat"); // "flat" or "height"
        declareIfNotDeclared<std::vector<double>>("voxel_flat_color", std::vector<double>{0.0, 1.0, 0.0});
        declareIfNotDeclared<bool>("lidar_denoise_enable", false);
        declareIfNotDeclared<int>("lidar_denoise_k", 50);
        declareIfNotDeclared<double>("lidar_denoise_stddev", 1.0);
        declareIfNotDeclared<bool>("lidar_smooth_enable", false);
        declareIfNotDeclared<double>("lidar_smooth_radius", 0.1);

        // Get parameters
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        truncation_distance_vox_ = this->get_parameter("truncation_distance_vox").as_double();
        max_integration_distance_ = this->get_parameter("max_integration_distance").as_double();
        lidar_vertical_fov_deg_ = this->get_parameter("lidar_vertical_fov_deg").as_double();
        lidar_num_azimuth_ = this->get_parameter("lidar_num_azimuth").as_int();
        lidar_num_elevation_ = this->get_parameter("lidar_num_elevation").as_int();
        mesh_update_rate_hz_ = this->get_parameter("mesh_update_rate_hz").as_double();
        esdf_update_rate_hz_ = this->get_parameter("esdf_update_rate_hz").as_double();
        pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        publish_mesh_ = this->get_parameter("publish_mesh").as_bool();
        publish_esdf_slice_ = this->get_parameter("publish_esdf_slice").as_bool();
        publish_voxels_ = this->get_parameter("publish_voxels").as_bool();
        publish_voxel_points_ = this->get_parameter("publish_voxel_points").as_bool();
        publish_occupancy_grid_ = this->get_parameter("publish_occupancy_grid").as_bool();
        esdf_mode_ = normalizeToken(this->get_parameter("esdf_mode").as_string());
        if (esdf_mode_ != "2d" && esdf_mode_ != "3d") {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid esdf_mode '%s'. Supported values are '2d' or '3d'. Falling back to '2d'.",
                        esdf_mode_.c_str());
            esdf_mode_ = "2d";
        }
        esdf_mode_is_3d_ = (esdf_mode_ == "3d");
        esdf_slice_height_ = this->get_parameter("esdf_slice_height").as_double();
        voxel_export_path_ = this->get_parameter("voxel_export_path").as_string();
    voxel_export_surface_band_ = std::max(5e-3, this->get_parameter("voxel_export_surface_band").as_double());
    voxel_export_min_weight_ = std::max(0.05, this->get_parameter("voxel_export_min_weight").as_double());
    voxel_export_surface_only_ = this->get_parameter("voxel_export_surface_only").as_bool();
        voxel_export_points_per_side_ = std::max(
            1,
            static_cast<int>(this->get_parameter("voxel_export_points_per_side").as_int()));
        voxel_marker_alpha_ = std::clamp(
            this->get_parameter("voxel_marker_alpha").as_double(), 0.0, 1.0);
        use_tf_ = this->get_parameter("use_tf").as_bool();
        tf_timeout_sec_ = this->get_parameter("tf_timeout_sec").as_double();
        lidar_hole_filling_radius_ = this->get_parameter("lidar_hole_filling_radius").as_int();
        voxel_color_mode_ = this->get_parameter("voxel_color_mode").as_string();
        voxel_flat_color_ = this->get_parameter("voxel_flat_color").as_double_array();
        if (voxel_flat_color_.size() != 3) {
            voxel_flat_color_ = {0.0, 1.0, 0.0};
        }
        lidar_denoise_enable_ = this->get_parameter("lidar_denoise_enable").as_bool();
        lidar_denoise_k_ = this->get_parameter("lidar_denoise_k").as_int();
        lidar_denoise_stddev_ = this->get_parameter("lidar_denoise_stddev").as_double();
        lidar_smooth_enable_ = this->get_parameter("lidar_smooth_enable").as_bool();
        lidar_smooth_radius_ = this->get_parameter("lidar_smooth_radius").as_double();

        // Initialize TF
        if (use_tf_) {
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

        // Initialize nvblox mapper
        initializeMapper();

        // Initialize lidar model
        lidar_ = std::make_unique<nvblox::Lidar>(
            lidar_num_azimuth_,
            lidar_num_elevation_,
            0.5f,  // min valid range
            static_cast<float>(lidar_vertical_fov_deg_ * M_PI / 180.0)
        );

        // Subscribers
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&NvbloxFastlioNode::pointcloudCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::SensorDataQoS(),
            std::bind(&NvbloxFastlioNode::odomCallback, this, std::placeholders::_1));

        // Publishers
        mesh_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "nvblox/mesh", 10);
        
        voxel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "nvblox/voxels", 10);

        if (publish_voxel_points_) {
            voxel_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "nvblox/voxel_points", 10);
        }

        esdf_slice_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "nvblox/esdf_slice", 10);

        occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "nvblox/occupancy_grid", 10);

        tsdf_slice_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "nvblox/tsdf_slice", 10);

        save_voxels_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "nvblox/save_voxels",
            std::bind(&NvbloxFastlioNode::handleSaveVoxelsService, this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3));

        // Timers for mesh and ESDF updates - created but callbacks check for data
        // Use a delayed start to allow system to stabilize
        if (publish_mesh_ && mesh_update_rate_hz_ > 0) {
            auto mesh_period = std::chrono::duration<double>(1.0 / mesh_update_rate_hz_);
            // Delay timer start by 2 seconds
            mesh_start_timer_ = this->create_wall_timer(
                std::chrono::seconds(2),
                [this, mesh_period]() {
                    mesh_start_timer_->cancel();
                    mesh_timer_ = this->create_wall_timer(
                        std::chrono::duration_cast<std::chrono::milliseconds>(mesh_period),
                        std::bind(&NvbloxFastlioNode::meshUpdateCallback, this));
                });
        }

        if (publish_esdf_slice_ && esdf_update_rate_hz_ > 0) {
            auto esdf_period = std::chrono::duration<double>(1.0 / esdf_update_rate_hz_);
            // Delay timer start by 3 seconds
            esdf_start_timer_ = this->create_wall_timer(
                std::chrono::seconds(3),
                [this, esdf_period]() {
                    esdf_start_timer_->cancel();
                    esdf_timer_ = this->create_wall_timer(
                        std::chrono::duration_cast<std::chrono::milliseconds>(esdf_period),
                        std::bind(&NvbloxFastlioNode::esdfUpdateCallback, this));
                });
        }

        RCLCPP_INFO(this->get_logger(), "nvblox_fastlio_node initialized");
        RCLCPP_INFO(this->get_logger(), "  Voxel size: %.3f m", voxel_size_);
        RCLCPP_INFO(this->get_logger(), "  Pointcloud topic: %s", pointcloud_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Odometry topic: %s", odom_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Voxel export path: %s", voxel_export_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Voxel export surface band: %.3f m", voxel_export_surface_band_);
    RCLCPP_INFO(this->get_logger(), "  Voxel export min weight: %.2f", voxel_export_min_weight_);
    RCLCPP_INFO(this->get_logger(), "  Voxel export surface only: %s", voxel_export_surface_only_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Voxel export points per side: %d", voxel_export_points_per_side_);
    RCLCPP_INFO(this->get_logger(), "  Publish voxel points: %s", publish_voxel_points_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Voxel marker alpha: %.2f", voxel_marker_alpha_);
        RCLCPP_INFO(this->get_logger(), "  Waiting for odometry and pointcloud data...");
    }

private:
    void initializeMapper()
    {
        // Create mapper with specified voxel size
        mapper_ = std::make_unique<nvblox::Mapper>(
            static_cast<float>(voxel_size_),
            nvblox::BlockMemoryPoolParams(),
            nvblox::ProjectiveLayerType::kTsdf
        );

        nvblox::MapperParams mapper_params;
        configureMapperParams(&mapper_params);
        mapper_->setMapperParams(mapper_params);

        RCLCPP_INFO(this->get_logger(),
                    "nvblox Mapper initialized (mode=%s, slice_height=%.2f m, thickness=%.2f m, window=[%.2f, %.2f])",
                    esdf_mode_.c_str(),
                    esdf_slice_height_,
                    esdf_slice_thickness_m_,
                    esdf_slice_min_height_,
                    esdf_slice_max_height_);
    }

    void dilateDepthImage(float* depth_data, int width, int height, int radius)
    {
        if (radius <= 0) return;

        // Use a temporary buffer to store dilated values
        std::vector<float> temp_buffer(width * height);
        
        // We can optimize this, but for now a simple implementation is fine
        // Iterate over all pixels
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;
                float val = depth_data[idx];

                // If pixel is valid, keep it
                if (val > 0.0f) {
                    temp_buffer[idx] = val;
                    continue;
                }

                // If pixel is invalid (hole), look for valid neighbors
                float min_depth = std::numeric_limits<float>::max();
                bool found = false;

                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {
                        int ny = y + dy;
                        int nx = x + dx;

                        if (ny >= 0 && ny < height && nx >= 0 && nx < width) {
                            float d = depth_data[ny * width + nx];
                            if (d > 0.0f) {
                                if (d < min_depth) {
                                    min_depth = d;
                                    found = true;
                                }
                            }
                        }
                    }
                }

                if (found) {
                    temp_buffer[idx] = min_depth;
                } else {
                    temp_buffer[idx] = 0.0f;
                }
            }
        }

        // Copy back
        std::memcpy(depth_data, temp_buffer.data(), width * height * sizeof(float));
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_ = *msg;
        has_odom_ = true;
    }

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "pointcloudCallback started, integration_count_=%d", integration_count_);
        
        if (!has_odom_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No odometry received yet, skipping pointcloud");
            return;
        }

        if (!mapper_ || !lidar_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Mapper or lidar not initialized");
            return;
        }

        try {
            RCLCPP_DEBUG(this->get_logger(), "Getting transform...");
            
            // Get the transform at pointcloud time
            nvblox::Transform T_L_C;
            bool transform_found = false;

            if (use_tf_ && tf_buffer_) {
                try {
                    // Look up transform from frame_id (map) to child_frame_id (body/lidar)
                    // We need the transform that takes points from Lidar frame to Map frame (T_M_L)
                    // In nvblox terms, T_L_C usually means Global(L) to Camera(C)? 
                    // Wait, nvblox::integrateDepth takes T_L_C where L is Layer (Global) and C is Camera (Sensor).
                    // So we need Map -> Sensor.
                    
                    // FAST-LIO publishes Map -> Body.
                    // We need to know the child frame. Usually it's in the odometry message.
                    std::string child_frame = latest_odom_.child_frame_id;
                    if (child_frame.empty()) child_frame = "body"; // Fallback

                    geometry_msgs::msg::TransformStamped t_stamped = 
                        tf_buffer_->lookupTransform(frame_id_, child_frame, 
                                                  msg->header.stamp, 
                                                  std::chrono::duration<double>(tf_timeout_sec_));
                    
                    Eigen::Quaternionf q(t_stamped.transform.rotation.w,
                                       t_stamped.transform.rotation.x,
                                       t_stamped.transform.rotation.y,
                                       t_stamped.transform.rotation.z);
                    Eigen::Vector3f t(t_stamped.transform.translation.x,
                                    t_stamped.transform.translation.y,
                                    t_stamped.transform.translation.z);
                    
                    T_L_C = nvblox::Transform::Identity();
                    T_L_C.linear() = q.toRotationMatrix();
                    T_L_C.translation() = t;
                    transform_found = true;
                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "TF lookup failed: %s. Falling back to latest odometry.", ex.what());
                }
            }

            if (!transform_found) {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                // Convert odometry to nvblox Transform (Eigen Isometry3f)
                const auto& pos = latest_odom_.pose.pose.position;
                const auto& quat = latest_odom_.pose.pose.orientation;
                
                Eigen::Quaternionf q(quat.w, quat.x, quat.y, quat.z);
                Eigen::Vector3f t(pos.x, pos.y, pos.z);
                
                T_L_C = nvblox::Transform::Identity();
                T_L_C.linear() = q.toRotationMatrix();
                T_L_C.translation() = t;
            }
            
            // Inverse transform to convert global points to lidar frame
            nvblox::Transform T_C_L = T_L_C.inverse();

            RCLCPP_DEBUG(this->get_logger(), "Converting ROS to PCL...");
            
            // Convert ROS pointcloud to PCL
            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*msg, *pcl_cloud);

            if (pcl_cloud->empty()) {
                RCLCPP_DEBUG(this->get_logger(), "Empty cloud, returning");
                return;
            }

            // Manual NaN removal to be absolutely sure (pcl::removeNaNFromPointCloud can be flaky with some types)
            pcl::PointCloud<pcl::PointXYZI>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            clean_cloud->header = pcl_cloud->header;
            clean_cloud->reserve(pcl_cloud->size());
            
            for (const auto& pt : pcl_cloud->points) {
                if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                    clean_cloud->push_back(pt);
                }
            }
            pcl_cloud = clean_cloud;

            if (pcl_cloud->empty()) {
                RCLCPP_DEBUG(this->get_logger(), "Empty cloud after NaN removal, returning");
                return;
            }

            // Denoise (Statistical Outlier Removal)
            if (lidar_denoise_enable_) {
                pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
                sor.setInputCloud(pcl_cloud);
                sor.setMeanK(lidar_denoise_k_);
                sor.setStddevMulThresh(lidar_denoise_stddev_);
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
                sor.filter(*cloud_filtered);
                pcl_cloud = cloud_filtered;
                RCLCPP_DEBUG(this->get_logger(), "Denoised cloud: %zu points", pcl_cloud->size());
            }

            // Smooth (Moving Least Squares)
            if (lidar_smooth_enable_) {
                pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI> mls;
                mls.setInputCloud(pcl_cloud);
                mls.setPolynomialOrder(2);
                mls.setSearchRadius(lidar_smooth_radius_);
                mls.setComputeNormals(false);
                
                pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
                mls.setSearchMethod(tree);
                
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZI>);
                mls.process(*cloud_smoothed);
                pcl_cloud = cloud_smoothed;
                RCLCPP_DEBUG(this->get_logger(), "Smoothed cloud: %zu points", pcl_cloud->size());
            }

            RCLCPP_DEBUG(this->get_logger(), "Cloud size: %zu points", pcl_cloud->size());

            // Check lidar dimensions
            const int width = lidar_->width();
            const int height = lidar_->height();
            
            RCLCPP_DEBUG(this->get_logger(), "Lidar dimensions: %d x %d", width, height);
            
            if (width <= 0 || height <= 0) {
                RCLCPP_ERROR_ONCE(this->get_logger(), "Invalid lidar dimensions: %d x %d", width, height);
                return;
            }

            RCLCPP_DEBUG(this->get_logger(), "Allocating depth image (host)...");
            
            // Create depth image on host memory so GPU copies are managed internally
            nvblox::DepthImage depth_image(height, width, nvblox::MemoryType::kHost);
            
            RCLCPP_DEBUG(this->get_logger(), "Getting depth data pointer...");
            
            float* depth_data = depth_image.dataPtr();
            if (!depth_data) {
                RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to get depth image data pointer");
                return;
            }
            
            RCLCPP_DEBUG(this->get_logger(), "Clearing depth image with memset...");
            std::memset(depth_data, 0, width * height * sizeof(float));

            RCLCPP_DEBUG(this->get_logger(), "Projecting pointcloud to depth image...");
            
            // Project pointcloud to depth image
            int valid_points = 0;
            for (const auto& pt : pcl_cloud->points) {
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                    continue;
                }

                // Transform point from Global (Map) to Lidar frame
                nvblox::Vector3f point_global(pt.x, pt.y, pt.z);
                nvblox::Vector3f point_lidar = T_C_L * point_global;

                nvblox::Index2D pixel_idx;
                
                if (lidar_->project(point_lidar, &pixel_idx)) {
                    const float depth = lidar_->getDepth(point_lidar);
                    if (depth > 0.5f && depth < max_integration_distance_) {
                        const int idx = pixel_idx.y() * width + pixel_idx.x();
                        if (idx >= 0 && idx < width * height) {
                            // Keep closest point
                            if (depth_data[idx] <= 0.0f || depth < depth_data[idx]) {
                                depth_data[idx] = depth;
                                valid_points++;
                            }
                        }
                    }
                }
            }

            RCLCPP_DEBUG(this->get_logger(), "Valid points projected: %d", valid_points);

            if (valid_points < 100) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Too few valid points projected: %d", valid_points);
                return;
            }

            // Apply hole filling/dilation if requested
            if (lidar_hole_filling_radius_ > 0) {
                RCLCPP_DEBUG(this->get_logger(), "Dilating depth image (radius: %d)...", lidar_hole_filling_radius_);
                dilateDepthImage(depth_data, width, height, lidar_hole_filling_radius_);
            }

            RCLCPP_DEBUG(this->get_logger(), "Integrating into TSDF...");
            
            // Integrate into TSDF
            {
                std::lock_guard<std::mutex> lock(mapper_mutex_);
                mapper_->integrateDepth(depth_image, T_L_C, *lidar_);
            }

            // Ensure GPU work finishes before the stack-allocated depth image goes out of scope
            cudaError_t cuda_status = cudaDeviceSynchronize();
            if (cuda_status != cudaSuccess) {
                RCLCPP_WARN(this->get_logger(),
                    "cudaDeviceSynchronize failed after integrateDepth: %s",
                    cudaGetErrorString(cuda_status));
            }

            RCLCPP_DEBUG(this->get_logger(), "Integration complete");

            integration_count_++;
            if (integration_count_ == 1) {
                RCLCPP_INFO(this->get_logger(), "First pointcloud integrated successfully!");
            } else if (integration_count_ % 50 == 0) {
                RCLCPP_INFO(this->get_logger(), "Integrated %d pointclouds", integration_count_);
            }
            
            RCLCPP_DEBUG(this->get_logger(), "pointcloudCallback complete");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                "Pointcloud callback exception: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Pointcloud callback unknown exception");
        }
    }

    void meshUpdateCallback()
    {
        // Need at least a few integrations before mesh can be generated
        if (integration_count_ < 5 || !mapper_) {
            return;
        }

        // Check for subscribers to avoid expensive work
        bool has_mesh_subs = mesh_pub_ && mesh_pub_->get_subscription_count() > 0;
        bool has_voxel_subs = voxel_pub_ && voxel_pub_->get_subscription_count() > 0;
        bool has_voxel_point_subs = publish_voxel_points_ && voxel_points_pub_ && voxel_points_pub_->get_subscription_count() > 0;

        try {
            {
                std::lock_guard<std::mutex> lock(mapper_mutex_);
                // CRITICAL: Use incremental update (kNo) instead of full update (kYes)
                // This prevents the update time from growing linearly with map size
                mapper_->updateColorMesh(nvblox::UpdateFullLayer::kNo);
                
                if (publish_mesh_ && has_mesh_subs) {
                    // Serialize mesh manually since Mapper might not be doing it
                    const auto& mesh_layer = mapper_->color_mesh_layer();
                    auto block_indices = mesh_layer.getAllBlockIndices();
                    // Use a temporary stream for serialization
                    mesh_serializer_.serialize(mesh_layer, block_indices, nvblox::CudaStreamOwning());
                }
            }

            // Publish mesh
            if (publish_mesh_ && has_mesh_subs) {
                publishMesh();
            }
            
            // Publish voxels
            if (publish_voxels_ && (has_voxel_subs || has_voxel_point_subs)) {
                publishVoxelGrid();
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Mesh update failed: %s", e.what());
        }
    }

    void esdfUpdateCallback()
    {
        // Need at least a few integrations before ESDF can be computed
        if (integration_count_ < 5 || !mapper_) {
            return;
        }

        bool has_slice_subs = esdf_slice_pub_ && esdf_slice_pub_->get_subscription_count() > 0;
        bool has_grid_subs = occupancy_grid_pub_ && occupancy_grid_pub_->get_subscription_count() > 0;

        // Only update ESDF if we have subscribers
        if (!has_slice_subs && !has_grid_subs) {
            return;
        }

        try {
            {
                std::lock_guard<std::mutex> lock(mapper_mutex_);
                mapper_->updateEsdf();
            }

            // Publish ESDF slice
            if (publish_esdf_slice_ || publish_occupancy_grid_) {
                publishEsdfSlice();
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "ESDF update failed: %s", e.what());
        }
    }

    void publishMesh()
    {
        if (!mesh_pub_) {
            return;
        }

        std::shared_ptr<const nvblox::SerializedColorMeshLayer> serialized_mesh;
        {
            std::lock_guard<std::mutex> lock(mapper_mutex_);
            if (!mapper_) {
                return;
            }
            // serialized_mesh = mapper_->serializedColorMeshLayer();
            serialized_mesh = mesh_serializer_.getSerializedLayer();
        }

        if (!serialized_mesh || serialized_mesh->vertices.empty() ||
            serialized_mesh->triangle_indices.empty() ||
            serialized_mesh->block_indices.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Serialized mesh is empty (Vertices: %zu, Triangles: %zu, Blocks: %zu)",
                serialized_mesh ? serialized_mesh->vertices.size() : 0,
                serialized_mesh ? serialized_mesh->triangle_indices.size() : 0,
                serialized_mesh ? serialized_mesh->block_indices.size() : 0);
            return;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Publishing mesh: %zu vertices, %zu blocks", 
            serialized_mesh->vertices.size(), serialized_mesh->block_indices.size());

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = this->now();
        marker.ns = "nvblox_mesh";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        try {
            const size_t num_blocks = serialized_mesh->block_indices.size();
            for (size_t block_i = 0; block_i < num_blocks; ++block_i) {
                const size_t num_triangle_indices =
                    serialized_mesh->getNumTriangleIndicesInBlock(block_i);
                if (num_triangle_indices < 3) {
                    continue;
                }

                for (size_t tri_offset = 0; tri_offset + 2 < num_triangle_indices;
                     tri_offset += 3) {
                    for (int corner = 0; corner < 3; ++corner) {
                        const int vertex_index = serialized_mesh->getTriangleIndex(
                            block_i, tri_offset + corner);
                        if (vertex_index < 0 ||
                            vertex_index >=
                                static_cast<int>(serialized_mesh->vertices.size())) {
                            continue;
                        }

                        const auto& vertex =
                            serialized_mesh->vertices[vertex_index];

                        geometry_msgs::msg::Point pt;
                        pt.x = vertex.x();
                        pt.y = vertex.y();
                        pt.z = vertex.z();
                        marker.points.push_back(pt);

                        std_msgs::msg::ColorRGBA color;
                        if (vertex_index <
                            static_cast<int>(
                                serialized_mesh->vertex_appearances.size())) {
                            const auto& appearance =
                                serialized_mesh->vertex_appearances[vertex_index];
                            color.r = appearance.r() / 255.0f;
                            color.g = appearance.g() / 255.0f;
                            color.b = appearance.b() / 255.0f;
                            color.a = 1.0f;
                        } else {
                            color.r = 0.7f;
                            color.g = 0.7f;
                            color.b = 0.7f;
                            color.a = 1.0f;
                        }
                        marker.colors.push_back(color);
                    }
                }
            }

            if (!marker.points.empty()) {
                mesh_pub_->publish(marker);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Mesh publish failed: %s", e.what());
        }
    }

    void publishEsdfSlice()
    {
        if (!mapper_) return;
        
        try {
            // Collect valid ESDF voxels respecting slice configuration
            struct EsdfPoint {
                float x, y, z, dist;
            };
            std::vector<EsdfPoint> slice_points;

            const float voxel_size = static_cast<float>(voxel_size_);
            const float block_size = mapper_->esdf_layer().block_size();
            const float slice_center = static_cast<float>(
                esdf_slice_height_ + esdf_slice_height_above_plane_m_);
            const float slice_thickness = static_cast<float>(
                std::max(esdf_slice_thickness_m_, voxel_size_));
            const float half_slice_thickness = slice_thickness * 0.5f;
            const float slice_lower = slice_center - half_slice_thickness;
            const float slice_upper = slice_center + half_slice_thickness;
            const float volume_min_z = static_cast<float>(esdf_slice_min_height_);
            const float volume_max_z = static_cast<float>(esdf_slice_max_height_);
            const bool publish_volume = esdf_mode_is_3d_;

            // Bounds for occupancy grid
            float min_x = std::numeric_limits<float>::max();
            float max_x = std::numeric_limits<float>::lowest();
            float min_y = std::numeric_limits<float>::max();
            float max_y = std::numeric_limits<float>::lowest();

            auto lambda = [&slice_points, &min_x, &max_x, &min_y, &max_y,
                           slice_lower, slice_upper, volume_min_z,
                           volume_max_z, publish_volume, voxel_size, block_size](
                const nvblox::Index3D& block_index, const nvblox::Index3D& voxel_index,
                const nvblox::EsdfVoxel* voxel) {
                
                if (!voxel || !voxel->observed) return;

                Eigen::Vector3f position = nvblox::getCenterPositionFromBlockIndexAndVoxelIndex(
                    block_size, block_index, voxel_index);

                bool within_band = false;
                if (publish_volume) {
                    within_band = position.z() >= volume_min_z &&
                                  position.z() <= volume_max_z;
                } else {
                    within_band = position.z() >= slice_lower &&
                                  position.z() <= slice_upper;
                }

                if (!within_band) {
                    return;
                }

                float dist = std::sqrt(voxel->squared_distance_vox) * voxel_size;
                if (voxel->is_inside) {
                    dist = -dist;
                }

                slice_points.push_back({position.x(), position.y(), position.z(), dist});

                min_x = std::min(min_x, position.x());
                max_x = std::max(max_x, position.x());
                min_y = std::min(min_y, position.y());
                max_y = std::max(max_y, position.y());
            };

            {
                std::lock_guard<std::mutex> lock(mapper_mutex_);
                nvblox::callFunctionOnAllVoxels<nvblox::EsdfVoxel>(
                    mapper_->esdf_layer(), lambda);
            }

            if (slice_points.empty()) {
                return;
            }

            // Publish PointCloud2
            if (publish_esdf_slice_ && esdf_slice_pub_) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr esdf_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                esdf_cloud->reserve(slice_points.size());
                
                for (const auto& p : slice_points) {
                    pcl::PointXYZI pt;
                    pt.x = p.x;
                    pt.y = p.y;
                    pt.z = p.z;
                    pt.intensity = p.dist;
                    esdf_cloud->push_back(pt);
                }

                sensor_msgs::msg::PointCloud2 cloud_msg;
                pcl::toROSMsg(*esdf_cloud, cloud_msg);
                cloud_msg.header.frame_id = frame_id_;
                cloud_msg.header.stamp = this->now();
                esdf_slice_pub_->publish(cloud_msg);
            }

            // Publish OccupancyGrid
            if (publish_occupancy_grid_ && occupancy_grid_pub_) {
                nav_msgs::msg::OccupancyGrid grid_msg;
                grid_msg.header.frame_id = frame_id_;
                grid_msg.header.stamp = this->now();
                
                // Add some padding
                float padding = 1.0f; // meters
                min_x -= padding;
                min_y -= padding;
                max_x += padding;
                max_y += padding;

                int width = static_cast<int>((max_x - min_x) / voxel_size);
                int height = static_cast<int>((max_y - min_y) / voxel_size);
                
                if (width <= 0 || height <= 0) return;

                grid_msg.info.resolution = voxel_size;
                grid_msg.info.width = width;
                grid_msg.info.height = height;
                grid_msg.info.origin.position.x = min_x;
                grid_msg.info.origin.position.y = min_y;
                grid_msg.info.origin.position.z = publish_volume ? esdf_slice_min_height_ : slice_lower;
                grid_msg.info.origin.orientation.w = 1.0;

                // Initialize with -1 (unknown)
                grid_msg.data.assign(width * height, -1);

                for (const auto& p : slice_points) {
                    int gx = static_cast<int>((p.x - min_x) / voxel_size);
                    int gy = static_cast<int>((p.y - min_y) / voxel_size);
                    
                    if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
                        int idx = gy * width + gx;
                        // Simple occupancy logic:
                        // If distance <= 0 (inside) or very close to surface -> Occupied (100)
                        // If distance > 0 -> Free (0)
                        // We use a small threshold for surface
                        if (p.dist <= voxel_size) {
                            grid_msg.data[idx] = 100;
                        } else {
                            // Only mark free if not already marked occupied (conservative)
                            if (grid_msg.data[idx] != 100) {
                                grid_msg.data[idx] = 0;
                            }
                        }
                    }
                }
                
                occupancy_grid_pub_->publish(grid_msg);
            }

        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "ESDF slice publish failed: %s", e.what());
        }
    }

    void publishVoxelGrid()
    {
        if (!voxel_pub_ || !mapper_) {
            return;
        }

        // Check subscribers
        bool has_voxel_subs = voxel_pub_->get_subscription_count() > 0;
        bool has_voxel_point_subs = publish_voxel_points_ && voxel_points_pub_ && voxel_points_pub_->get_subscription_count() > 0;
        
        if (!has_voxel_subs && !has_voxel_point_subs) return;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = this->now();
        marker.ns = "nvblox_voxels";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = voxel_size_;
        marker.scale.y = voxel_size_;
        marker.scale.z = voxel_size_;
        marker.color.a = voxel_marker_alpha_;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        std::vector<geometry_msgs::msg::Point> points;
        std::vector<std_msgs::msg::ColorRGBA> colors;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_point_cloud;
        if (publish_voxel_points_ && voxel_points_pub_) {
            voxel_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        }

        const float block_size = mapper_->tsdf_layer().block_size();
        const float surface_band = static_cast<float>(voxel_export_surface_band_);
        const float min_weight = static_cast<float>(voxel_export_min_weight_);
        const bool surface_only = voxel_export_surface_only_;

        auto lambda = [&points, &colors, voxel_point_cloud, surface_band, min_weight,
                       block_size, surface_only, this](
            const nvblox::Index3D& block_index, const nvblox::Index3D& voxel_index,
            const nvblox::TsdfVoxel* voxel) {
            
            if (voxel->weight >= min_weight && (!surface_only || std::abs(voxel->distance) <= surface_band)) {
                Eigen::Vector3f position = nvblox::getCenterPositionFromBlockIndexAndVoxelIndex(
                    block_size, block_index, voxel_index);
                
                geometry_msgs::msg::Point pt;
                pt.x = position.x();
                pt.y = position.y();
                pt.z = position.z();
                points.push_back(pt);

                // Color
                std_msgs::msg::ColorRGBA color;
                color.a = static_cast<float>(this->voxel_marker_alpha_);
                
                if (this->voxel_color_mode_ == "flat") {
                    color.r = static_cast<float>(this->voxel_flat_color_[0]);
                    color.g = static_cast<float>(this->voxel_flat_color_[1]);
                    color.b = static_cast<float>(this->voxel_flat_color_[2]);
                } else {
                    // Simple height map coloring
                    float h = position.z();
                    color.r = std::fmin(1.0f, std::fmax(0.0f, (h + 2.0f) / 4.0f));
                    color.g = std::fmin(1.0f, std::fmax(0.0f, 1.0f - std::abs(h) / 2.0f));
                    color.b = std::fmin(1.0f, std::fmax(0.0f, (2.0f - h) / 4.0f));
                }
                colors.push_back(color);

                if (voxel_point_cloud) {
                    pcl::PointXYZRGB cloud_point;
                    cloud_point.x = position.x();
                    cloud_point.y = position.y();
                    cloud_point.z = position.z();
                    cloud_point.r = static_cast<uint8_t>(color.r * 255.0f);
                    cloud_point.g = static_cast<uint8_t>(color.g * 255.0f);
                    cloud_point.b = static_cast<uint8_t>(color.b * 255.0f);
                    voxel_point_cloud->push_back(cloud_point);
                }
            }
        };

        {
            std::lock_guard<std::mutex> lock(mapper_mutex_);
            nvblox::callFunctionOnAllVoxels<nvblox::TsdfVoxel>(
                mapper_->tsdf_layer(), lambda);
        }

        if (points.empty()) {
            if (surface_only) {
                RCLCPP_DEBUG_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Voxel filter removed all cells. Try lowering voxel_export_min_weight (%.2f) or increasing surface band (%.3f m)",
                    voxel_export_min_weight_, voxel_export_surface_band_);
            } else {
                RCLCPP_DEBUG_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "No voxels met min weight %.2f for visualization.",
                    voxel_export_min_weight_);
            }
            return;
        }

        marker.points = points;
        marker.colors = colors;
        voxel_pub_->publish(marker);

        if (voxel_point_cloud && !voxel_point_cloud->empty()) {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*voxel_point_cloud, cloud_msg);
            cloud_msg.header.frame_id = frame_id_;
            cloud_msg.header.stamp = this->now();
            voxel_points_pub_->publish(cloud_msg);
        }
    }

    bool saveVoxelsToPcd(const std::string& file_path,
                         size_t* points_saved = nullptr,
                         std::string* error_message = nullptr)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        float block_size = 0.0f;
        const float surface_band = static_cast<float>(voxel_export_surface_band_);
        const float min_weight = static_cast<float>(voxel_export_min_weight_);
    const bool surface_only = voxel_export_surface_only_;
    const int samples_per_side = std::max(1, voxel_export_points_per_side_);
    const float voxel_size = static_cast<float>(voxel_size_);
    const float sample_spacing = voxel_size / static_cast<float>(samples_per_side);
    const float sample_offset = -0.5f * voxel_size + 0.5f * sample_spacing;

        {
            std::lock_guard<std::mutex> lock(mapper_mutex_);
            if (!mapper_) {
                if (error_message) {
                    *error_message = "Mapper not initialized";
                }
                return false;
            }

            const auto& tsdf_layer = mapper_->tsdf_layer();
            block_size = tsdf_layer.block_size();

            nvblox::callFunctionOnAllVoxels<nvblox::TsdfVoxel>(
                tsdf_layer,
                [&voxel_cloud, block_size, surface_band, min_weight, surface_only,
                 samples_per_side, sample_spacing, sample_offset](const nvblox::Index3D& block_index,
                                           const nvblox::Index3D& voxel_index,
                                           const nvblox::TsdfVoxel* voxel) {
                    if (!voxel || voxel->weight < min_weight ||
                        (surface_only && std::abs(voxel->distance) > surface_band)) {
                        return;
                    }

                    const Eigen::Vector3f position =
                        nvblox::getCenterPositionFromBlockIndexAndVoxelIndex(
                            block_size, block_index, voxel_index);

                    if (samples_per_side == 1) {
                        pcl::PointXYZI point;
                        point.x = position.x();
                        point.y = position.y();
                        point.z = position.z();
                        point.intensity = voxel->distance;
                        voxel_cloud->push_back(point);
                        return;
                    }

                    for (int ix = 0; ix < samples_per_side; ++ix) {
                        for (int iy = 0; iy < samples_per_side; ++iy) {
                            for (int iz = 0; iz < samples_per_side; ++iz) {
                                pcl::PointXYZI point;
                                point.x = position.x() + sample_offset + ix * sample_spacing;
                                point.y = position.y() + sample_offset + iy * sample_spacing;
                                point.z = position.z() + sample_offset + iz * sample_spacing;
                                point.intensity = voxel->distance;
                                voxel_cloud->push_back(point);
                            }
                        }
                    }
                });
        }

        if (voxel_cloud->empty()) {
            if (surface_only) {
                RCLCPP_WARN(this->get_logger(),
                            "Voxel export filter removed all voxels (min_weight=%.2f, surface_band=%.3f m).",
                            voxel_export_min_weight_, voxel_export_surface_band_);
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "Voxel export removed all voxels (min_weight=%.2f). Consider lowering the threshold.",
                            voxel_export_min_weight_);
            }
            if (error_message) {
                *error_message = "TSDF layer empty or no valid voxels";
            }
            return false;
        }

        try {
            std::filesystem::path output_path(file_path);
            if (output_path.has_parent_path() && !output_path.parent_path().empty()) {
                std::filesystem::create_directories(output_path.parent_path());
            }
        } catch (const std::exception& e) {
            if (error_message) {
                *error_message = std::string("Failed to prepare output path: ") + e.what();
            }
            return false;
        }

        try {
            if (pcl::io::savePCDFileBinary(file_path, *voxel_cloud) != 0) {
                if (error_message) {
                    *error_message = "Failed to write PCD file";
                }
                return false;
            }
        } catch (const std::exception& e) {
            if (error_message) {
                *error_message = std::string("PCD export error: ") + e.what();
            }
            return false;
        }

        if (points_saved) {
            *points_saved = voxel_cloud->size();
        }

        return true;
    }

    void handleSaveVoxelsService(
        const std::shared_ptr<rmw_request_id_t> /*request_header*/,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        size_t point_count = 0;
        std::string error;

        const bool success = saveVoxelsToPcd(voxel_export_path_, &point_count, &error);

        response->success = success;
        if (success) {
            response->message = "Saved " + std::to_string(point_count) +
                                 " voxels to " + voxel_export_path_;
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        } else {
            if (error.empty()) {
                error = "Unknown error";
            }
            response->message = "Failed to save voxels: " + error;
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        }
    }

    template <typename T>
    void declareIfNotDeclared(const std::string& name, const T& default_value)
    {
        if (!this->has_parameter(name)) {
            this->declare_parameter<T>(name, default_value);
        }
    }

    double getDoubleParam(const std::initializer_list<std::string>& names,
                          double default_value)
    {
        for (const auto& name : names) {
            double value_double;
            if (this->get_parameter(name, value_double)) {
                return value_double;
            }
            int64_t value_int;
            if (this->get_parameter(name, value_int)) {
                return static_cast<double>(value_int);
            }
        }

        const std::string primary = *names.begin();
        if (this->has_parameter(primary)) {
            double value_double;
            if (this->get_parameter(primary, value_double)) {
                return value_double;
            }
            int64_t value_int;
            if (this->get_parameter(primary, value_int)) {
                return static_cast<double>(value_int);
            }
            std::string value_string;
            if (this->get_parameter(primary, value_string)) {
                try {
                    return std::stod(value_string);
                } catch (const std::exception&) {
                    // fall through
                }
            }
        }

        return this->declare_parameter<double>(primary, default_value);
    }

    int getIntParam(const std::initializer_list<std::string>& names,
                    int default_value)
    {
        for (const auto& name : names) {
            int64_t value_int;
            if (this->get_parameter(name, value_int)) {
                return static_cast<int>(value_int);
            }
            double value_double;
            if (this->get_parameter(name, value_double)) {
                return static_cast<int>(std::lround(value_double));
            }
        }

        const std::string primary = *names.begin();
        if (this->has_parameter(primary)) {
            int64_t value_int;
            if (this->get_parameter(primary, value_int)) {
                return static_cast<int>(value_int);
            }
            double value_double;
            if (this->get_parameter(primary, value_double)) {
                return static_cast<int>(std::lround(value_double));
            }
            std::string value_string;
            if (this->get_parameter(primary, value_string)) {
                try {
                    return std::stoi(value_string);
                } catch (const std::exception&) {
                }
            }
        }

        return this->declare_parameter<int>(primary, default_value);
    }

    bool getBoolParam(const std::initializer_list<std::string>& names,
                      bool default_value)
    {
        for (const auto& name : names) {
            bool value_bool;
            if (this->get_parameter(name, value_bool)) {
                return value_bool;
            }
            int64_t value_int;
            if (this->get_parameter(name, value_int)) {
                return value_int != 0;
            }
            std::string value_string;
            if (this->get_parameter(name, value_string)) {
                auto normalized = normalizeToken(value_string);
                if (normalized == "true" || normalized == "1") {
                    return true;
                }
                if (normalized == "false" || normalized == "0") {
                    return false;
                }
            }
        }

        const std::string primary = *names.begin();
        if (this->has_parameter(primary)) {
            bool value_bool;
            if (this->get_parameter(primary, value_bool)) {
                return value_bool;
            }
            int64_t value_int;
            if (this->get_parameter(primary, value_int)) {
                return value_int != 0;
            }
            std::string value_string;
            if (this->get_parameter(primary, value_string)) {
                auto normalized = normalizeToken(value_string);
                if (normalized == "true" || normalized == "1") {
                    return true;
                }
                if (normalized == "false" || normalized == "0") {
                    return false;
                }
            }
        }

        return this->declare_parameter<bool>(primary, default_value);
    }

    std::string getStringParam(const std::initializer_list<std::string>& names,
                               const std::string& default_value)
    {
        for (const auto& name : names) {
            std::string value;
            if (this->get_parameter(name, value)) {
                return value;
            }
        }

        const std::string primary = *names.begin();
        if (this->has_parameter(primary)) {
            std::string value;
            if (this->get_parameter(primary, value)) {
                return value;
            }
        }

        return this->declare_parameter<std::string>(primary, default_value);
    }

    std::vector<double> getDoubleVectorParam(
        const std::initializer_list<std::string>& names,
        const std::vector<double>& default_value)
    {
        for (const auto& name : names) {
            std::vector<double> values_double;
            if (this->get_parameter(name, values_double)) {
                return values_double;
            }
            std::vector<int64_t> values_long;
            if (this->get_parameter(name, values_long)) {
                std::vector<double> converted;
                converted.reserve(values_long.size());
                for (const auto& v : values_long) {
                    converted.push_back(static_cast<double>(v));
                }
                if (!converted.empty()) {
                    return converted;
                }
            }
        }

        const std::string primary = *names.begin();
        if (this->has_parameter(primary)) {
            std::vector<double> values_double;
            if (this->get_parameter(primary, values_double)) {
                return values_double;
            }
        }

        return this->declare_parameter<std::vector<double>>(primary, default_value);
    }

    nvblox::Time getTimeParamMs(const std::initializer_list<std::string>& names,
                                double default_ms)
    {
        const double value = getDoubleParam(names, default_ms);
        return nvblox::Time(static_cast<int64_t>(value));
    }

    std::string normalizeToken(const std::string& value) const
    {
        std::string normalized = value;
        std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return normalized;
    }

    nvblox::WeightingFunctionType parseWeightingMode(
        const std::string& token,
        nvblox::WeightingFunctionType fallback) const
    {
        static const std::unordered_map<std::string, nvblox::WeightingFunctionType> kStringToMode = {
            {"constant", nvblox::WeightingFunctionType::kConstantWeight},
            {"kconstantweight", nvblox::WeightingFunctionType::kConstantWeight},
            {"constant_dropoff", nvblox::WeightingFunctionType::kConstantDropoffWeight},
            {"kconstantdropoffweight", nvblox::WeightingFunctionType::kConstantDropoffWeight},
            {"inverse_square", nvblox::WeightingFunctionType::kInverseSquareWeight},
            {"kinversesquareweight", nvblox::WeightingFunctionType::kInverseSquareWeight},
            {"inverse_square_dropoff", nvblox::WeightingFunctionType::kInverseSquareDropoffWeight},
            {"kinversesquaredropoffweight", nvblox::WeightingFunctionType::kInverseSquareDropoffWeight},
            {"inverse_square_tsdf_distance_penalty",
             nvblox::WeightingFunctionType::kInverseSquareTsdfDistancePenalty},
            {"kinversesquaretsdfdistancepenalty",
             nvblox::WeightingFunctionType::kInverseSquareTsdfDistancePenalty},
            {"linear_with_max", nvblox::WeightingFunctionType::kLinearWithMax},
            {"klinearwithmax", nvblox::WeightingFunctionType::kLinearWithMax}
        };

        const auto normalized = normalizeToken(token);
        const auto it = kStringToMode.find(normalized);
        if (it != kStringToMode.end()) {
            return it->second;
        }

        try {
            const int numeric = std::stoi(token);
            switch (numeric) {
                case 0:
                    return nvblox::WeightingFunctionType::kConstantWeight;
                case 1:
                    return nvblox::WeightingFunctionType::kConstantDropoffWeight;
                case 2:
                    return nvblox::WeightingFunctionType::kInverseSquareWeight;
                case 3:
                    return nvblox::WeightingFunctionType::kInverseSquareDropoffWeight;
                case 4:
                    return nvblox::WeightingFunctionType::kInverseSquareTsdfDistancePenalty;
                case 5:
                    return nvblox::WeightingFunctionType::kLinearWithMax;
                default:
                    break;
            }
        } catch (const std::exception&) {
            // ignore
        }

        return fallback;
    }

    nvblox::WorkspaceBoundsType parseWorkspaceBoundsType(
        const std::string& token,
        nvblox::WorkspaceBoundsType fallback) const
    {
        const auto normalized = normalizeToken(token);
        if (normalized == "unbounded" || normalized == "kunbounded") {
            return nvblox::WorkspaceBoundsType::kUnbounded;
        }
        if (normalized == "height_bounds" || normalized == "kheightbounds") {
            return nvblox::WorkspaceBoundsType::kHeightBounds;
        }
        if (normalized == "bounding_box" || normalized == "kboundingbox") {
            return nvblox::WorkspaceBoundsType::kBoundingBox;
        }

        return fallback;
    }

    void configureMapperParams(nvblox::MapperParams* mapper_params)
    {
        if (!mapper_params) {
            return;
        }

        mapper_params->do_depth_preprocessing.set(
            getBoolParam({"mapper.do_depth_preprocessing", "do_depth_preprocessing"},
                         mapper_params->do_depth_preprocessing.get()));

        mapper_params->depth_preprocessing_num_dilations.set(
            getIntParam({"mapper.depth_preprocessing_num_dilations",
                         "depth_preprocessing_num_dilations"},
                        mapper_params->depth_preprocessing_num_dilations.get()));

        auto& esdf_params = mapper_params->esdf_integrator_params;
        esdf_params.esdf_integrator_max_distance_m.set(static_cast<float>(
            getDoubleParam({"mapper.esdf_integrator_max_distance_m",
                            "esdf_integrator_max_distance_m"},
                           esdf_params.esdf_integrator_max_distance_m.get())));
        esdf_params.esdf_integrator_min_weight.set(static_cast<float>(
            getDoubleParam({"mapper.esdf_integrator_min_weight",
                            "esdf_integrator_min_weight"},
                           esdf_params.esdf_integrator_min_weight.get())));
        esdf_params.esdf_integrator_max_site_distance_vox.set(static_cast<float>(
            getDoubleParam({"mapper.esdf_integrator_max_site_distance_vox",
                            "esdf_integrator_max_site_distance_vox"},
                           esdf_params.esdf_integrator_max_site_distance_vox.get())));
        esdf_params.esdf_slice_min_height.set(static_cast<float>(
            getDoubleParam({"mapper.esdf_slice_min_height", "esdf_slice_min_height"},
                           esdf_params.esdf_slice_min_height.get())));
        esdf_params.esdf_slice_max_height.set(static_cast<float>(
            getDoubleParam({"mapper.esdf_slice_max_height", "esdf_slice_max_height"},
                           esdf_params.esdf_slice_max_height.get())));
        esdf_params.esdf_slice_height.set(static_cast<float>(
            getDoubleParam({"mapper.esdf_slice_height", "esdf_slice_height"},
                           esdf_params.esdf_slice_height.get())));
        esdf_params.slice_height_above_plane_m.set(static_cast<float>(
            getDoubleParam({"mapper.slice_height_above_plane_m",
                            "slice_height_above_plane_m"},
                           esdf_params.slice_height_above_plane_m.get())));
        esdf_params.slice_height_thickness_m.set(static_cast<float>(
            getDoubleParam({"mapper.slice_height_thickness_m",
                            "slice_height_thickness_m"},
                           esdf_params.slice_height_thickness_m.get())));

        auto& projective_params = mapper_params->projective_integrator_params;
        projective_params.projective_integrator_max_integration_distance_m.set(
            static_cast<float>(getDoubleParam(
                {"mapper.projective_integrator_max_integration_distance_m",
                 "projective_integrator_max_integration_distance_m"},
                max_integration_distance_)));
        projective_params.lidar_projective_integrator_max_integration_distance_m
            .set(static_cast<float>(getDoubleParam(
                {"mapper.lidar_projective_integrator_max_integration_distance_m",
                 "lidar_projective_integrator_max_integration_distance_m"},
                max_integration_distance_)));
        projective_params.projective_integrator_truncation_distance_vox.set(
            static_cast<float>(getDoubleParam(
                {"mapper.projective_integrator_truncation_distance_vox",
                 "projective_integrator_truncation_distance_vox"},
                truncation_distance_vox_)));
        const auto weighting_mode = parseWeightingMode(
            getStringParam({"mapper.projective_integrator_weighting_mode",
                            "projective_integrator_weighting_mode"},
                           nvblox::to_string(projective_params
                                                 .projective_integrator_weighting_mode.get())),
            projective_params.projective_integrator_weighting_mode.get());
        projective_params.projective_integrator_weighting_mode.set(weighting_mode);
        projective_params.projective_integrator_max_weight.set(static_cast<float>(
            getDoubleParam({"mapper.projective_integrator_max_weight",
                            "projective_integrator_max_weight"},
                           projective_params.projective_integrator_max_weight.get())));
        projective_params.projective_tsdf_integrator_invalid_depth_decay_factor.set(
            static_cast<float>(getDoubleParam(
                {"mapper.projective_tsdf_integrator_invalid_depth_decay_factor",
                 "projective_tsdf_integrator_invalid_depth_decay_factor"},
                projective_params
                    .projective_tsdf_integrator_invalid_depth_decay_factor.get())));
        projective_params.projective_appearance_integrator_measurement_weight.set(
            static_cast<float>(getDoubleParam(
                {"mapper.projective_appearance_integrator_measurement_weight",
                 "projective_appearance_integrator_measurement_weight"},
                projective_params.projective_appearance_integrator_measurement_weight
                    .get())));

        auto& view_params = mapper_params->view_calculator_params;
        view_params.raycast_subsampling_factor.set(getIntParam(
            {"mapper.raycast_subsampling_factor", "raycast_subsampling_factor"},
            view_params.raycast_subsampling_factor.get()));
        const auto workspace_type = parseWorkspaceBoundsType(
            getStringParam({"mapper.workspace_bounds_type", "workspace_bounds_type"},
                           nvblox::to_string(view_params.workspace_bounds_type.get())),
            view_params.workspace_bounds_type.get());
        view_params.workspace_bounds_type.set(workspace_type);
        view_params.workspace_bounds_min_height_m.set(static_cast<float>(
            getDoubleParam({"mapper.workspace_bounds_min_height_m",
                            "workspace_bounds_min_height_m"},
                           view_params.workspace_bounds_min_height_m.get())));
        view_params.workspace_bounds_max_height_m.set(static_cast<float>(
            getDoubleParam({"mapper.workspace_bounds_max_height_m",
                            "workspace_bounds_max_height_m"},
                           view_params.workspace_bounds_max_height_m.get())));
        view_params.workspace_bounds_min_corner_x_m.set(static_cast<float>(
            getDoubleParam({"mapper.workspace_bounds_min_corner_x_m",
                            "workspace_bounds_min_corner_x_m"},
                           view_params.workspace_bounds_min_corner_x_m.get())));
        view_params.workspace_bounds_max_corner_x_m.set(static_cast<float>(
            getDoubleParam({"mapper.workspace_bounds_max_corner_x_m",
                            "workspace_bounds_max_corner_x_m"},
                           view_params.workspace_bounds_max_corner_x_m.get())));
        view_params.workspace_bounds_min_corner_y_m.set(static_cast<float>(
            getDoubleParam({"mapper.workspace_bounds_min_corner_y_m",
                            "workspace_bounds_min_corner_y_m"},
                           view_params.workspace_bounds_min_corner_y_m.get())));
        view_params.workspace_bounds_max_corner_y_m.set(static_cast<float>(
            getDoubleParam({"mapper.workspace_bounds_max_corner_y_m",
                            "workspace_bounds_max_corner_y_m"},
                           view_params.workspace_bounds_max_corner_y_m.get())));

        auto& occupancy_params = mapper_params->occupancy_integrator_params;
        occupancy_params.free_region_occupancy_probability.set(static_cast<float>(
            getDoubleParam({"mapper.free_region_occupancy_probability",
                            "free_region_occupancy_probability"},
                           occupancy_params.free_region_occupancy_probability.get())));
        occupancy_params.occupied_region_occupancy_probability.set(static_cast<float>(
            getDoubleParam({"mapper.occupied_region_occupancy_probability",
                            "occupied_region_occupancy_probability"},
                           occupancy_params.occupied_region_occupancy_probability.get())));
        occupancy_params.unobserved_region_occupancy_probability.set(static_cast<float>(
            getDoubleParam({"mapper.unobserved_region_occupancy_probability",
                            "unobserved_region_occupancy_probability"},
                           occupancy_params.unobserved_region_occupancy_probability.get())));
        occupancy_params.occupied_region_half_width_m.set(static_cast<float>(
            getDoubleParam({"mapper.occupied_region_half_width_m",
                            "occupied_region_half_width_m"},
                           occupancy_params.occupied_region_half_width_m.get())));

        auto& mesh_params = mapper_params->mesh_integrator_params;
        mesh_params.mesh_integrator_min_weight.set(static_cast<float>(
            getDoubleParam({"mapper.mesh_integrator_min_weight",
                            "mesh_integrator_min_weight"},
                           mesh_params.mesh_integrator_min_weight.get())));
        mesh_params.mesh_integrator_weld_vertices.set(
            getBoolParam({"mapper.mesh_integrator_weld_vertices",
                          "mesh_integrator_weld_vertices"},
                         mesh_params.mesh_integrator_weld_vertices.get()));

        auto& tsdf_decay_params = mapper_params->tsdf_decay_integrator_params;
        tsdf_decay_params.tsdf_decay_factor.set(static_cast<float>(
            getDoubleParam({"mapper.tsdf_decay_factor", "tsdf_decay_factor"},
                           tsdf_decay_params.tsdf_decay_factor.get())));
        tsdf_decay_params.tsdf_decayed_weight_threshold.set(static_cast<float>(
            getDoubleParam({"mapper.tsdf_decayed_weight_threshold",
                            "tsdf_decayed_weight_threshold"},
                           tsdf_decay_params.tsdf_decayed_weight_threshold.get())));
        tsdf_decay_params.tsdf_set_free_distance_on_decayed.set(
            getBoolParam({"mapper.tsdf_set_free_distance_on_decayed",
                          "tsdf_set_free_distance_on_decayed"},
                         tsdf_decay_params.tsdf_set_free_distance_on_decayed.get()));
        tsdf_decay_params.tsdf_decayed_free_distance_vox.set(static_cast<float>(
            getDoubleParam({"mapper.tsdf_decayed_free_distance_vox",
                            "tsdf_decayed_free_distance_vox"},
                           tsdf_decay_params.tsdf_decayed_free_distance_vox.get())));

        auto& decay_base_params = mapper_params->decay_integrator_base_params;
        decay_base_params.decay_integrator_deallocate_decayed_blocks.set(
            getBoolParam({"mapper.decay_integrator_deallocate_decayed_blocks",
                          "decay_integrator_deallocate_decayed_blocks"},
                         decay_base_params.decay_integrator_deallocate_decayed_blocks
                             .get()));

        auto& occupancy_decay_params = mapper_params->occupancy_decay_integrator_params;
        occupancy_decay_params.free_region_decay_probability.set(static_cast<float>(
            getDoubleParam({"mapper.free_region_decay_probability",
                            "free_region_decay_probability"},
                           occupancy_decay_params.free_region_decay_probability.get())));
        occupancy_decay_params.occupied_region_decay_probability.set(static_cast<float>(
            getDoubleParam({"mapper.occupied_region_decay_probability",
                            "occupied_region_decay_probability"},
                           occupancy_decay_params.occupied_region_decay_probability.get())));
        occupancy_decay_params.occupancy_decay_to_free.set(
            getBoolParam({"mapper.occupancy_decay_to_free", "occupancy_decay_to_free"},
                         occupancy_decay_params.occupancy_decay_to_free.get()));

        auto& freespace_params = mapper_params->freespace_integrator_params;
        freespace_params.max_tsdf_distance_for_occupancy_m.set(static_cast<float>(
            getDoubleParam({"mapper.max_tsdf_distance_for_occupancy_m",
                            "max_tsdf_distance_for_occupancy_m"},
                           freespace_params.max_tsdf_distance_for_occupancy_m.get())));
        freespace_params.max_unobserved_to_keep_consecutive_occupancy_ms.set(
            getTimeParamMs({"mapper.max_unobserved_to_keep_consecutive_occupancy_ms",
                            "max_unobserved_to_keep_consecutive_occupancy_ms"},
                           static_cast<double>(static_cast<int64_t>(
                               freespace_params
                                   .max_unobserved_to_keep_consecutive_occupancy_ms.get()))));
        freespace_params.min_duration_since_occupied_for_freespace_ms.set(
            getTimeParamMs({"mapper.min_duration_since_occupied_for_freespace_ms",
                            "min_duration_since_occupied_for_freespace_ms"},
                           static_cast<double>(static_cast<int64_t>(
                               freespace_params.min_duration_since_occupied_for_freespace_ms
                                   .get()))));
        freespace_params.min_consecutive_occupancy_duration_for_reset_ms.set(
            getTimeParamMs(
                {"mapper.min_consecutive_occupancy_duration_for_reset_ms",
                 "min_consecutive_occupancy_duration_for_reset_ms"},
                static_cast<double>(static_cast<int64_t>(
                    freespace_params.min_consecutive_occupancy_duration_for_reset_ms
                        .get()))));
        freespace_params.check_neighborhood.set(
            getBoolParam({"mapper.check_neighborhood", "check_neighborhood"},
                         freespace_params.check_neighborhood.get()));
        freespace_params.initialize_to_high_confidence_freespace.set(
            getBoolParam({"mapper.initialize_to_high_confidence_freespace",
                          "initialize_to_high_confidence_freespace"},
                         freespace_params.initialize_to_high_confidence_freespace
                             .get()));

        esdf_slice_min_height_ = esdf_params.esdf_slice_min_height.get();
        esdf_slice_max_height_ = esdf_params.esdf_slice_max_height.get();
        esdf_slice_height_ = esdf_params.esdf_slice_height.get();
        esdf_slice_height_above_plane_m_ =
            esdf_params.slice_height_above_plane_m.get();
        esdf_slice_thickness_m_ = esdf_params.slice_height_thickness_m.get();
    }

    // Parameters
    double voxel_size_;
    double truncation_distance_vox_;
    double max_integration_distance_;
    double lidar_vertical_fov_deg_;
    int lidar_num_azimuth_;
    int lidar_num_elevation_;
    double mesh_update_rate_hz_;
    double esdf_update_rate_hz_;
    std::string pointcloud_topic_;
    std::string odom_topic_;
    std::string frame_id_;
    bool publish_mesh_;
    bool publish_esdf_slice_;
    bool publish_occupancy_grid_;
    bool publish_voxels_;
    bool publish_voxel_points_;
    double esdf_slice_height_;
    std::string voxel_export_path_;
    double voxel_export_surface_band_;
    double voxel_export_min_weight_;
    bool voxel_export_surface_only_;
    int voxel_export_points_per_side_;
    double voxel_marker_alpha_;
    std::string voxel_color_mode_;
    std::vector<double> voxel_flat_color_;
    std::string esdf_mode_ = "2d";
    bool esdf_mode_is_3d_ = false;
    double esdf_slice_min_height_ = 0.0;
    double esdf_slice_max_height_ = 1.0;
    double esdf_slice_thickness_m_ = 0.1;
    double esdf_slice_height_above_plane_m_ = 0.0;

    // nvblox components
    std::unique_ptr<nvblox::Mapper> mapper_;
    std::unique_ptr<nvblox::Lidar> lidar_;
    nvblox::MeshSerializerGpu<nvblox::Color> mesh_serializer_;

    // ROS components
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr voxel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_slice_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tsdf_slice_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_voxels_srv_;
    rclcpp::TimerBase::SharedPtr mesh_timer_;
    rclcpp::TimerBase::SharedPtr esdf_timer_;
    rclcpp::TimerBase::SharedPtr mesh_start_timer_;
    rclcpp::TimerBase::SharedPtr esdf_start_timer_;

    // TF components
    bool use_tf_;
    double tf_timeout_sec_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Post-processing
    int lidar_hole_filling_radius_;
    bool lidar_denoise_enable_;
    int lidar_denoise_k_;
    double lidar_denoise_stddev_;
    bool lidar_smooth_enable_;
    double lidar_smooth_radius_;

    // State
    nav_msgs::msg::Odometry latest_odom_;
    bool has_odom_ = false;
    std::mutex odom_mutex_;
    std::mutex mapper_mutex_;
    int integration_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<NvbloxFastlioNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
