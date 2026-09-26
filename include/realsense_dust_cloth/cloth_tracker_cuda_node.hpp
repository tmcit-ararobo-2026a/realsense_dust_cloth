#pragma once

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "realsense_dust_cloth/cuda_kernels.cuh"

#ifdef HAVE_REALSENSE2
#include <librealsense2/rs.hpp>
#endif

namespace realsense_dust_cloth {

class ClothTrackerCudaNode : public rclcpp::Node {
public:
    explicit ClothTrackerCudaNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ClothTrackerCudaNode() override;

private:
    void declare_parameters();
    void load_parameters();

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    void process_and_publish(const uint16_t* depth_ptr, const rclcpp::Time& stamp);

    void run_direct_realsense_loop();

    // CUDA Context
    CudaTrackerContext cuda_ctx_;
    CameraIntrinsics camera_intrin_;
    CameraPoseConfig camera_pose_;
    FilterConfig filter_cfg_;

    bool intrinsics_ready_{false};
    bool publish_debug_cloud_{false};
    bool use_direct_realsense_{false};
    double trajectory_history_sec_{3.0};

    // Trajectory history
    std::vector<std::pair<double, geometry_msgs::msg::Point>> trajectory_points_;

    // ROS 2 Interfaces
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster_cloud_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_image_;

    // Direct RealSense Thread
    std::atomic<bool> is_running_{false};
    std::thread realsense_thread_;

#ifdef HAVE_REALSENSE2
    std::unique_ptr<rs2::pipeline> rs_pipe_;
#endif
};

} // namespace realsense_dust_cloth

