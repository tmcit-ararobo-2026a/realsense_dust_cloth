#pragma once

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "realsense_dust_cloth/cuda_kernels.cuh"
#include "realsense_dust_cloth/ballistic_tracker.hpp"
#include "realsense_dust_cloth/parabola_fit.hpp"
#include "realsense_dust_cloth/marker_visualizer.hpp"

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
    void publish_static_transforms();

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
    bool enable_ballistic_filter_{true};
    double trajectory_history_sec_{3.0};
    int bg_frames_collected_{0};
    int bg_target_frames_{20};

    // Trajectory history & persistent parabola visualization
    std::vector<std::pair<double, geometry_msgs::msg::Point>> trajectory_points_;
    ParabolaFit cached_parabola_;
    double last_frame_stamp_sec_{0.0};
    geometry_msgs::msg::Point last_published_pt_;
    bool has_last_published_pt_{false};

    // Last known cloth position (for continuous TF broadcasting base_link -> flying_cloth)
    double last_cloth_x_{0.0};
    double last_cloth_y_{0.0};
    double last_cloth_z_{0.0};
    bool has_cloth_pose_{false};

    // Ballistic State Tracker (Eliminates stationary noise, locks onto flying cloth)
    BallisticTracker ballistic_tracker_;

    // ROS 2 Interfaces
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::string camera_parent_frame_{"base_link"};
    std::string camera_frame_{"camera_link"};
    bool publish_static_tf_{true};
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_roi_markers_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_depth_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_depth_info_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_color_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_color_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_color_info_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_image_;

    void color_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    void publish_roi_markers(const rclcpp::Time& stamp, const ClothTrackingResult& raw_res);

    bool enable_color_stream_{true};
    int color_fps_{30};
    int jpeg_quality_{65};
    int color_rotation_deg_{180};
    int depth_fps_{60};
    double rs_laser_power_{-1.0};
    int rs_exposure_us_{-1};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rotated_color_;
    TrackerState prev_state_{TrackerState::IDLE};

    // Direct RealSense Thread
    std::atomic<bool> is_running_{false};
    std::thread realsense_thread_;

#ifdef HAVE_REALSENSE2
    std::unique_ptr<rs2::pipeline> rs_pipe_;
#endif
};

} // namespace realsense_dust_cloth
