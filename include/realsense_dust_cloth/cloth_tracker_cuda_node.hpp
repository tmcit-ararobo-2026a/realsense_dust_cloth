#pragma once

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
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
    void declare_and_load_parameters();
    void setup_map_background();
    bool prepare_map_frame(const rclcpp::Time& stamp);
    void publish_background_state(const std::string&, bool);
    std::recursive_mutex processing_mutex_;
    std::string map_frame_;
    MapBackground map_background_;
    MapFilterView map_filter_;
    std::shared_ptr<tf2_ros::Buffer> map_tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> map_tf_listener_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_background_status_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ready_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_tracking_state_;
    double map_tf_wait_sec_{0.15}, map_max_image_age_sec_{0.25};
    double map_tf_poll_sec_{0.002};
    double map_pose_loss_reset_sec_{0.25}, map_last_pose_sec_{-1.0};
    int map_depth_queue_size_{8};
    bool map_processing_failed_{false};
    void publish_static_transforms();

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void depth_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

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

    // Trajectory history
    std::vector<std::pair<double, geometry_msgs::msg::Point>> trajectory_points_;
    double last_frame_stamp_sec_{0.0};
    double last_static_publish_sec_{0.0};

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
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_landing_pos_;
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

    rclcpp::CallbackGroup::SharedPtr depth_cb_group_;
    rclcpp::CallbackGroup::SharedPtr color_cb_group_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_image_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compressed_color_;
    bool raw_color_received_{false};

    void color_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void compressed_color_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);

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

    // Robot Motion Monitoring & Stop-and-Shoot Fast Background Relearn
    bool enable_cmd_vel_watch_{false};
    std::string cmd_vel_topic_{"/robot/operation/cmd_vel"};
    std::string landing_position_topic_{"/cloth/landing_position"};
    double cmd_vel_deadband_linear_{0.02};
    double cmd_vel_deadband_angular_{0.02};
    double cmd_vel_timeout_sec_{0.25};
    double stop_settle_sec_{0.10};

    bool is_robot_moving_{false};
    rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time stop_detected_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_fallback_cmd_vel_;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void check_robot_motion_state(const rclcpp::Time& current_time);

    // Direct RealSense Thread
    std::atomic<bool> is_running_{false};
    std::thread realsense_thread_;

#ifdef HAVE_REALSENSE2
    std::unique_ptr<rs2::pipeline> rs_pipe_;
#endif

    // Asynchronous Color Worker Thread
    struct ColorTask {
        cv::Mat bgr_frame;
        std::vector<uint8_t> compressed_data;
        std::string frame_id;
        rclcpp::Time stamp;
        sensor_msgs::msg::CameraInfo color_info;
        bool is_compressed{false};
        bool has_color_info{false};
        bool has_new_frame{false};
    };
    std::thread color_worker_thread_;
    std::condition_variable color_cv_;
    std::mutex color_mutex_;
    std::atomic<bool> color_worker_running_{false};
    ColorTask latest_color_task_;
    void color_worker_loop();
    void push_color_frame(const cv::Mat& bgr_mat, const rclcpp::Time& stamp, const sensor_msgs::msg::CameraInfo* info = nullptr);
};

} // namespace realsense_dust_cloth
