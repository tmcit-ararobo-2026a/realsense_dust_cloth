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

#ifdef HAVE_REALSENSE2
#include <librealsense2/rs.hpp>
#endif

namespace realsense_dust_cloth {

struct ParabolaFit {
    bool valid{false};
    double a{0.0}, b{0.0}, c{0.0}; // Z(X) = a*X^2 + b*X + c
    double d{0.0}, e{0.0};          // Y(X) = d*X + e
    double x_start{0.0};
    double x_apex{0.0}, z_apex{0.0};
    double x_land{0.0}, y_land{0.0}, z_land{0.0};
    double r2{0.0};
    std::vector<geometry_msgs::msg::Point> curve_pts;
};

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

    // Ballistic State Tracker (Eliminates stationary noise, locks onto flying cloth)
    enum class TrackerState { IDLE, TRACKING, LANDED };
    struct BallisticTracker {
        TrackerState state{TrackerState::IDLE};
        double x{0.0}, y{0.0}, z{0.0};
        double vx{0.0}, vy{0.0}, vz{0.0};
        double last_stamp{0.0};
        int tracked_count{0};
        int lost_count{0};

        // Launch velocity detection (prevents stationary noise from triggering)
        bool has_pending{false};
        double pending_x{0.0}, pending_y{0.0}, pending_z{0.0};
        double pending_stamp{0.0};
        double min_launch_vx{0.30}; // minimum forward velocity (m/s) to confirm launch

        // Parameters
        double launch_min_x{0.40};
        double launch_max_x{0.85};
        double launch_min_y{-0.30};
        double launch_max_y{0.30};
        double launch_min_z{0.65};
        double launch_max_z{1.30};
        double gate_radius_m{0.80};
        int max_lost_frames{6};
        double gravity_m_s2{9.8};

        void reset() {
            state = TrackerState::IDLE;
            tracked_count = 0;
            lost_count = 0;
            has_pending = false;
            last_stamp = 0.0;
            pending_stamp = 0.0;
        }

        bool get_prediction(double stamp_sec, float& px, float& py, float& pz) const {
            if (state != TrackerState::TRACKING || last_stamp <= 0.0) {
                return false;
            }
            double dt = stamp_sec - last_stamp;
            if (dt <= 0.0 || dt > 0.3) {
                dt = 0.016; // 60fps fallback
            }
            px = static_cast<float>(x + vx * dt);
            py = static_cast<float>(y + vy * dt);
            pz = static_cast<float>(z + vz * dt - 0.5 * 3.0 * dt * dt);
            return true;
        }

        bool update(const ClothTrackingResult& raw, double stamp_sec, ClothTrackingResult& out);
    } ballistic_tracker_;

    // ROS 2 Interfaces
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_roi_markers_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_color_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_color_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_color_info_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_image_;

    void color_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    void publish_static_tf();
    void publish_roi_markers(const rclcpp::Time& stamp, const ClothTrackingResult& raw_res);

    bool enable_color_stream_{true};
    int color_fps_{30};
    int jpeg_quality_{65};
    int color_rotation_deg_{180};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_rotated_color_;

    // Direct RealSense Thread
    std::atomic<bool> is_running_{false};
    std::thread realsense_thread_;

#ifdef HAVE_REALSENSE2
    std::unique_ptr<rs2::pipeline> rs_pipe_;
#endif
};

} // namespace realsense_dust_cloth

