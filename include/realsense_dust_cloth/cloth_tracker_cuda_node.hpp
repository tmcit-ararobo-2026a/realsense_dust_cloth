#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/core.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "realsense_dust_cloth/ballistic_tracker.hpp"
#include "realsense_dust_cloth/cuda_kernels.cuh"

#ifdef HAVE_REALSENSE2
#include <librealsense2/rs.hpp>
#endif

namespace realsense_dust_cloth {

// 深度の背景差分、または外部差分点群 → 候補選択 → 予測追跡 → 実測結果の配信。
class ClothTrackerCudaNode : public rclcpp::Node {
  public:
    explicit ClothTrackerCudaNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ClothTrackerCudaNode() override;

  private:
    friend struct ClothTrackerNodeTestAccess;
    // 起動時設定・実行時動的変更: tracker_parameters.cpp
    void declare_and_load_parameters();
    rcl_interfaces::msg::SetParametersResult
    on_parameters_change(const std::vector<rclcpp::Parameter>& params);

    // カメラ入力: camera_input.cpp / realsense_camera.cpp
    void setup_camera_input();
    void setup_direct_camera();
    void setup_camera_subscriptions();
    void publish_static_transforms();
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void depth_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void process_and_publish(const uint16_t* depth, const rclcpp::Time& stamp);
    void run_direct_realsense_loop();

    // gn10への送信・差分点群の受信: external_cloud_tracking.cpp
    void setup_external_cloud();
    void publish_filter_input(const std::vector<float3>& points, const rclcpp::Time& stamp);
    void cropped_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void dynamic_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void filter_status_callback(const std_msgs::msg::String::SharedPtr msg);
    void check_external_timeout();
    ClothTrackingResult find_cloth_candidate(const std::vector<float3>& points, double stamp_sec,
                                             std::vector<float3>* cluster,
                                             std::vector<float3>* filtered);

    FilterConfig candidate_search_filter(double stamp_sec, float& x, float& y, float& z,
                                         bool& predicted) const;
    bool debug_frame_due();
    bool want_cluster_debug() const;
    bool want_filtered_debug() const;

    // 射出検知 & 背景凍結ステートマシン
    enum class BackgroundState { ROLLING, FROZEN };
    BackgroundState bg_state_{BackgroundState::ROLLING};
    double shot_freeze_until_sec_{0.0};
    uint8_t last_shot_count_{0};
    bool has_received_shot_count_{false};
    std::string shot_count_topic_{"/robot/feedback/shot_count"};
    std::string reset_topic_{"/cloth/reset"};
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_shot_count_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_reset_;
    void shot_count_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void reset_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void trigger_freeze(double now_sec);

    // 実測結果と状態の配信: tracking_output.cpp / tracking_status.cpp
    void setup_tracking_status();
    void publish_input_status(const std::string& status, bool ready);
    void publish_tracking_result(const ClothTrackingResult& candidate, const rclcpp::Time& stamp,
                                 const std::vector<float3>& cluster,
                                 const std::vector<float3>& filtered);
    void publish_tracking_transition(const ClothTrackingResult& candidate,
                                     const ClothTrackingResult& observation,
                                     const rclcpp::Time& stamp, bool accepted);
    void remember_observation(const ClothTrackingResult& observation, const rclcpp::Time& stamp);
    void publish_tracking_visualization(const rclcpp::Time& stamp, bool accepted);
    void publish_roi_markers(const rclcpp::Time& stamp, const ClothTrackingResult& candidate);

    // カラー画像は最新1枚だけを別スレッドで処理: color_stream.cpp
    void color_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void compressed_color_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
    void push_color_frame(const cv::Mat& bgr, const rclcpp::Time& stamp,
                          const sensor_msgs::msg::CameraInfo* info = nullptr);
    void color_worker_loop();

    // パイプラインの状態とGPU領域
    std::recursive_mutex processing_mutex_;
    CudaTrackerContext cuda_ctx_;
    CameraIntrinsics camera_intrin_;
    CameraPoseConfig camera_pose_;
    FilterConfig filter_cfg_;
    BallisticTracker ballistic_tracker_;
    TrackerState prev_state_{TrackerState::IDLE};
    TrackerState last_visualization_state_{TrackerState::IDLE};
    bool intrinsics_ready_{false};

    // カメラと座標系の設定。値は起動時にYAMLから取得する。
    std::string input_source_;
    std::string camera_parent_frame_{"base_link"};
    std::string camera_frame_{"camera_link"};
    std::string depth_optical_frame_, color_optical_frame_, cloth_tf_frame_;
    bool use_direct_realsense_{false};
    bool publish_static_tf_{true};
    bool enable_color_stream_{true};
    std::atomic<bool> publish_raw_depth_{false};
    int depth_fps_{60}, color_fps_{30};
    int jpeg_quality_{65}, color_rotation_deg_{180};
    double rs_laser_power_{-1.0};
    int rs_exposure_us_{-1};
    int realsense_wait_timeout_ms_{};
    int depth_queue_size_{}, publisher_queue_size_{}, color_queue_size_{};
    int warning_log_throttle_ms_{}, status_log_throttle_ms_{};
    double image_max_age_sec_{};

    // 外部フィルターとの接続。wall時刻は入力途絶監視、ROS時刻は観測の整合に使う。
    std::string cropped_replay_topic_, cropped_cloud_topic_, dynamic_cloud_topic_;
    double external_keep_radius_m_{0.3};
    double external_result_timeout_sec_{0.25}, external_poll_sec_{0.02};
    double last_external_wall_sec_{-1.0}, last_external_stamp_sec_{-1.0};
    double bag_rewind_tolerance_sec_{};

    // 最後に採用した実測位置と表示用の履歴
    std::vector<std::pair<double, geometry_msgs::msg::Point>> trajectory_points_;
    double trajectory_history_sec_{3.0};
    double min_trajectory_step_m_{}, static_marker_interval_sec_{};
    int max_trajectory_points_{};
    double last_frame_stamp_sec_{0.0}, last_static_publish_sec_{0.0};
    double last_cloth_x_{0.0}, last_cloth_y_{0.0}, last_cloth_z_{0.0};
    bool has_cloth_pose_{false};
    bool publish_debug_cloud_{false};
    double debug_publish_hz_{};
    std::chrono::steady_clock::time_point last_debug_publish_{};
    bool debug_frame_due_{false};
    double bg_update_interval_sec_{};
    double last_background_update_sec_{-1.0};
    std::string last_input_status_, last_tracking_status_;
    bool ready_published_{false}, last_ready_{false};
    std::string landing_position_topic_{"/cloth/landing_position"};

    // ROS購読・タイマー
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_image_, sub_color_image_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compressed_color_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cropped_replay_,
        sub_dynamic_cloud_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_filter_status_;
    rclcpp::CallbackGroup::SharedPtr depth_cb_group_, color_cb_group_;
    rclcpp::TimerBase::SharedPtr external_watchdog_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // ROS配信
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point_, pub_landing_pos_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_,
        pub_filter_prediction_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_roi_markers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cropped_cloud_,
        pub_cluster_cloud_, pub_filtered_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_depth_, pub_raw_color_,
        pub_rotated_color_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_color_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_depth_info_, pub_color_info_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_input_status_, pub_tracking_state_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ready_;

    // USBカメラ取得スレッド
    std::atomic<bool> is_running_{false};
    std::thread realsense_thread_;
#ifdef HAVE_REALSENSE2
    std::unique_ptr<rs2::pipeline> rs_pipe_;
#endif

    // 画像配信スレッド。未処理フレームを積み上げず、常に最新フレームへ置き換える。
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
    bool raw_color_received_{false};
    std::thread color_worker_thread_;
    std::condition_variable color_cv_;
    std::mutex color_mutex_;
    std::atomic<bool> color_worker_running_{false};
    ColorTask latest_color_task_;
};

} // namespace realsense_dust_cloth
