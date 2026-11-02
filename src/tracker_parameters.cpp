// YAMLの必須設定の読込と起動時検証。
#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"
#include <type_traits>

namespace realsense_dust_cloth {

namespace {
template <class T> T required_parameter(rclcpp::Node& node, const std::string& name) {
    rclcpp::ParameterType type;
    if constexpr (std::is_same_v<T, bool>)
        type = rclcpp::ParameterType::PARAMETER_BOOL;
    else if constexpr (std::is_same_v<T, int>)
        type = rclcpp::ParameterType::PARAMETER_INTEGER;
    else if constexpr (std::is_same_v<T, double>)
        type = rclcpp::ParameterType::PARAMETER_DOUBLE;
    else
        type = rclcpp::ParameterType::PARAMETER_STRING;
    node.declare_parameter(name, type);
    const auto value = node.get_parameter(name).get_value<T>();
    if constexpr (std::is_same_v<T, double>) {
        if (!std::isfinite(value))
            throw std::invalid_argument("Non-finite parameter: " + name);
    }
    return value;
}
} // namespace

void ClothTrackerCudaNode::declare_and_load_parameters() {
    bag_rewind_tolerance_sec_ = required_parameter<double>(*this, "bag_rewind_tolerance_sec");
    min_trajectory_step_m_ = required_parameter<double>(*this, "min_trajectory_step_m");
    max_trajectory_points_ = required_parameter<int>(*this, "max_trajectory_points");
    static_marker_interval_sec_ = required_parameter<double>(*this, "static_marker_interval_sec");
    realsense_wait_timeout_ms_ = required_parameter<int>(*this, "realsense_wait_timeout_ms");
    publisher_queue_size_ = required_parameter<int>(*this, "publisher_queue_size");
    color_queue_size_ = required_parameter<int>(*this, "color_queue_size");
    warning_log_throttle_ms_ = required_parameter<int>(*this, "warning_log_throttle_ms");
    status_log_throttle_ms_ = required_parameter<int>(*this, "status_log_throttle_ms");
    color_optical_frame_ = required_parameter<std::string>(*this, "color_optical_frame");
    depth_optical_frame_ = required_parameter<std::string>(*this, "depth_optical_frame");
    cloth_tf_frame_ = required_parameter<std::string>(*this, "cloth_tf_frame");

    // 1. 入力トピックとカメラの接続方式
    required_parameter<std::string>(*this, "depth_topic");
    required_parameter<std::string>(*this, "camera_info_topic");
    required_parameter<std::string>(*this, "color_topic");
    use_direct_realsense_ = required_parameter<bool>(*this, "use_direct_realsense");

    // 2. カメラの座標系・取付位置・撮影設定
    camera_parent_frame_ = required_parameter<std::string>(*this, "camera_parent_frame");
    camera_frame_ = required_parameter<std::string>(*this, "camera_frame");
    publish_static_tf_ = required_parameter<bool>(*this, "publish_static_tf");
    camera_intrin_.width = required_parameter<int>(*this, "camera_width");
    camera_intrin_.height = required_parameter<int>(*this, "camera_height");
    camera_pose_.pitch_deg =
        static_cast<float>(required_parameter<double>(*this, "camera_pitch_deg"));
    camera_pose_.roll_deg =
        static_cast<float>(required_parameter<double>(*this, "camera_roll_deg"));
    camera_pose_.x_offset_m =
        static_cast<float>(required_parameter<double>(*this, "camera_x_offset_m"));
    camera_pose_.height_offset_m =
        static_cast<float>(required_parameter<double>(*this, "camera_height_offset_m"));
    color_rotation_deg_ = required_parameter<int>(*this, "color_rotation_deg");
    depth_fps_ = required_parameter<int>(*this, "depth_fps");
    rs_laser_power_ = required_parameter<double>(*this, "rs_laser_power");
    rs_exposure_us_ = required_parameter<int>(*this, "rs_exposure_us");

    // 3. 点群の入力段階・外部フィルター接続・ROI
    input_source_ = required_parameter<std::string>(*this, "input_source");
    cropped_replay_topic_ = required_parameter<std::string>(*this, "cropped_replay_topic");
    if (input_source_ != "depth" && input_source_ != "cropped_cloud" &&
        input_source_ != "dynamic_cloud")
        throw std::invalid_argument("input_source must be depth, cropped_cloud or dynamic_cloud");
    if (input_source_ != "depth" && use_direct_realsense_)
        throw std::invalid_argument("Point-cloud replay requires use_direct_realsense=false");
    cropped_cloud_topic_ = required_parameter<std::string>(*this, "cropped_cloud_topic");
    dynamic_cloud_topic_ = required_parameter<std::string>(*this, "dynamic_cloud_topic");
    external_result_timeout_sec_ = required_parameter<double>(*this, "external_result_timeout_sec");
    external_poll_sec_ = required_parameter<double>(*this, "external_poll_sec");
    external_keep_radius_m_ = required_parameter<double>(*this, "external_tracking_keep_radius_m");
    required_parameter<std::string>(*this, "external_prediction_topic");
    if (!std::isfinite(external_keep_radius_m_) || external_keep_radius_m_ < 0)
        throw std::invalid_argument("Invalid tracking protection radius");
    required_parameter<std::string>(*this, "external_filter_status_topic");
    if (!std::isfinite(external_result_timeout_sec_) || external_result_timeout_sec_ <= 0 ||
        !std::isfinite(external_poll_sec_) || external_poll_sec_ <= 0)
        throw std::invalid_argument("Invalid external cloud timing");

    filter_cfg_.min_depth_m = static_cast<float>(required_parameter<double>(*this, "min_depth_m"));
    filter_cfg_.max_depth_m = static_cast<float>(required_parameter<double>(*this, "max_depth_m"));
    filter_cfg_.min_x_m = static_cast<float>(required_parameter<double>(*this, "min_x_m"));
    filter_cfg_.max_x_m = static_cast<float>(required_parameter<double>(*this, "max_x_m"));
    filter_cfg_.min_y_m = static_cast<float>(required_parameter<double>(*this, "min_y_m"));
    filter_cfg_.max_y_m = static_cast<float>(required_parameter<double>(*this, "max_y_m"));
    filter_cfg_.min_z_m = static_cast<float>(required_parameter<double>(*this, "min_z_m"));
    filter_cfg_.max_z_m = static_cast<float>(required_parameter<double>(*this, "max_z_m"));

    // 4. 間引きとクラスタ判定
    filter_cfg_.voxel_size_m =
        static_cast<float>(required_parameter<double>(*this, "voxel_size_m"));
    filter_cfg_.cluster_eps_m =
        static_cast<float>(required_parameter<double>(*this, "cluster_eps_m"));
    filter_cfg_.min_cluster_points = required_parameter<int>(*this, "min_cluster_points");
    filter_cfg_.max_cluster_points = required_parameter<int>(*this, "max_cluster_points");
    filter_cfg_.min_cloth_size_m =
        static_cast<float>(required_parameter<double>(*this, "min_cloth_size_m"));
    filter_cfg_.max_cloth_size_m =
        static_cast<float>(required_parameter<double>(*this, "max_cloth_size_m"));
    filter_cfg_.continuity_thresh_m =
        static_cast<float>(required_parameter<double>(*this, "continuity_thresh_m"));
    filter_cfg_.continuity_min_neighbors =
        required_parameter<int>(*this, "continuity_min_neighbors");

    // 5. デバッグ表示と色画像の配信
    publish_debug_cloud_ = required_parameter<bool>(*this, "publish_debug_cloud");
    trajectory_history_sec_ = required_parameter<double>(*this, "trajectory_history_sec");
    enable_color_stream_ = required_parameter<bool>(*this, "enable_color_stream");
    color_fps_ = required_parameter<int>(*this, "color_fps");
    jpeg_quality_ = required_parameter<int>(*this, "jpeg_quality");

    // 6. 射出確認と飛翔予測
    ballistic_tracker_.launch_min_x = required_parameter<double>(*this, "launch_min_x");
    ballistic_tracker_.launch_max_x = required_parameter<double>(*this, "launch_max_x");
    ballistic_tracker_.launch_min_y = required_parameter<double>(*this, "launch_min_y");
    ballistic_tracker_.launch_max_y = required_parameter<double>(*this, "launch_max_y");
    ballistic_tracker_.launch_min_z = required_parameter<double>(*this, "launch_min_z");
    ballistic_tracker_.launch_max_z = required_parameter<double>(*this, "launch_max_z");
    ballistic_tracker_.max_lost_frames = required_parameter<int>(*this, "max_lost_frames");
    ballistic_tracker_.min_launch_vx = required_parameter<double>(*this, "min_launch_vx");
    ballistic_tracker_.min_launch_move_m = required_parameter<double>(*this, "min_launch_move_m");
    ballistic_tracker_.launch_timeout_sec = required_parameter<double>(*this, "pending_reset_sec");
    ballistic_tracker_.min_track_points_limit =
        required_parameter<int>(*this, "min_tracking_points");
    ballistic_tracker_.min_cluster_points = filter_cfg_.min_cluster_points;
    ballistic_tracker_.landing_min_z = required_parameter<double>(*this, "landing_min_z");
    ballistic_tracker_.drag_coeff = required_parameter<double>(*this, "drag_coeff");
    ballistic_tracker_.vel_smoothing_beta = required_parameter<double>(*this, "vel_smoothing_beta");
    ballistic_tracker_.min_flight_vx_m_s = required_parameter<double>(*this, "min_flight_vx_m_s");
    ballistic_tracker_.max_vx_m_s = required_parameter<double>(*this, "max_vx_m_s");
    ballistic_tracker_.max_abs_vy_m_s = required_parameter<double>(*this, "max_abs_vy_m_s");
    ballistic_tracker_.min_vz_m_s = required_parameter<double>(*this, "min_vz_m_s");
    ballistic_tracker_.max_vz_m_s = required_parameter<double>(*this, "max_vz_m_s");
    ballistic_tracker_.max_flight_duration_sec =
        required_parameter<double>(*this, "max_flight_duration_sec");
    ballistic_tracker_.min_stationary_move_m =
        required_parameter<double>(*this, "min_stationary_move_m");
    ballistic_tracker_.max_stationary_frames =
        required_parameter<int>(*this, "max_stationary_frames");

    // 7. 候補の物理条件と予測からの許容範囲
    ballistic_tracker_.tube_radius_m = required_parameter<double>(*this, "tube_radius_m");
    ballistic_tracker_.max_total_accel_m_s2 =
        required_parameter<double>(*this, "max_total_accel_m_s2");
    ballistic_tracker_.centroid_position_tolerance_m =
        required_parameter<double>(*this, "centroid_position_tolerance_m");
    ballistic_tracker_.min_vertical_accel_m_s2 =
        required_parameter<double>(*this, "min_vertical_accel_m_s2");
    ballistic_tracker_.max_lateral_accel_m_s2 =
        required_parameter<double>(*this, "max_lateral_accel_m_s2");
    ballistic_tracker_.max_heading_deg = required_parameter<double>(*this, "max_heading_deg");
    ballistic_tracker_.backward_jitter_tolerance_m =
        required_parameter<double>(*this, "backward_jitter_tolerance_m");
    ballistic_tracker_.vertical_jitter_tolerance_m =
        required_parameter<double>(*this, "vertical_jitter_tolerance_m");
    ballistic_tracker_.reject_ascent_after_apex =
        required_parameter<bool>(*this, "reject_ascent_after_apex");
    ballistic_tracker_.abort_on_anomaly = required_parameter<bool>(*this, "abort_on_anomaly");
    ballistic_tracker_.max_anomaly_frames = required_parameter<int>(*this, "max_anomaly_frames");

    ballistic_tracker_.gravity_m_s2 = required_parameter<double>(*this, "gravity_m_s2");
    ballistic_tracker_.lost_frame_gate_expansion_m =
        required_parameter<double>(*this, "lost_frame_gate_expansion_m");
    ballistic_tracker_.max_gate_expansion_m =
        required_parameter<double>(*this, "max_gate_expansion_m");
    ballistic_tracker_.heading_min_speed_m_s =
        required_parameter<double>(*this, "heading_min_speed_m_s");
    ballistic_tracker_.heading_min_step_m = required_parameter<double>(*this, "heading_min_step_m");
    ballistic_tracker_.pending_max_drop_m = required_parameter<double>(*this, "pending_max_drop_m");

    filter_cfg_.continuity_radius_px = required_parameter<int>(*this, "continuity_radius_px");
    filter_cfg_.prediction_distance_weight =
        required_parameter<double>(*this, "prediction_distance_weight");
    filter_cfg_.cluster_size_weight = required_parameter<double>(*this, "cluster_size_weight");
    filter_cfg_.launch_forward_weight = required_parameter<double>(*this, "launch_forward_weight");
    cuda_ctx_.max_voxel_points = required_parameter<int>(*this, "max_voxel_points");
    cuda_ctx_.hash_table_size = required_parameter<int>(*this, "voxel_hash_table_size");

    // Synchronize CUDA filter launch gate & tracking points with tracker parameters
    filter_cfg_.min_tracking_points = ballistic_tracker_.min_track_points_limit;
    filter_cfg_.launch_min_x = static_cast<float>(ballistic_tracker_.launch_min_x);
    filter_cfg_.launch_max_x = static_cast<float>(ballistic_tracker_.launch_max_x);
    filter_cfg_.launch_min_y = static_cast<float>(ballistic_tracker_.launch_min_y);
    filter_cfg_.launch_max_y = static_cast<float>(ballistic_tracker_.launch_max_y);
    filter_cfg_.launch_min_z = static_cast<float>(ballistic_tracker_.launch_min_z);
    filter_cfg_.launch_max_z = static_cast<float>(ballistic_tracker_.launch_max_z);
    filter_cfg_.gate_radius_m = static_cast<float>(ballistic_tracker_.tube_radius_m);

    landing_position_topic_ = required_parameter<std::string>(*this, "landing_position_topic");

    image_max_age_sec_ = required_parameter<double>(*this, "image_max_age_sec");
    depth_queue_size_ = required_parameter<int>(*this, "depth_queue_size");
    if (camera_intrin_.width <= 0 || camera_intrin_.height <= 0 || image_max_age_sec_ <= 0 ||
        depth_queue_size_ <= 0 || publisher_queue_size_ <= 0 || color_queue_size_ <= 0 ||
        max_trajectory_points_ <= 0 || min_trajectory_step_m_ < 0 ||
        static_marker_interval_sec_ <= 0 || realsense_wait_timeout_ms_ <= 0 ||
        bag_rewind_tolerance_sec_ <= 0 || filter_cfg_.voxel_size_m <= 0 ||
        filter_cfg_.cluster_eps_m <= 0 || filter_cfg_.min_x_m >= filter_cfg_.max_x_m ||
        filter_cfg_.min_y_m >= filter_cfg_.max_y_m || filter_cfg_.min_z_m >= filter_cfg_.max_z_m ||
        filter_cfg_.min_depth_m >= filter_cfg_.max_depth_m ||
        filter_cfg_.continuity_radius_px < 1 || filter_cfg_.continuity_min_neighbors < 0 ||
        filter_cfg_.continuity_min_neighbors > 4 || cuda_ctx_.max_voxel_points <= 0 ||
        cuda_ctx_.hash_table_size < cuda_ctx_.max_voxel_points ||
        (cuda_ctx_.hash_table_size & (cuda_ctx_.hash_table_size - 1)) != 0 ||
        ballistic_tracker_.centroid_position_tolerance_m < 0 ||
        ballistic_tracker_.gravity_m_s2 <= 0 || ballistic_tracker_.tube_radius_m <= 0 ||
        ballistic_tracker_.min_track_points_limit <= 0 ||
        ballistic_tracker_.min_cluster_points <= 0 || ballistic_tracker_.max_anomaly_frames <= 0 ||
        ballistic_tracker_.max_lost_frames < 0 || ballistic_tracker_.vel_smoothing_beta < 0 ||
        ballistic_tracker_.vel_smoothing_beta > 1)
        throw std::invalid_argument("Invalid camera/ROI/GPU/tracking parameters in YAML");
}

} // namespace realsense_dust_cloth
