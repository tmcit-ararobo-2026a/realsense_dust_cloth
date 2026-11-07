// YAMLの必須設定の読込・起動時検証・実行時動的変更。保存先はGUIで選択する。
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

    publish_raw_depth_ = required_parameter<bool>(*this, "publish_raw_depth");
    debug_publish_hz_ = required_parameter<double>(*this, "debug_publish_hz");
    bg_update_interval_sec_ = required_parameter<double>(*this, "bg_update_interval_sec");

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

    // 5.5. 背景差分 (GPU深度リングバッファ) & 射出連携
    filter_cfg_.use_bg_subtraction = required_parameter<bool>(*this, "use_bg_subtraction");
    filter_cfg_.bg_diff_thresh_m =
        static_cast<float>(required_parameter<double>(*this, "bg_diff_thresh_m"));
    filter_cfg_.bg_ring_frames = required_parameter<int>(*this, "bg_ring_frames");
    filter_cfg_.bg_pre_shot_exclude_sec =
        required_parameter<double>(*this, "bg_pre_shot_exclude_sec");
    filter_cfg_.shot_window_sec = required_parameter<double>(*this, "shot_window_sec");
    shot_count_topic_ = required_parameter<std::string>(*this, "shot_count_topic");
    reset_topic_ = required_parameter<std::string>(*this, "reset_topic");

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
    if (debug_publish_hz_ <= 0 || bg_update_interval_sec_ <= 0 ||
        camera_intrin_.width <= 0 || camera_intrin_.height <= 0 || image_max_age_sec_ <= 0 ||
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
        ballistic_tracker_.vel_smoothing_beta > 1 ||
        filter_cfg_.bg_diff_thresh_m <= 0.0f || filter_cfg_.bg_ring_frames <= 0 ||
        filter_cfg_.bg_pre_shot_exclude_sec < 0.0 || filter_cfg_.shot_window_sec <= 0.0)
        throw std::invalid_argument("Invalid camera/ROI/GPU/tracking parameters in YAML");

    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ClothTrackerCudaNode::on_parameters_change, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
ClothTrackerCudaNode::on_parameters_change(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::lock_guard<std::recursive_mutex> lock(processing_mutex_);
    bool update_tf = false;
    // 変更を一つでも適用する前に、タイミング値を検証する。
    for (const auto& param : params) {
        if ((param.get_name() == "debug_publish_hz" ||
             param.get_name() == "bg_update_interval_sec" ||
             param.get_name() == "static_marker_interval_sec") &&
            (!std::isfinite(param.as_double()) || param.as_double() <= 0)) {
            result.successful = false;
            result.reason = "Timing parameters must be finite and positive";
            return result;
        }
    }

    for (const auto& param : params) {
        const auto& name = param.get_name();
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
            !std::isfinite(param.as_double())) {
            result.successful = false;
            result.reason = "Non-finite double value for " + name;
            return result;
        }

        // Camera mounting pose
        if (name == "camera_pitch_deg") {
            camera_pose_.pitch_deg = static_cast<float>(param.as_double());
            update_tf = true;
        } else if (name == "camera_roll_deg") {
            camera_pose_.roll_deg = static_cast<float>(param.as_double());
            update_tf = true;
        } else if (name == "camera_x_offset_m") {
            camera_pose_.x_offset_m = static_cast<float>(param.as_double());
            update_tf = true;
        } else if (name == "camera_height_offset_m") {
            camera_pose_.height_offset_m = static_cast<float>(param.as_double());
            update_tf = true;
        }
        // ROI & Depth continuity
        else if (name == "min_depth_m") {
            filter_cfg_.min_depth_m = static_cast<float>(param.as_double());
        } else if (name == "max_depth_m") {
            filter_cfg_.max_depth_m = static_cast<float>(param.as_double());
        } else if (name == "min_x_m") {
            filter_cfg_.min_x_m = static_cast<float>(param.as_double());
        } else if (name == "max_x_m") {
            filter_cfg_.max_x_m = static_cast<float>(param.as_double());
        } else if (name == "min_y_m") {
            filter_cfg_.min_y_m = static_cast<float>(param.as_double());
        } else if (name == "max_y_m") {
            filter_cfg_.max_y_m = static_cast<float>(param.as_double());
        } else if (name == "min_z_m") {
            filter_cfg_.min_z_m = static_cast<float>(param.as_double());
        } else if (name == "max_z_m") {
            filter_cfg_.max_z_m = static_cast<float>(param.as_double());
        } else if (name == "continuity_radius_px") {
            filter_cfg_.continuity_radius_px = static_cast<int>(param.as_int());
        } else if (name == "continuity_thresh_m") {
            filter_cfg_.continuity_thresh_m = static_cast<float>(param.as_double());
        } else if (name == "continuity_min_neighbors") {
            filter_cfg_.continuity_min_neighbors = static_cast<int>(param.as_int());
        }
        // Launch Gate
        else if (name == "launch_min_x") {
            ballistic_tracker_.launch_min_x = param.as_double();
            filter_cfg_.launch_min_x = static_cast<float>(param.as_double());
        } else if (name == "launch_max_x") {
            ballistic_tracker_.launch_max_x = param.as_double();
            filter_cfg_.launch_max_x = static_cast<float>(param.as_double());
        } else if (name == "launch_min_y") {
            ballistic_tracker_.launch_min_y = param.as_double();
            filter_cfg_.launch_min_y = static_cast<float>(param.as_double());
        } else if (name == "launch_max_y") {
            ballistic_tracker_.launch_max_y = param.as_double();
            filter_cfg_.launch_max_y = static_cast<float>(param.as_double());
        } else if (name == "launch_min_z") {
            ballistic_tracker_.launch_min_z = param.as_double();
            filter_cfg_.launch_min_z = static_cast<float>(param.as_double());
        } else if (name == "launch_max_z") {
            ballistic_tracker_.launch_max_z = param.as_double();
            filter_cfg_.launch_max_z = static_cast<float>(param.as_double());
        } else if (name == "min_launch_vx") {
            ballistic_tracker_.min_launch_vx = param.as_double();
        } else if (name == "min_launch_move_m") {
            ballistic_tracker_.min_launch_move_m = param.as_double();
        } else if (name == "pending_reset_sec") {
            ballistic_tracker_.launch_timeout_sec = param.as_double();
        } else if (name == "pending_max_drop_m") {
            ballistic_tracker_.pending_max_drop_m = param.as_double();
        }
        // Ballistic prediction & physical guards
        else if (name == "gravity_m_s2") {
            ballistic_tracker_.gravity_m_s2 = param.as_double();
        } else if (name == "drag_coeff") {
            ballistic_tracker_.drag_coeff = param.as_double();
        } else if (name == "vel_smoothing_beta") {
            ballistic_tracker_.vel_smoothing_beta = std::clamp(param.as_double(), 0.0, 1.0);
        } else if (name == "tube_radius_m") {
            ballistic_tracker_.tube_radius_m = param.as_double();
            filter_cfg_.gate_radius_m = static_cast<float>(param.as_double());
        } else if (name == "centroid_position_tolerance_m") {
            ballistic_tracker_.centroid_position_tolerance_m = param.as_double();
        } else if (name == "max_total_accel_m_s2") {
            ballistic_tracker_.max_total_accel_m_s2 = param.as_double();
        } else if (name == "min_vertical_accel_m_s2") {
            ballistic_tracker_.min_vertical_accel_m_s2 = param.as_double();
        } else if (name == "max_lateral_accel_m_s2") {
            ballistic_tracker_.max_lateral_accel_m_s2 = param.as_double();
        } else if (name == "max_heading_deg") {
            ballistic_tracker_.max_heading_deg = param.as_double();
        } else if (name == "heading_min_speed_m_s") {
            ballistic_tracker_.heading_min_speed_m_s = param.as_double();
        } else if (name == "heading_min_step_m") {
            ballistic_tracker_.heading_min_step_m = param.as_double();
        } else if (name == "reject_ascent_after_apex") {
            ballistic_tracker_.reject_ascent_after_apex = param.as_bool();
        } else if (name == "backward_jitter_tolerance_m") {
            ballistic_tracker_.backward_jitter_tolerance_m = param.as_double();
        } else if (name == "vertical_jitter_tolerance_m") {
            ballistic_tracker_.vertical_jitter_tolerance_m = param.as_double();
        } else if (name == "min_flight_vx_m_s") {
            ballistic_tracker_.min_flight_vx_m_s = param.as_double();
        } else if (name == "max_vx_m_s") {
            ballistic_tracker_.max_vx_m_s = param.as_double();
        } else if (name == "max_abs_vy_m_s") {
            ballistic_tracker_.max_abs_vy_m_s = param.as_double();
        } else if (name == "min_vz_m_s") {
            ballistic_tracker_.min_vz_m_s = param.as_double();
        } else if (name == "max_vz_m_s") {
            ballistic_tracker_.max_vz_m_s = param.as_double();
        } else if (name == "abort_on_anomaly") {
            ballistic_tracker_.abort_on_anomaly = param.as_bool();
        } else if (name == "max_anomaly_frames") {
            ballistic_tracker_.max_anomaly_frames = std::max(1, static_cast<int>(param.as_int()));
        } else if (name == "max_lost_frames") {
            ballistic_tracker_.max_lost_frames = std::max(0, static_cast<int>(param.as_int()));
        } else if (name == "max_flight_duration_sec") {
            ballistic_tracker_.max_flight_duration_sec = param.as_double();
        } else if (name == "lost_frame_gate_expansion_m") {
            ballistic_tracker_.lost_frame_gate_expansion_m = param.as_double();
        } else if (name == "max_gate_expansion_m") {
            ballistic_tracker_.max_gate_expansion_m = param.as_double();
        } else if (name == "min_stationary_move_m") {
            ballistic_tracker_.min_stationary_move_m = param.as_double();
        } else if (name == "max_stationary_frames") {
            ballistic_tracker_.max_stationary_frames = static_cast<int>(param.as_int());
        } else if (name == "landing_min_z") {
            ballistic_tracker_.landing_min_z = param.as_double();
        }
        // Voxel & Clustering
        else if (name == "voxel_size_m") {
            filter_cfg_.voxel_size_m = static_cast<float>(param.as_double());
        } else if (name == "cluster_eps_m") {
            filter_cfg_.cluster_eps_m = static_cast<float>(param.as_double());
        } else if (name == "min_cluster_points") {
            const int v = std::max(1, static_cast<int>(param.as_int()));
            filter_cfg_.min_cluster_points = v;
            ballistic_tracker_.min_cluster_points = v;
        } else if (name == "min_tracking_points") {
            const int v = std::max(1, static_cast<int>(param.as_int()));
            filter_cfg_.min_tracking_points = v;
            ballistic_tracker_.min_track_points_limit = v;
        } else if (name == "max_cluster_points") {
            filter_cfg_.max_cluster_points = static_cast<int>(param.as_int());
        } else if (name == "min_cloth_size_m") {
            filter_cfg_.min_cloth_size_m = static_cast<float>(param.as_double());
        } else if (name == "max_cloth_size_m") {
            filter_cfg_.max_cloth_size_m = static_cast<float>(param.as_double());
        } else if (name == "prediction_distance_weight") {
            filter_cfg_.prediction_distance_weight = static_cast<float>(param.as_double());
        } else if (name == "cluster_size_weight") {
            filter_cfg_.cluster_size_weight = static_cast<float>(param.as_double());
        } else if (name == "launch_forward_weight") {
            filter_cfg_.launch_forward_weight = static_cast<float>(param.as_double());
        } else if (name == "external_tracking_keep_radius_m") {
            external_keep_radius_m_ = std::max(0.0, param.as_double());
        } else if (name == "static_marker_interval_sec") {
            static_marker_interval_sec_ = param.as_double();
        } else if (name == "debug_publish_hz") {
            debug_publish_hz_ = param.as_double();
        } else if (name == "bg_update_interval_sec") {
            bg_update_interval_sec_ = param.as_double();
        } else if (name == "publish_raw_depth") {
            publish_raw_depth_ = param.as_bool();
        } else if (name == "publish_debug_cloud") {
            publish_debug_cloud_ = param.as_bool();
            if (publish_debug_cloud_) {
                ensure_debug_publishers();
            }
        } else if (name == "use_bg_subtraction") {
            if (filter_cfg_.use_bg_subtraction != param.as_bool()) {
                reset_background_cuda(cuda_ctx_);
                bg_state_ = BackgroundState::ROLLING;
                last_background_update_sec_ = -1.0;
            }
            filter_cfg_.use_bg_subtraction = param.as_bool();
        } else if (name == "bg_diff_thresh_m") {
            filter_cfg_.bg_diff_thresh_m = static_cast<float>(param.as_double());
        } else if (name == "bg_pre_shot_exclude_sec") {
            filter_cfg_.bg_pre_shot_exclude_sec = param.as_double();
        } else if (name == "shot_window_sec") {
            filter_cfg_.shot_window_sec = param.as_double();
        }

        RCLCPP_INFO(this->get_logger(), "[DynamicParam] %s = %s", name.c_str(),
                    param.value_to_string().c_str());
    }

    if (update_tf) {
        publish_static_transforms();
    }
    if (publish_debug_cloud_ && pub_roi_markers_ && pub_roi_markers_->get_subscription_count() > 0) {
        publish_roi_markers(this->now(), ClothTrackingResult{});
    }

    return result;
}

} // namespace realsense_dust_cloth
