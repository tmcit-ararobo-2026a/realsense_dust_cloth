#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace realsense_dust_cloth {

namespace {
constexpr float DEFAULT_DEPTH_FOCAL_LENGTH_PX = 425.0f;
constexpr double DEFAULT_COLOR_FOCAL_LENGTH_PX = 600.0;
constexpr float METERS_TO_MM = 1000.0f;
constexpr float MAX_UINT16_DEPTH_MM = 65535.0f;
constexpr double BAG_LOOP_RESET_TOLERANCE_SEC = 0.5;
constexpr double MIN_TRAJECTORY_STEP_M = 0.005;
constexpr size_t MAX_TRAJECTORY_POINTS = 1000;
constexpr double STATIC_MARKER_PUBLISH_INTERVAL_SEC = 0.10;
constexpr unsigned int RS_WAIT_TIMEOUT_MS = 1000;

constexpr size_t DEFAULT_QOS_DEPTH = 10;
constexpr size_t SENSOR_QOS_DEPTH = 250;
constexpr int WARN_THROTTLE_MS = 5000;
constexpr int STATUS_LOG_THROTTLE_MS = 1000;

constexpr int ROTATION_90_CW_DEG = 90;
constexpr int ROTATION_180_DEG = 180;
constexpr int ROTATION_270_CW_DEG = 270;
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr size_t PLUMB_BOB_COEFF_COUNT = 5;

constexpr const char* COLOR_OPTICAL_FRAME = "camera_color_optical_frame";
constexpr const char* DEPTH_OPTICAL_FRAME = "camera_depth_optical_frame";
constexpr const char* CLOTH_TF_FRAME = "flying_cloth";

sensor_msgs::msg::Image mat_to_image_msg(const cv::Mat& mat, const rclcpp::Time& stamp, const std::string& frame_id) {
    sensor_msgs::msg::Image img;
    img.header.stamp = stamp;
    img.header.frame_id = frame_id;
    img.height = mat.rows;
    img.width = mat.cols;
    img.encoding = (mat.channels() == 1) ? "mono8" : "bgr8";
    img.is_bigendian = false;
    img.step = static_cast<sensor_msgs::msg::Image::_step_type>(mat.step);
    const size_t size = mat.step * mat.rows;
    img.data.resize(size);
    std::memcpy(img.data.data(), mat.data, size);
    return img;
}

sensor_msgs::msg::CameraInfo build_camera_info_msg(
    uint32_t width, uint32_t height, double fx, double fy, double cx, double cy,
    const std::string& frame_id, const float* coeffs = nullptr) {
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = frame_id;
    info.width = width;
    info.height = height;
    info.distortion_model = "plumb_bob";
    info.d.resize(PLUMB_BOB_COEFF_COUNT, 0.0);
    if (coeffs != nullptr) {
        for (size_t i = 0; i < PLUMB_BOB_COEFF_COUNT; ++i) {
            info.d[i] = coeffs[i];
        }
    }
    info.k[0] = fx;
    info.k[2] = cx;
    info.k[4] = fy;
    info.k[5] = cy;
    info.k[8] = 1.0;
    info.r[0] = 1.0;
    info.r[4] = 1.0;
    info.r[8] = 1.0;
    info.p[0] = fx;
    info.p[2] = cx;
    info.p[5] = fy;
    info.p[6] = cy;
    info.p[10] = 1.0;
    return info;
}
} // namespace

// =============================================================================
// Section 1: Lifecycle & Node Initialization
// =============================================================================

ClothTrackerCudaNode::ClothTrackerCudaNode(const rclcpp::NodeOptions& options)
    : Node("cloth_tracker_cuda_node", options) {
    declare_and_load_parameters();
    setup_map_background();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_static_transforms();
    pub_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/cloth/position", DEFAULT_QOS_DEPTH);
    pub_landing_pos_ = this->create_publisher<geometry_msgs::msg::PointStamped>(landing_position_topic_, DEFAULT_QOS_DEPTH);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/cloth/trajectory_marker", DEFAULT_QOS_DEPTH);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/cloth/measured_path", DEFAULT_QOS_DEPTH);
    pub_rotated_color_ = this->create_publisher<sensor_msgs::msg::Image>("/cloth/camera_view", DEFAULT_QOS_DEPTH);

    // Debug publishers (always available for RViz visualization & verification)
    pub_roi_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cloth/debug/roi_markers", DEFAULT_QOS_DEPTH);
    pub_cluster_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloth/debug/cluster_points", DEFAULT_QOS_DEPTH);
    pub_filtered_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloth/debug/filtered_points", DEFAULT_QOS_DEPTH);

    // Initialize CUDA memory and stream
    if (!init_cuda_tracker(cuda_ctx_, camera_intrin_.width, camera_intrin_.height)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize CUDA tracker context!");
        return;
    }

    const std::string depth_topic = this->get_parameter("depth_topic").as_string();
    const std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    const std::string color_topic = this->get_parameter("color_topic").as_string();

    if (use_direct_realsense_) {
#ifdef HAVE_REALSENSE2
        pub_raw_depth_ = this->create_publisher<sensor_msgs::msg::Image>(depth_topic, DEFAULT_QOS_DEPTH);
        pub_depth_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, DEFAULT_QOS_DEPTH);
        pub_raw_color_ = this->create_publisher<sensor_msgs::msg::Image>(color_topic, DEFAULT_QOS_DEPTH);
        pub_compressed_color_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/camera/color/image_raw/compressed", DEFAULT_QOS_DEPTH);
        pub_color_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera/color/camera_info", DEFAULT_QOS_DEPTH);

        RCLCPP_INFO(this->get_logger(), "Starting Direct librealsense2 C++ API loop (Hardware Native)...");
        is_running_ = true;
        realsense_thread_ = std::thread(&ClothTrackerCudaNode::run_direct_realsense_loop, this);
#else
        RCLCPP_WARN(this->get_logger(), "librealsense2 library not compiled in! Falling back to ROS 2 Image subscription.");
        use_direct_realsense_ = false;
#endif
    }

    if (!use_direct_realsense_) {
        rclcpp::QoS depth_qos(map_depth_queue_size_);
        depth_qos.reliable();

        rclcpp::QoS color_qos(SENSOR_QOS_DEPTH);
        color_qos.best_effort();

        depth_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        color_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions depth_sub_opts;
        depth_sub_opts.callback_group = depth_cb_group_;

        rclcpp::SubscriptionOptions color_sub_opts;
        color_sub_opts.callback_group = color_cb_group_;

        sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, DEFAULT_QOS_DEPTH,
            std::bind(&ClothTrackerCudaNode::camera_info_callback, this, std::placeholders::_1)
        );

        sub_depth_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic, depth_qos,
            std::bind(&ClothTrackerCudaNode::depth_image_callback, this, std::placeholders::_1),
            depth_sub_opts
        );

        if (enable_color_stream_) {
        sub_color_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            color_topic, color_qos,
            std::bind(&ClothTrackerCudaNode::color_image_callback, this, std::placeholders::_1),
            color_sub_opts
        );

        sub_compressed_color_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            color_topic + "/compressed", color_qos,
            std::bind(&ClothTrackerCudaNode::compressed_color_callback, this, std::placeholders::_1),
            color_sub_opts
        );

        }
        RCLCPP_INFO(this->get_logger(), "Listening on Depth Image topic: %s, Color Image topic: %s (+/compressed)",
                    depth_topic.c_str(), color_topic.c_str());
    }

    // Start asynchronous color worker thread
    color_worker_running_ = enable_color_stream_;
    if (enable_color_stream_) color_worker_thread_ = std::thread(&ClothTrackerCudaNode::color_worker_loop, this);

    RCLCPP_INFO(this->get_logger(), "RealSense Cloth CUDA Tracker Node initialized successfully.");
}

ClothTrackerCudaNode::~ClothTrackerCudaNode() {
    is_running_ = false;
    color_worker_running_ = false;
    color_cv_.notify_all();
    if (color_worker_thread_.joinable()) {
        color_worker_thread_.join();
    }
    if (realsense_thread_.joinable()) {
        realsense_thread_.join();
    }
    destroy_cuda_tracker(cuda_ctx_);
}

// =============================================================================
// =============================================================================
// Section 2: ROS 2 Parameter Declaration & Loading (gn10-style unified setup)
// =============================================================================

void ClothTrackerCudaNode::declare_and_load_parameters() {
    // 1. Topic Names & Connection Mode
    this->declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw");
    this->declare_parameter("camera_info_topic", "/camera/camera/depth/camera_info");
    this->declare_parameter("color_topic", "/camera/camera/color/image_raw");
    use_direct_realsense_ = this->declare_parameter("use_direct_realsense", false);

    // 2. Camera Frames & Physical Mount Offsets
    camera_parent_frame_ = this->declare_parameter("camera_parent_frame", "base_link");
    camera_frame_ = this->declare_parameter("camera_frame", "camera_link");
    publish_static_tf_ = this->declare_parameter("publish_static_tf", true);
    camera_intrin_.width = this->declare_parameter("camera_width", 848);
    camera_intrin_.height = this->declare_parameter("camera_height", 480);
    camera_pose_.pitch_deg = static_cast<float>(this->declare_parameter("camera_pitch_deg", 20.0));
    camera_pose_.roll_deg = static_cast<float>(this->declare_parameter("camera_roll_deg", 90.0));
    camera_pose_.x_offset_m = static_cast<float>(this->declare_parameter("camera_x_offset_m", 0.50));
    camera_pose_.height_offset_m = static_cast<float>(this->declare_parameter("camera_height_offset_m", 1.00));
    color_rotation_deg_ = this->declare_parameter("color_rotation_deg", 90);
    depth_fps_ = this->declare_parameter("depth_fps", 60);
    rs_laser_power_ = this->declare_parameter("rs_laser_power", -1.0);
    rs_exposure_us_ = this->declare_parameter("rs_exposure_us", -1);

    // 3. 3D Spatial ROI Filter Bounds
    filter_cfg_.min_depth_m = static_cast<float>(this->declare_parameter("min_depth_m", 0.25));
    filter_cfg_.max_depth_m = static_cast<float>(this->declare_parameter("max_depth_m", 6.00));
    filter_cfg_.min_x_m = static_cast<float>(this->declare_parameter("min_x_m", 0.35));
    filter_cfg_.max_x_m = static_cast<float>(this->declare_parameter("max_x_m", 6.00));
    filter_cfg_.min_y_m = static_cast<float>(this->declare_parameter("min_y_m", -0.60));
    filter_cfg_.max_y_m = static_cast<float>(this->declare_parameter("max_y_m", 0.60));
    filter_cfg_.min_z_m = static_cast<float>(this->declare_parameter("min_z_m", 0.05));
    filter_cfg_.max_z_m = static_cast<float>(this->declare_parameter("max_z_m", 2.40));

    // 4. Voxel Downsampling & DBSCAN Clustering
    filter_cfg_.voxel_size_m = static_cast<float>(this->declare_parameter("voxel_size_m", 0.02));
    filter_cfg_.cluster_eps_m = static_cast<float>(this->declare_parameter("cluster_eps_m", 0.08));
    filter_cfg_.min_cluster_points = this->declare_parameter("min_cluster_points", 45);
    filter_cfg_.max_cluster_points = this->declare_parameter("max_cluster_points", 12000);
    filter_cfg_.min_cloth_size_m = static_cast<float>(this->declare_parameter("min_cloth_size_m", 0.05));
    filter_cfg_.max_cloth_size_m = static_cast<float>(this->declare_parameter("max_cloth_size_m", 0.80));
    filter_cfg_.continuity_thresh_m = static_cast<float>(this->declare_parameter("continuity_thresh_m", 0.08));
    filter_cfg_.continuity_min_neighbors = this->declare_parameter("continuity_min_neighbors", 3);

    // 5. Debug Point Clouds & Streaming Options
    publish_debug_cloud_ = this->declare_parameter("publish_debug_cloud", false);
    trajectory_history_sec_ = this->declare_parameter("trajectory_history_sec", 15.0);
    enable_color_stream_ = this->declare_parameter("enable_color_stream", true);
    color_fps_ = this->declare_parameter("color_fps", 30);
    jpeg_quality_ = this->declare_parameter("jpeg_quality", 65);

    // 6. Ballistic Tracking & Launch Gate Parameters
    enable_ballistic_filter_ = this->declare_parameter("enable_ballistic_filter", true);
    ballistic_tracker_.observed_landing_only = this->declare_parameter("observed_landing_only", true);
    ballistic_tracker_.launch_min_x = this->declare_parameter("launch_min_x", 0.80);
    ballistic_tracker_.launch_max_x = this->declare_parameter("launch_max_x", 1.35);
    ballistic_tracker_.launch_min_y = this->declare_parameter("launch_min_y", -0.25);
    ballistic_tracker_.launch_max_y = this->declare_parameter("launch_max_y", 0.18);
    ballistic_tracker_.launch_min_z = this->declare_parameter("launch_min_z", 1.45);
    ballistic_tracker_.launch_max_z = this->declare_parameter("launch_max_z", 2.10);
    ballistic_tracker_.gate_radius_m = this->declare_parameter("gate_radius_m", 0.70);
    ballistic_tracker_.max_lost_frames = this->declare_parameter("max_lost_frames", 8);
    ballistic_tracker_.min_launch_vx = this->declare_parameter("min_launch_vx", 0.30);
    ballistic_tracker_.min_launch_move_m = this->declare_parameter("min_launch_move_m", 0.05);
    ballistic_tracker_.launch_timeout_sec = this->declare_parameter("pending_reset_sec", 0.25);
    ballistic_tracker_.min_track_points_limit = this->declare_parameter("min_tracking_points", 12);
    ballistic_tracker_.min_cluster_points = filter_cfg_.min_cluster_points;
    ballistic_tracker_.landing_min_z = this->declare_parameter("landing_min_z", 0.15);
    ballistic_tracker_.drag_coeff = this->declare_parameter("drag_coeff", 0.40);
    ballistic_tracker_.vel_smoothing_beta = this->declare_parameter("vel_smoothing_beta", 0.50);
    ballistic_tracker_.min_flight_vx_m_s = this->declare_parameter("min_flight_vx_m_s", -1.50);
    ballistic_tracker_.max_vx_m_s = this->declare_parameter("max_vx_m_s", 15.0);
    ballistic_tracker_.max_abs_vy_m_s = this->declare_parameter("max_abs_vy_m_s", 3.50);
    ballistic_tracker_.min_vz_m_s = this->declare_parameter("min_vz_m_s", -10.0);
    ballistic_tracker_.max_vz_m_s = this->declare_parameter("max_vz_m_s", 12.0);
    ballistic_tracker_.max_flight_duration_sec = this->declare_parameter("max_flight_duration_sec", 2.50);
    ballistic_tracker_.min_stationary_move_m = this->declare_parameter("min_stationary_move_m", 0.12);
    ballistic_tracker_.max_stationary_frames = this->declare_parameter("max_stationary_frames", 5);

    // Synchronize CUDA filter launch gate & tracking points with tracker parameters
    filter_cfg_.min_tracking_points = ballistic_tracker_.min_track_points_limit;
    filter_cfg_.launch_min_x = static_cast<float>(ballistic_tracker_.launch_min_x);
    filter_cfg_.launch_max_x = static_cast<float>(ballistic_tracker_.launch_max_x);
    filter_cfg_.launch_min_y = static_cast<float>(ballistic_tracker_.launch_min_y);
    filter_cfg_.launch_max_y = static_cast<float>(ballistic_tracker_.launch_max_y);
    filter_cfg_.launch_min_z = static_cast<float>(ballistic_tracker_.launch_min_z);
    filter_cfg_.launch_max_z = static_cast<float>(ballistic_tracker_.launch_max_z);
    filter_cfg_.gate_radius_m = static_cast<float>(ballistic_tracker_.gate_radius_m);

    // 7. Landing Position Output Topic
    landing_position_topic_ = this->declare_parameter("landing_position_topic", "/cloth/landing_position");

    // 8. Prior-Map ESDF Background Parameters
    this->declare_parameter("map_file_path", "");
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("map_resolution_m", 0.05);
    this->declare_parameter("map_max_distance_m", 0.50);
    this->declare_parameter("map_min_x_m", -5.50);
    this->declare_parameter("map_max_x_m", 5.50);
    this->declare_parameter("map_min_y_m", -6.00);
    this->declare_parameter("map_max_y_m", 6.00);
    this->declare_parameter("map_min_z_m", -0.20);
    this->declare_parameter("map_max_z_m", 4.00);
    this->declare_parameter("map_max_memory_mb", 128);
    this->declare_parameter("map_exclusion_distance_m", 0.08);
    this->declare_parameter("map_tracking_keep_radius_m", 0.30);
    this->declare_parameter("map_reject_unknown", true);
    this->declare_parameter("map_use_floor", true);
    this->declare_parameter("map_floor_z_m", 0.00);
    this->declare_parameter("map_tf_wait_sec", 0.15);
    this->declare_parameter("map_tf_poll_sec", 0.002);
    this->declare_parameter("map_max_image_age_sec", 0.25);
    this->declare_parameter("map_pose_loss_reset_sec", 0.25);
    this->declare_parameter("map_depth_queue_size", 8);
}

// =============================================================================
// Section 3: Static Coordinate Frame Broadcasting (base_link -> camera_link)
// =============================================================================

void ClothTrackerCudaNode::publish_static_transforms() {
    if (!publish_static_tf_) {
        return;
    }

    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    // 1. base_link -> camera_link (Strictly derived from tracker_params.yaml)
    geometry_msgs::msg::TransformStamped tf_mount;
    tf_mount.header.stamp = rclcpp::Time(0);
    tf_mount.header.frame_id = camera_parent_frame_;
    tf_mount.child_frame_id = camera_frame_;
    tf_mount.transform.translation.x = camera_pose_.x_offset_m;
    tf_mount.transform.translation.y = 0.0;
    tf_mount.transform.translation.z = camera_pose_.height_offset_m;

    double roll_rad = camera_pose_.roll_deg * DEG_TO_RAD;
    double pitch_rad = -camera_pose_.pitch_deg * DEG_TO_RAD; // 上向きはマイナス
    tf2::Quaternion q_pitch, q_roll;
    q_pitch.setRPY(0.0, pitch_rad, 0.0);
    q_roll.setRPY(roll_rad, 0.0, 0.0);
    tf2::Quaternion q_mount = q_pitch * q_roll;
    tf_mount.transform.rotation.x = q_mount.x();
    tf_mount.transform.rotation.y = q_mount.y();
    tf_mount.transform.rotation.z = q_mount.z();
    tf_mount.transform.rotation.w = q_mount.w();
    transforms.push_back(tf_mount);

    // 2. camera_link -> camera_color_optical_frame & camera_depth_optical_frame (Standard ROS optical frame orientation)
    tf2::Quaternion q_opt;
    q_opt.setRPY(-M_PI_2, 0.0, -M_PI_2);

    geometry_msgs::msg::TransformStamped tf_color_opt;
    tf_color_opt.header.stamp = rclcpp::Time(0);
    tf_color_opt.header.frame_id = camera_frame_;
    tf_color_opt.child_frame_id = COLOR_OPTICAL_FRAME;
    tf_color_opt.transform.rotation.x = q_opt.x();
    tf_color_opt.transform.rotation.y = q_opt.y();
    tf_color_opt.transform.rotation.z = q_opt.z();
    tf_color_opt.transform.rotation.w = q_opt.w();
    transforms.push_back(tf_color_opt);

    geometry_msgs::msg::TransformStamped tf_depth_opt = tf_color_opt;
    tf_depth_opt.child_frame_id = DEPTH_OPTICAL_FRAME;
    transforms.push_back(tf_depth_opt);

    tf_static_broadcaster_->sendTransform(transforms);
    RCLCPP_INFO(this->get_logger(),
                "Static TF published directly from config: %s -> %s (x=%.2f, z=%.2f, pitch=%.1f deg, roll=%.1f deg)",
                camera_parent_frame_.c_str(), camera_frame_.c_str(),
                camera_pose_.x_offset_m, camera_pose_.height_offset_m,
                camera_pose_.pitch_deg, camera_pose_.roll_deg);
}

// =============================================================================
// Section 4: Sensor Callbacks (CameraInfo, Depth Image, Color Image)
// =============================================================================

void ClothTrackerCudaNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    std::lock_guard<std::recursive_mutex> lock(processing_mutex_);
    if (!intrinsics_ready_) {
        camera_intrin_.fx = static_cast<float>(msg->k[0]);
        camera_intrin_.fy = static_cast<float>(msg->k[4]);
        camera_intrin_.cx = static_cast<float>(msg->k[2]);
        camera_intrin_.cy = static_cast<float>(msg->k[5]);
        camera_intrin_.width = msg->width;
        camera_intrin_.height = msg->height;
        intrinsics_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera intrinsics loaded: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f, %dx%d",
                    camera_intrin_.fx, camera_intrin_.fy, camera_intrin_.cx, camera_intrin_.cy,
                    camera_intrin_.width, camera_intrin_.height);
    }
}

void ClothTrackerCudaNode::depth_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    std::lock_guard<std::recursive_mutex> lock(processing_mutex_);
    camera_intrin_.width = msg->width;
    camera_intrin_.height = msg->height;
    if (!intrinsics_ready_) {
        publish_background_state("MAP_WAITING_FOR_CAMERA_INFO", false);
        return;
    }

    // Auto-resize CUDA context if incoming image exceeds allocated max_pixels
    int img_pixels = static_cast<int>(msg->width) * msg->height;
    if (img_pixels > cuda_ctx_.max_pixels) {
        destroy_cuda_tracker(cuda_ctx_);
        init_cuda_tracker(cuda_ctx_, msg->width, msg->height);
    }

    std::vector<uint16_t> depth_u16_buf;
    const uint16_t* depth_ptr = nullptr;

    if (msg->encoding == "16UC1" || msg->encoding == "mono16") {
        depth_ptr = reinterpret_cast<const uint16_t*>(msg->data.data());
    } else if (msg->encoding == "32FC1") {
        // Convert Isaac Sim float depth (meters) to uint16 (millimeters)
        size_t num_pixels = static_cast<size_t>(msg->width) * msg->height;
        depth_u16_buf.resize(num_pixels);
        const float* fptr = reinterpret_cast<const float*>(msg->data.data());
        for (size_t i = 0; i < num_pixels; ++i) {
            float d_m = fptr[i];
            if (std::isnan(d_m) || std::isinf(d_m) || d_m <= 0.0f) {
                depth_u16_buf[i] = 0;
            } else {
                float d_mm = d_m * METERS_TO_MM;
                depth_u16_buf[i] = (d_mm > MAX_UINT16_DEPTH_MM) ? static_cast<uint16_t>(MAX_UINT16_DEPTH_MM) : static_cast<uint16_t>(d_mm);
            }
        }
        depth_ptr = depth_u16_buf.data();
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), WARN_THROTTLE_MS,
                             "Expected 16UC1/mono16 or 32FC1 depth encoding, got: %s", msg->encoding.c_str());
        return;
    }

    // Use msg header stamp for robust and exact time delta calculation (works identically on rosbag & real hardware)
    rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0) {
        stamp = this->get_clock()->now();
    }
    process_and_publish(depth_ptr, stamp);
}

void ClothTrackerCudaNode::push_color_frame(const cv::Mat& bgr_mat, const rclcpp::Time& stamp, const sensor_msgs::msg::CameraInfo* info) {
    if (!color_worker_running_ || bgr_mat.empty()) return;
    {
        std::lock_guard<std::mutex> lock(color_mutex_);
        bgr_mat.copyTo(latest_color_task_.bgr_frame);
        latest_color_task_.stamp = stamp;
        latest_color_task_.is_compressed = false;
        if (info) {
            latest_color_task_.color_info = *info;
            latest_color_task_.has_color_info = true;
        } else {
            latest_color_task_.has_color_info = false;
        }
        latest_color_task_.has_new_frame = true;
    }
    color_cv_.notify_one();
}

void ClothTrackerCudaNode::color_worker_loop() {
    std::vector<uint8_t> compressed_buf;
    cv::Mat rotated_frame;

    while (color_worker_running_) {
        ColorTask task;
        {
            std::unique_lock<std::mutex> lock(color_mutex_);
            color_cv_.wait(lock, [this] {
                return !color_worker_running_ || latest_color_task_.has_new_frame;
            });
            if (!color_worker_running_) break;
            task = std::move(latest_color_task_);
            latest_color_task_.has_new_frame = false;
        }

        try {
            if (task.is_compressed) {
                if (task.compressed_data.empty() || !pub_rotated_color_ || pub_rotated_color_->get_subscription_count() == 0) {
                    continue;
                }
                cv::Mat decoded = cv::imdecode(task.compressed_data, cv::IMREAD_COLOR);
                if (!decoded.empty()) {
                    pub_rotated_color_->publish(mat_to_image_msg(decoded, task.stamp, task.frame_id));
                }
                continue;
            }

            if (task.bgr_frame.empty()) continue;

            // 1. Raw color publication if requested
            if (pub_raw_color_ && pub_raw_color_->get_subscription_count() > 0) {
                pub_raw_color_->publish(mat_to_image_msg(task.bgr_frame, task.stamp, COLOR_OPTICAL_FRAME));
            }

            // 2. Rotation
            if (color_rotation_deg_ == ROTATION_180_DEG) {
                cv::rotate(task.bgr_frame, rotated_frame, cv::ROTATE_180);
            } else if (color_rotation_deg_ == ROTATION_90_CW_DEG) {
                cv::rotate(task.bgr_frame, rotated_frame, cv::ROTATE_90_CLOCKWISE);
            } else if (color_rotation_deg_ == ROTATION_270_CW_DEG) {
                cv::rotate(task.bgr_frame, rotated_frame, cv::ROTATE_90_COUNTERCLOCKWISE);
            } else {
                rotated_frame = task.bgr_frame;
            }

            // 3. Rotated uncompressed color (/cloth/camera_view)
            if (pub_rotated_color_ && pub_rotated_color_->get_subscription_count() > 0) {
                pub_rotated_color_->publish(mat_to_image_msg(rotated_frame, task.stamp, COLOR_OPTICAL_FRAME));
            }

            // 4. JPEG Compressed Color (/camera/camera/color/image_raw/compressed)
            if (pub_compressed_color_) {
                const std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
                compressed_buf.clear();
                cv::imencode(".jpg", rotated_frame, compressed_buf, jpeg_params);

                sensor_msgs::msg::CompressedImage comp_msg;
                comp_msg.header.stamp = task.stamp;
                comp_msg.header.frame_id = COLOR_OPTICAL_FRAME;
                comp_msg.format = "jpeg";
                comp_msg.data = std::move(compressed_buf);
                pub_compressed_color_->publish(comp_msg);
            }

            // 5. Camera Info
            if (pub_color_info_) {
                if (task.has_color_info) {
                    task.color_info.header.stamp = task.stamp;
                    task.color_info.header.frame_id = COLOR_OPTICAL_FRAME;
                    pub_color_info_->publish(task.color_info);
                } else {
                    const double fx = (camera_intrin_.fx > 0.0f) ? camera_intrin_.fx : DEFAULT_COLOR_FOCAL_LENGTH_PX;
                    const double fy = (camera_intrin_.fy > 0.0f) ? camera_intrin_.fy : DEFAULT_COLOR_FOCAL_LENGTH_PX;
                    auto cinfo = build_camera_info_msg(
                        task.bgr_frame.cols, task.bgr_frame.rows,
                        fx, fy, task.bgr_frame.cols * 0.5, task.bgr_frame.rows * 0.5,
                        COLOR_OPTICAL_FRAME);
                    cinfo.header.stamp = task.stamp;
                    pub_color_info_->publish(cinfo);
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), WARN_THROTTLE_MS,
                                 "Color worker loop error: %s", e.what());
        }
    }
}

void ClothTrackerCudaNode::color_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    if (!pub_compressed_color_ && !pub_rotated_color_ && !pub_raw_color_) return;
    raw_color_received_ = true;

    try {
        cv::Mat frame;
        if (msg->encoding == "bgr8") {
            frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uchar*>(msg->data.data()), msg->step);
        } else if (msg->encoding == "rgb8") {
            cv::Mat rgb_mat(msg->height, msg->width, CV_8UC3, const_cast<uchar*>(msg->data.data()), msg->step);
            cv::cvtColor(rgb_mat, frame, cv::COLOR_RGB2BGR);
        } else if (msg->encoding == "mono8") {
            frame = cv::Mat(msg->height, msg->width, CV_8UC1, const_cast<uchar*>(msg->data.data()), msg->step);
        } else {
            return;
        }

        rclcpp::Time stamp(msg->header.stamp);
        if (stamp.nanoseconds() == 0) {
            stamp = this->get_clock()->now();
        }
        push_color_frame(frame, stamp);
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), WARN_THROTTLE_MS,
                             "Color decode error: %s", e.what());
    }
}

void ClothTrackerCudaNode::compressed_color_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
    // When replaying a rosbag that only recorded /camera/camera/color/image_raw/compressed,
    // offload JPEG decoding to the asynchronous color worker thread so depth tracking never stalls.
    if (raw_color_received_ || !pub_rotated_color_ || pub_rotated_color_->get_subscription_count() == 0 || msg->data.empty()) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(color_mutex_);
        latest_color_task_.compressed_data = msg->data;
        latest_color_task_.frame_id = msg->header.frame_id;
        latest_color_task_.stamp = rclcpp::Time(msg->header.stamp);
        latest_color_task_.is_compressed = true;
        latest_color_task_.has_new_frame = true;
    }
    color_cv_.notify_one();
}

// =============================================================================
// Section 5: Real-Time Tracking & Visualization Pipeline
// =============================================================================

void ClothTrackerCudaNode::publish_roi_markers(const rclcpp::Time& stamp, const ClothTrackingResult& raw_res) {
    if (!pub_roi_markers_) return;

    double speed = std::sqrt(ballistic_tracker_.vx * ballistic_tracker_.vx +
                             ballistic_tracker_.vy * ballistic_tracker_.vy +
                             ballistic_tracker_.vz * ballistic_tracker_.vz);

    auto markers = MarkerVisualizer::create_roi_markers(
        stamp, filter_cfg_, enable_ballistic_filter_,
        ballistic_tracker_.launch_min_x, ballistic_tracker_.launch_max_x,
        ballistic_tracker_.launch_min_y, ballistic_tracker_.launch_max_y,
        ballistic_tracker_.launch_min_z, ballistic_tracker_.launch_max_z,
        ballistic_tracker_.state == TrackerState::TRACKING,
        ballistic_tracker_.x, ballistic_tracker_.y, ballistic_tracker_.z,
        ballistic_tracker_.vx, ballistic_tracker_.vz, speed,
        raw_res, camera_pose_, trajectory_points_,
        camera_parent_frame_
    );

    pub_roi_markers_->publish(markers);
}

void ClothTrackerCudaNode::process_and_publish(const uint16_t* depth_ptr, const rclcpp::Time& stamp) {
    std::lock_guard<std::recursive_mutex> lock(processing_mutex_);
    double now_sec = stamp.seconds();
    if (last_frame_stamp_sec_ > 0.0 && now_sec < last_frame_stamp_sec_ - BAG_LOOP_RESET_TOLERANCE_SEC) {
        // Rosbag looped back to the beginning: clear all trajectory history and reset tracker
        trajectory_points_.clear();
        ballistic_tracker_.reset();
        has_cloth_pose_ = false;
    }
    last_frame_stamp_sec_ = now_sec;

    if (!prepare_map_frame(stamp)) return;

    std::vector<float3> debug_cluster_pts;
    std::vector<float3> debug_filtered_pts;

    float pred_x = 0.0f, pred_y = 0.0f, pred_z = 0.0f;
    bool has_pred = ballistic_tracker_.get_prediction(stamp.seconds(), pred_x, pred_y, pred_z);

    ClothTrackingResult raw_res = process_depth_frame_cuda(
        cuda_ctx_, depth_ptr, camera_intrin_, camera_pose_, filter_cfg_,
        publish_debug_cloud_ ? &debug_cluster_pts : nullptr, publish_debug_cloud_ ? &debug_filtered_pts : nullptr,
        pred_x, pred_y, pred_z, has_pred, map_filter_
    );

    auto error = cudaGetLastError();
    if (error != cudaSuccess) {
        map_processing_failed_ = true;
        publish_background_state("MAP_PROCESSING_ERROR", false);
        RCLCPP_ERROR(get_logger(), "Map tracking CUDA failure: %s", cudaGetErrorString(error));
        return;
    }

    // Phase 2: Ballistic State Tracking (Filter out stationary walls/ceiling & robot shaking)
    ClothTrackingResult res;
    bool is_actively_tracked = false;
    if (enable_ballistic_filter_) {
        is_actively_tracked = ballistic_tracker_.update(raw_res, stamp.seconds(), res);
    } else {
        res = raw_res;
        is_actively_tracked = raw_res.valid;
    }

    publish_background_state("MAP_READY", ballistic_tracker_.state != TrackerState::TRACKING);

    // Publish Debug Point Clouds & ROI Markers after tracker state update
    if (publish_debug_cloud_) {
        MarkerVisualizer::publish_point_cloud(pub_filtered_cloud_, debug_filtered_pts, stamp, camera_parent_frame_);
        publish_roi_markers(stamp, raw_res);
    }

    // Publish cyan cluster ONLY when actively tracked in this frame; clear immediately otherwise
    if (is_actively_tracked && !debug_cluster_pts.empty()) {
        MarkerVisualizer::publish_point_cloud(pub_cluster_cloud_, debug_cluster_pts, stamp, camera_parent_frame_);
    } else {
        std::vector<float3> empty_pts;
        MarkerVisualizer::publish_point_cloud(pub_cluster_cloud_, empty_pts, stamp, camera_parent_frame_);
    }

    const char* state_str = "IDLE";
    switch (ballistic_tracker_.state) {
        case TrackerState::TRACKING: state_str = "TRACKING"; break;
        case TrackerState::LANDED:   state_str = "LANDED"; break;
        case TrackerState::LOST:     state_str = "LOST"; break;
        case TrackerState::RESTING:  state_str = "RESTING"; break;
        default:                     state_str = "IDLE"; break;
    }

    // Diagnostics to console every 1 second
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), STATUS_LOG_THROTTLE_MS,
        "[ClothTracker] ROI points: %zu | Candidate cluster: %d pts at (%.2f, %.2f, %.2f) | State: %s",
        debug_filtered_pts.size(), raw_res.cluster_points_count, raw_res.cx, raw_res.cy, raw_res.cz,
        state_str
    );

    if (is_actively_tracked && ballistic_tracker_.state == TrackerState::LANDED) {
        last_cloth_x_ = res.cx;
        last_cloth_y_ = res.cy;
        last_cloth_z_ = res.cz;
    }
    if (prev_state_ == TrackerState::TRACKING && ballistic_tracker_.state == TrackerState::LANDED) {
        if (pub_landing_pos_) {
            geometry_msgs::msg::PointStamped landing_msg;
            landing_msg.header.stamp = stamp;
            landing_msg.header.frame_id = camera_parent_frame_;
            landing_msg.point.x = last_cloth_x_;
            landing_msg.point.y = last_cloth_y_;
            landing_msg.point.z = last_cloth_z_;
            pub_landing_pos_->publish(landing_msg);
        }

        RCLCPP_INFO(this->get_logger(),
                    "========================================================================\n"
                    "*** CLOTH LANDED at (%.2f, %.2f, %.2f)! Recorded %zu flight points. ***\n"
                    "========================================================================",
                    last_cloth_x_, last_cloth_y_, last_cloth_z_, trajectory_points_.size());
    }
    prev_state_ = ballistic_tracker_.state;

    if (is_actively_tracked) {
        RCLCPP_INFO(this->get_logger(),
                    "[ClothTracker] >>> TRACKED CLOTH: points=%d, pos=(%.2f, %.2f, %.2f) <<<",
                    res.cluster_points_count, res.cx, res.cy, res.cz);

        last_cloth_x_ = res.cx;
        last_cloth_y_ = res.cy;
        last_cloth_z_ = res.cz;
        has_cloth_pose_ = true;

        if (ballistic_tracker_.tracked_count <= 1) {
            trajectory_points_.clear(); // Brand new launch: clear previous trajectory line
            if (enable_ballistic_filter_) {
                geometry_msgs::msg::Point launch_pt;
                launch_pt.x = ballistic_tracker_.launch_x;
                launch_pt.y = ballistic_tracker_.launch_y;
                launch_pt.z = ballistic_tracker_.launch_z;
                trajectory_points_.emplace_back(ballistic_tracker_.launch_stamp, launch_pt);
            }
            RCLCPP_INFO(this->get_logger(),
                        "========================================================================\n"
                        "*** LAUNCH CONFIRMED at (%.2f, %.2f, %.2f) with %d pts! Tracking locked! ***\n"
                        "========================================================================",
                        res.cx, res.cy, res.cz, res.cluster_points_count);
        }

        // Publish Centroid Point (Strictly real measured point)
        geometry_msgs::msg::PointStamped pt_msg;
        pt_msg.header.stamp = stamp;
        pt_msg.header.frame_id = camera_parent_frame_;
        pt_msg.point.x = res.cx;
        pt_msg.point.y = res.cy;
        pt_msg.point.z = res.cz;
        pub_point_->publish(pt_msg);

        // Trajectory History (Strictly true measured points advancing forward)
        geometry_msgs::msg::Point p_geom;
        p_geom.x = res.cx;
        p_geom.y = res.cy;
        p_geom.z = res.cz;

        if (trajectory_points_.empty()) {
            trajectory_points_.emplace_back(now_sec, p_geom);
        } else {
            const auto& prev_pt = trajectory_points_.back().second;
            double dx = p_geom.x - prev_pt.x;
            double dy = p_geom.y - prev_pt.y;
            double dz = p_geom.z - prev_pt.z;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist >= MIN_TRAJECTORY_STEP_M) {
                trajectory_points_.emplace_back(now_sec, p_geom);
            }
        }

        while (trajectory_history_sec_ > 0.0 &&
               trajectory_points_.size() > 1 &&
               (now_sec - trajectory_points_.front().first) > trajectory_history_sec_) {
            trajectory_points_.erase(trajectory_points_.begin());
        }
        if (trajectory_points_.size() > MAX_TRAJECTORY_POINTS) {
            trajectory_points_.erase(trajectory_points_.begin());
        }
    }

    // 1. Broadcast TF: base_link -> flying_cloth (Every frame while tracked or landed so RViz TF axes NEVER flicker or expire)
    if (has_cloth_pose_ && is_actively_tracked) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = camera_parent_frame_;
        tf_msg.child_frame_id = CLOTH_TF_FRAME;
        tf_msg.transform.translation.x = last_cloth_x_;
        tf_msg.transform.translation.y = last_cloth_y_;
        tf_msg.transform.translation.z = last_cloth_z_;
        tf_msg.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    // 2. Cloth Tip Ball Marker (Red Sphere, diameter 12cm at current or final landing position)
    if (has_cloth_pose_) {
        pub_marker_->publish(MarkerVisualizer::create_cloth_tip_marker(
            stamp, last_cloth_x_, last_cloth_y_, last_cloth_z_, camera_parent_frame_));
    }

    // 3. PERSISTENT VISUALIZATION: Strictly true measured flight trajectory (NO fake prediction lines!)
    // When actively tracking: publish every frame.
    // When landed/idle: publish at 10Hz so the line and markers stay solidly on RViz screen without disappearing.
    bool should_publish_markers = is_actively_tracked || (std::abs(now_sec - last_static_publish_sec_) >= STATIC_MARKER_PUBLISH_INTERVAL_SEC);

    if (should_publish_markers && !trajectory_points_.empty()) {
        last_static_publish_sec_ = now_sec;

        // 1. Measured Flight Trajectory Line Strip (Bright Gold, strictly connects true camera detections)
        pub_marker_->publish(MarkerVisualizer::create_trajectory_line(stamp, trajectory_points_, camera_parent_frame_));

        // 2. True Measured Points Marker (Orange Spheres, exactly where camera detected cloth)
        pub_marker_->publish(MarkerVisualizer::create_measured_points(stamp, trajectory_points_, camera_parent_frame_));

        // 3. Measured Path (True measured points only)
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = camera_parent_frame_;
        for (const auto& item : trajectory_points_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = stamp;
            pose.header.frame_id = camera_parent_frame_;
            pose.pose.position = item.second;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        pub_path_->publish(path_msg);
    }
}

// =============================================================================
// Section 6: Native librealsense Hardware Streaming (Native C++ API)
// =============================================================================

void ClothTrackerCudaNode::run_direct_realsense_loop() {
#ifdef HAVE_REALSENSE2
    try {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH, camera_intrin_.width, camera_intrin_.height, RS2_FORMAT_Z16, depth_fps_);
        if (enable_color_stream_) {
            cfg.enable_stream(RS2_STREAM_COLOR, camera_intrin_.width, camera_intrin_.height, RS2_FORMAT_BGR8, color_fps_);
        }

        rs_pipe_ = std::make_unique<rs2::pipeline>();
        rs2::pipeline_profile profile = rs_pipe_->start(cfg);

        // Hardware Controls: IR laser power, emitter, and exposure
        try {
            auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
            if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
                depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.0f);
            }
            if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
                if (rs_laser_power_ < 0.0) {
                    auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
                    depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max);
                    RCLCPP_INFO(this->get_logger(), "RealSense Laser Power set to maximum (%.1f mW) for high-speed tracking", range.max);
                } else {
                    depth_sensor.set_option(RS2_OPTION_LASER_POWER, static_cast<float>(rs_laser_power_));
                    RCLCPP_INFO(this->get_logger(), "RealSense Laser Power set to %.1f mW", rs_laser_power_);
                }
            }
            if (depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                if (rs_exposure_us_ > 0) {
                    depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
                    if (depth_sensor.supports(RS2_OPTION_EXPOSURE)) {
                        depth_sensor.set_option(RS2_OPTION_EXPOSURE, static_cast<float>(rs_exposure_us_));
                        RCLCPP_INFO(this->get_logger(), "RealSense Depth Manual Exposure set to %d us", rs_exposure_us_);
                    }
                } else {
                    depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
                    RCLCPP_INFO(this->get_logger(), "RealSense Depth Auto Exposure enabled");
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Could not set RealSense sensor options: %s", e.what());
        }

        // Fetch camera intrinsics from device
        auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        rs2_intrinsics intrin = depth_stream.get_intrinsics();
        camera_intrin_.fx = intrin.fx;
        camera_intrin_.fy = intrin.fy;
        camera_intrin_.cx = intrin.ppx;
        camera_intrin_.cy = intrin.ppy;
        camera_intrin_.width = intrin.width;
        camera_intrin_.height = intrin.height;
        intrinsics_ready_ = true;

        int stream_pixels = intrin.width * intrin.height;
        if (stream_pixels > cuda_ctx_.max_pixels) {
            destroy_cuda_tracker(cuda_ctx_);
            init_cuda_tracker(cuda_ctx_, intrin.width, intrin.height);
        }

        sensor_msgs::msg::CameraInfo color_info_msg;
        if (enable_color_stream_) {
            try {
                auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
                rs2_intrinsics c_intrin = color_stream.get_intrinsics();
                color_info_msg = build_camera_info_msg(
                    c_intrin.width, c_intrin.height,
                    c_intrin.fx, c_intrin.fy, c_intrin.ppx, c_intrin.ppy,
                    COLOR_OPTICAL_FRAME, c_intrin.coeffs);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to extract color intrinsics: %s", e.what());
                color_info_msg = build_camera_info_msg(
                    camera_intrin_.width, camera_intrin_.height,
                    DEFAULT_COLOR_FOCAL_LENGTH_PX, DEFAULT_COLOR_FOCAL_LENGTH_PX,
                    camera_intrin_.width * 0.5, camera_intrin_.height * 0.5,
                    COLOR_OPTICAL_FRAME);
            }
        }

        RCLCPP_INFO(this->get_logger(), "RealSense Hardware Pipeline streaming: Depth %dx%d @ %dfps (Color enabled=%d)",
                    intrin.width, intrin.height, depth_fps_, static_cast<int>(enable_color_stream_));

        // Pre-build depth CameraInfo msg
        sensor_msgs::msg::CameraInfo depth_info_msg = build_camera_info_msg(
            intrin.width, intrin.height,
            intrin.fx, intrin.fy, intrin.ppx, intrin.ppy,
            DEPTH_OPTICAL_FRAME, intrin.coeffs);

        while (rclcpp::ok() && is_running_) {
            rs2::frameset frames = rs_pipe_->wait_for_frames(RS_WAIT_TIMEOUT_MS);
            rclcpp::Time now = this->get_clock()->now();

            // 1. Process Depth for CUDA cloth tracking (native GPU)
            rs2::depth_frame depth = frames.get_depth_frame();
            if (depth) {
                const uint16_t* depth_ptr = reinterpret_cast<const uint16_t*>(depth.get_data());
                process_and_publish(depth_ptr, now);

                // Publish raw depth & camera_info ONLY when subscribed (e.g. ros2 bag record) to ensure ZERO overhead normally
                bool depth_subscribed = (pub_raw_depth_ && pub_raw_depth_->get_subscription_count() > 0) ||
                                        (pub_depth_info_ && pub_depth_info_->get_subscription_count() > 0);
                if (depth_subscribed && pub_raw_depth_) {
                    sensor_msgs::msg::Image depth_img;
                    depth_img.header.stamp = now;
                    depth_img.header.frame_id = DEPTH_OPTICAL_FRAME;
                    depth_img.height = depth.get_height();
                    depth_img.width = depth.get_width();
                    depth_img.encoding = "16UC1";
                    depth_img.is_bigendian = false;
                    depth_img.step = depth.get_stride_in_bytes();
                    size_t depth_size = static_cast<size_t>(depth.get_stride_in_bytes()) * depth.get_height();
                    depth_img.data.resize(depth_size);
                    std::memcpy(depth_img.data.data(), depth.get_data(), depth_size);
                    pub_raw_depth_->publish(depth_img);

                    if (pub_depth_info_) {
                        depth_info_msg.header.stamp = now;
                        pub_depth_info_->publish(depth_info_msg);
                    }
                }
            }

            // 2. Color Image Processing: Offloaded asynchronously to worker thread
            if (enable_color_stream_ && (pub_compressed_color_ || pub_rotated_color_ || pub_raw_color_)) {
                rs2::video_frame color = frames.get_color_frame();
                if (color) {
                    cv::Mat color_mat(cv::Size(color.get_width(), color.get_height()), CV_8UC3,
                                      const_cast<void*>(color.get_data()), cv::Mat::AUTO_STEP);
                    push_color_frame(color_mat, now, pub_color_info_ ? &color_info_msg : nullptr);
                }
            }
        }

        rs_pipe_->stop();
    } catch (const rs2::error& e) {
        RCLCPP_ERROR(this->get_logger(), "RealSense Exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
#endif
}

} // namespace realsense_dust_cloth

// =============================================================================
// Section 7: ROS 2 Main Entrypoint
// =============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<realsense_dust_cloth::ClothTrackerCudaNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
