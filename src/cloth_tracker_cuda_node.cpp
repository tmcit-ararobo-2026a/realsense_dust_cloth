#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>

namespace realsense_dust_cloth {



ClothTrackerCudaNode::ClothTrackerCudaNode(const rclcpp::NodeOptions& options)
    : Node("cloth_tracker_cuda_node", options) {
    declare_parameters();
    load_parameters();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    pub_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/cloth/position", 10);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/cloth/trajectory_marker", 10);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/cloth/measured_path", 10);
    pub_raw_color_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", 10);
    pub_compressed_color_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/camera/color/image_raw/compressed", 10);
    pub_color_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera/color/camera_info", 10);
    pub_rotated_color_ = this->create_publisher<sensor_msgs::msg::Image>("/cloth/camera_view", 10);

    // Debug publishers (always available for RViz visualization & verification)
    pub_roi_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cloth/debug/roi_markers", 10);
    pub_cluster_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloth/debug/cluster_points", 10);
    pub_filtered_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloth/debug/filtered_points", 10);

    // Initialize CUDA memory and stream
    if (!init_cuda_tracker(cuda_ctx_, camera_intrin_.width, camera_intrin_.height)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize CUDA tracker context!");
        return;
    }

    if (use_direct_realsense_) {
#ifdef HAVE_REALSENSE2
        RCLCPP_INFO(this->get_logger(), "Starting Direct librealsense2 C++ API loop (Hardware Native)...");
        is_running_ = true;
        realsense_thread_ = std::thread(&ClothTrackerCudaNode::run_direct_realsense_loop, this);
#else
        RCLCPP_WARN(this->get_logger(), "librealsense2 library not compiled in! Falling back to ROS 2 Image subscription.");
        use_direct_realsense_ = false;
#endif
    }

    if (!use_direct_realsense_) {
        std::string depth_topic = this->get_parameter("depth_topic").as_string();
        std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
        std::string color_topic = this->get_parameter("color_topic").as_string();

        rclcpp::QoS qos(120);
        qos.best_effort();

        sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, 10,
            std::bind(&ClothTrackerCudaNode::camera_info_callback, this, std::placeholders::_1)
        );

        sub_depth_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic, qos,
            std::bind(&ClothTrackerCudaNode::depth_image_callback, this, std::placeholders::_1)
        );

        sub_color_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            color_topic, qos,
            std::bind(&ClothTrackerCudaNode::color_image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Listening on Depth Image topic: %s, Color Image topic: %s",
                    depth_topic.c_str(), color_topic.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "RealSense Cloth CUDA Tracker Node initialized successfully.");
}

ClothTrackerCudaNode::~ClothTrackerCudaNode() {
    is_running_ = false;
    if (realsense_thread_.joinable()) {
        realsense_thread_.join();
    }
    destroy_cuda_tracker(cuda_ctx_);
}

void ClothTrackerCudaNode::declare_parameters() {
    this->declare_parameter("tracking_mode", "depth");
    this->declare_parameter("points_topic", "/camera/camera/depth/color/points");
    this->declare_parameter("use_direct_realsense", false);
    this->declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw");
    this->declare_parameter("camera_info_topic", "/camera/camera/depth/camera_info");
    this->declare_parameter("color_topic", "/camera/camera/color/image_raw");

    this->declare_parameter("camera_width", 848);
    this->declare_parameter("camera_height", 480);
    this->declare_parameter("camera_pitch_deg", 20.0);
    this->declare_parameter("camera_roll_deg", 90.0);
    this->declare_parameter("camera_x_offset_m", 0.15);
    this->declare_parameter("camera_height_offset_m", 0.35);

    this->declare_parameter("min_depth_m", 0.25);
    this->declare_parameter("max_depth_m", 4.5);

    this->declare_parameter("min_x_m", 0.30);
    this->declare_parameter("max_x_m", 5.50);
    this->declare_parameter("min_y_m", -1.50);
    this->declare_parameter("max_y_m", 1.50);
    this->declare_parameter("min_z_m", 0.00);
    this->declare_parameter("max_z_m", 3.50);

    this->declare_parameter("voxel_size_m", 0.02);
    this->declare_parameter("cluster_eps_m", 0.15);
    this->declare_parameter("min_cluster_points", 10);
    this->declare_parameter("max_cluster_points", 6000);
    this->declare_parameter("max_cloth_size_m", 0.85);
    this->declare_parameter("use_bg_subtraction", true);
    this->declare_parameter("bg_diff_thresh_m", 0.08);
    this->declare_parameter("bg_target_frames", 10);

    this->declare_parameter("publish_debug_cloud", false);
    this->declare_parameter("trajectory_history_sec", 15.0);
    this->declare_parameter("enable_color_stream", true);
    this->declare_parameter("color_fps", 30);
    this->declare_parameter("jpeg_quality", 65);
    this->declare_parameter("color_rotation_deg", 180);

    // Ballistic Filter & Launch Gate parameters
    this->declare_parameter("enable_ballistic_filter", true);
    this->declare_parameter("launch_min_x", 0.30);
    this->declare_parameter("launch_max_x", 1.80);
    this->declare_parameter("launch_min_y", -1.00);
    this->declare_parameter("launch_max_y", 1.00);
    this->declare_parameter("launch_min_z", 0.50);
    this->declare_parameter("launch_max_z", 2.00);
    this->declare_parameter("gate_radius_m", 0.60);
    this->declare_parameter("max_lost_frames", 15);
    this->declare_parameter("min_launch_vx", 0.10);
}



void ClothTrackerCudaNode::load_parameters() {
    use_direct_realsense_ = this->get_parameter("use_direct_realsense").as_bool();
    camera_intrin_.width = this->get_parameter("camera_width").as_int();
    camera_intrin_.height = this->get_parameter("camera_height").as_int();

    camera_pose_.pitch_deg = static_cast<float>(this->get_parameter("camera_pitch_deg").as_double());
    camera_pose_.roll_deg = static_cast<float>(this->get_parameter("camera_roll_deg").as_double());
    camera_pose_.x_offset_m = static_cast<float>(this->get_parameter("camera_x_offset_m").as_double());
    camera_pose_.height_offset_m = static_cast<float>(this->get_parameter("camera_height_offset_m").as_double());
    color_rotation_deg_ = this->get_parameter("color_rotation_deg").as_int();

    filter_cfg_.min_depth_m = static_cast<float>(this->get_parameter("min_depth_m").as_double());
    filter_cfg_.max_depth_m = static_cast<float>(this->get_parameter("max_depth_m").as_double());

    filter_cfg_.min_x_m = static_cast<float>(this->get_parameter("min_x_m").as_double());
    filter_cfg_.max_x_m = static_cast<float>(this->get_parameter("max_x_m").as_double());
    filter_cfg_.min_y_m = static_cast<float>(this->get_parameter("min_y_m").as_double());
    filter_cfg_.max_y_m = static_cast<float>(this->get_parameter("max_y_m").as_double());
    filter_cfg_.min_z_m = static_cast<float>(this->get_parameter("min_z_m").as_double());
    filter_cfg_.max_z_m = static_cast<float>(this->get_parameter("max_z_m").as_double());

    filter_cfg_.voxel_size_m = static_cast<float>(this->get_parameter("voxel_size_m").as_double());
    filter_cfg_.cluster_eps_m = static_cast<float>(this->get_parameter("cluster_eps_m").as_double());
    filter_cfg_.min_cluster_points = this->get_parameter("min_cluster_points").as_int();
    filter_cfg_.max_cluster_points = this->get_parameter("max_cluster_points").as_int();
    filter_cfg_.max_cloth_size_m = static_cast<float>(this->get_parameter("max_cloth_size_m").as_double());
    filter_cfg_.use_bg_subtraction = this->get_parameter("use_bg_subtraction").as_bool();
    filter_cfg_.bg_diff_thresh_m = static_cast<float>(this->get_parameter("bg_diff_thresh_m").as_double());
    bg_target_frames_ = this->get_parameter("bg_target_frames").as_int();

    publish_debug_cloud_ = this->get_parameter("publish_debug_cloud").as_bool();
    trajectory_history_sec_ = this->get_parameter("trajectory_history_sec").as_double();
    enable_color_stream_ = this->get_parameter("enable_color_stream").as_bool();
    color_fps_ = this->get_parameter("color_fps").as_int();
    jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();

    enable_ballistic_filter_ = this->get_parameter("enable_ballistic_filter").as_bool();
    ballistic_tracker_.launch_min_x = this->get_parameter("launch_min_x").as_double();
    ballistic_tracker_.launch_max_x = this->get_parameter("launch_max_x").as_double();
    ballistic_tracker_.launch_min_y = this->get_parameter("launch_min_y").as_double();
    ballistic_tracker_.launch_max_y = this->get_parameter("launch_max_y").as_double();
    ballistic_tracker_.launch_min_z = this->get_parameter("launch_min_z").as_double();
    ballistic_tracker_.launch_max_z = this->get_parameter("launch_max_z").as_double();
    ballistic_tracker_.gate_radius_m = this->get_parameter("gate_radius_m").as_double();
    ballistic_tracker_.max_lost_frames = this->get_parameter("max_lost_frames").as_int();
    ballistic_tracker_.min_launch_vx = this->get_parameter("min_launch_vx").as_double();
}



void ClothTrackerCudaNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
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

void ClothTrackerCudaNode::depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!intrinsics_ready_) {
        // Fallback default intrinsics if CameraInfo has not arrived yet
        camera_intrin_.width = msg->width;
        camera_intrin_.height = msg->height;
        camera_intrin_.fx = 425.0f; // Approx for D435 848x480
        camera_intrin_.fy = 425.0f;
        camera_intrin_.cx = msg->width * 0.5f;
        camera_intrin_.cy = msg->height * 0.5f;
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
                float d_mm = d_m * 1000.0f;
                depth_u16_buf[i] = (d_mm > 65535.0f) ? 65535 : static_cast<uint16_t>(d_mm);
            }
        }
        depth_ptr = depth_u16_buf.data();
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
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

void ClothTrackerCudaNode::color_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!pub_compressed_color_) return;

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

        if (color_rotation_deg_ == 180) {
            cv::rotate(frame, frame, cv::ROTATE_180);
        } else if (color_rotation_deg_ == 90) {
            cv::rotate(frame, frame, cv::ROTATE_90_CLOCKWISE);
        } else if (color_rotation_deg_ == 270) {
            cv::rotate(frame, frame, cv::ROTATE_90_COUNTERCLOCKWISE);
        }

        if (pub_rotated_color_) {
            sensor_msgs::msg::Image out_img;
            out_img.header = msg->header;
            out_img.height = frame.rows;
            out_img.width = frame.cols;
            out_img.encoding = "bgr8";
            out_img.is_bigendian = false;
            out_img.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
            size_t size = frame.step * frame.rows;
            out_img.data.resize(size);
            std::memcpy(out_img.data.data(), frame.data, size);
            pub_rotated_color_->publish(out_img);
        }

        std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        std::vector<uint8_t> compressed_buf;
        cv::imencode(".jpg", frame, compressed_buf, jpeg_params);

        sensor_msgs::msg::CompressedImage comp_msg;
        comp_msg.header = msg->header;
        comp_msg.format = "jpeg";
        comp_msg.data = std::move(compressed_buf);
        pub_compressed_color_->publish(comp_msg);

        if (pub_color_info_) {
            sensor_msgs::msg::CameraInfo cinfo;
            cinfo.header = msg->header;
            cinfo.header.frame_id = "camera_color_optical_frame";
            cinfo.width = msg->width;
            cinfo.height = msg->height;
            cinfo.distortion_model = "plumb_bob";
            cinfo.d.resize(5, 0.0);
            cinfo.k[0] = (camera_intrin_.fx > 0.0f) ? camera_intrin_.fx : 600.0;
            cinfo.k[2] = msg->width * 0.5;
            cinfo.k[4] = (camera_intrin_.fy > 0.0f) ? camera_intrin_.fy : 600.0;
            cinfo.k[5] = msg->height * 0.5;
            cinfo.k[8] = 1.0;
            cinfo.r[0] = 1.0; cinfo.r[4] = 1.0; cinfo.r[8] = 1.0;
            cinfo.p[0] = cinfo.k[0]; cinfo.p[2] = cinfo.k[2];
            cinfo.p[5] = cinfo.k[4]; cinfo.p[6] = cinfo.k[5];
            cinfo.p[10] = 1.0;
            pub_color_info_->publish(cinfo);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Color compression relay error: %s", e.what());
    }
}

bool ClothTrackerCudaNode::BallisticTracker::update(
    const ClothTrackingResult& raw, double stamp_sec, ClothTrackingResult& out)
{
    if (state == TrackerState::IDLE || state == TrackerState::LANDED) {
        if (!raw.valid) {
            has_pending = false;
            return false;
        }

        // Lock onto genuine newly ejected cloth only (substantial points >= 20 to reject edge noise)
        if (raw.cx >= launch_min_x && raw.cx <= launch_max_x &&
            raw.cy >= launch_min_y && raw.cy <= launch_max_y &&
            raw.cz >= launch_min_z && raw.cz <= launch_max_z &&
            raw.cluster_points_count >= 20)
        {
            state = TrackerState::TRACKING;
            x = raw.cx;
            y = raw.cy;
            z = raw.cz;
            vx = 2.0;
            vy = 0.0;
            vz = 0.5;
            last_stamp = stamp_sec;
            tracked_count = 1;
            lost_count = 0;
            has_pending = false;
            out = raw;
            return true;
        } else {
            has_pending = false;
            return false;
        }
    } else { // TRACKING: Persistently follow this single cloth cluster until it touches the ground!
        double dt = stamp_sec - last_stamp;
        if (dt <= 0.0 || dt > 0.40) {
            dt = 0.033;
        }

        // Ballistic prediction with aerodynamic drag deceleration & gravity acceleration
        double drag_factor = std::max(0.60, 1.0 - 0.8 * dt);
        double pred_x = x + vx * dt * drag_factor;
        double pred_y = y + vy * dt;
        double pred_z = z + vz * dt - 0.5 * 9.8 * dt * dt;
        vz -= 9.8 * dt; // Continuously apply gravity

        bool matched = false;
        if (raw.valid && raw.cluster_points_count >= 6) {
            double dx_prev = raw.cx - x;
            double dy_prev = raw.cy - y;
            double dz_prev = raw.cz - z;
            double dist_prev = std::sqrt(dx_prev * dx_prev + dy_prev * dy_prev + dz_prev * dz_prev);

            double dx_pred = raw.cx - pred_x;
            double dy_pred = raw.cy - pred_y;
            double dz_pred = raw.cz - pred_z;
            double dist_pred = std::sqrt(dx_pred * dx_pred + dy_pred * dy_pred + dz_pred * dz_pred);

            // Persistently track cloth: within 2.0m of prediction or previous location and advancing
            if (dist_pred <= 2.00 || (dist_prev <= 2.00 && dx_prev >= -0.30)) {
                matched = true;
                double dt_safe = std::clamp(dt, 0.005, 0.100);

                double new_vx = (raw.cx - x) / dt_safe;
                double new_vy = (raw.cy - y) / dt_safe;
                double new_vz = (raw.cz - z) / dt_safe;

                vx = 0.6 * vx + 0.4 * new_vx;
                vy = 0.6 * vy + 0.4 * new_vy;
                vz = 0.6 * vz + 0.4 * new_vz;

                x = raw.cx;
                y = raw.cy;
                z = raw.cz;

                last_stamp = stamp_sec;
                tracked_count++;
                lost_count = 0;

                out = raw;
                out.cx = static_cast<float>(x);
                out.cy = static_cast<float>(y);
                out.cz = static_cast<float>(z);

                // Stop tracking ONLY when cloth truly touches the floor plane (Z <= 8cm) or stops
                if (z <= 0.08 || (z <= 0.15 && vx <= 0.20) || x >= 5.5) {
                    state = TrackerState::LANDED;
                }
                return true;
            }
        }

        if (!matched) {
            lost_count++;
            if (lost_count > 25 || pred_z <= 0.04 || pred_x >= 5.20) {
                state = TrackerState::LANDED;
                return false;
            }

            // Maintain continuous search gate
            x = pred_x;
            y = pred_y;
            z = std::max(0.04, pred_z);
            last_stamp = stamp_sec;
            return false;
        }
    }
    return false;
}


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
        raw_res, camera_pose_, trajectory_points_
    );

    pub_roi_markers_->publish(markers);
}

void ClothTrackerCudaNode::process_and_publish(const uint16_t* depth_ptr, const rclcpp::Time& stamp) {
    double now_sec = stamp.seconds();
    if (last_frame_stamp_sec_ > 0.0 && now_sec < last_frame_stamp_sec_ - 0.5) {
        // Rosbag looped back to the beginning: clear all trajectory history and reset tracker
        trajectory_points_.clear();
        ballistic_tracker_.reset();
        bg_frames_collected_ = 0;
        has_cloth_pose_ = false;
    }
    last_frame_stamp_sec_ = now_sec;

    // Phase 1: Background Calibration (learn static walls, ceiling, furniture)
    if (filter_cfg_.use_bg_subtraction && bg_frames_collected_ < bg_target_frames_) {
        update_background_cuda(cuda_ctx_, depth_ptr, camera_intrin_.width, camera_intrin_.height);
        bg_frames_collected_++;
        if (bg_frames_collected_ == bg_target_frames_) {
            RCLCPP_INFO(this->get_logger(), "=== BACKGROUND CALIBRATED (%d frames)! Static walls, ceiling and furniture are now IGNORED. ===", bg_target_frames_);
        }
        return;
    }

    std::vector<float3> debug_cluster_pts;
    std::vector<float3> debug_filtered_pts;

    float pred_x = 0.0f, pred_y = 0.0f, pred_z = 0.0f;
    bool has_pred = ballistic_tracker_.get_prediction(stamp.seconds(), pred_x, pred_y, pred_z);

    ClothTrackingResult raw_res = process_depth_frame_cuda(
        cuda_ctx_, depth_ptr, camera_intrin_, camera_pose_, filter_cfg_,
        &debug_cluster_pts, &debug_filtered_pts,
        pred_x, pred_y, pred_z, has_pred
    );

    // Publish Debug Point Clouds & ROI Markers
    if (publish_debug_cloud_) {
        MarkerVisualizer::publish_point_cloud(pub_filtered_cloud_, debug_filtered_pts, stamp, "base_link");
        publish_roi_markers(stamp, raw_res);
    }
    
    // Publish cyan cluster ONLY when actively flying; clear it immediately when landed/idle
    if (ballistic_tracker_.state == TrackerState::TRACKING && !debug_cluster_pts.empty())
    {
        MarkerVisualizer::publish_point_cloud(pub_cluster_cloud_, debug_cluster_pts, stamp, "base_link");
    }
    else
    {
        std::vector<float3> empty_pts;
        MarkerVisualizer::publish_point_cloud(pub_cluster_cloud_, empty_pts, stamp, "base_link");
    }

    // Diagnostics to console every 1 second
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[ClothTracker] ROI points: %zu | Candidate cluster: %d pts at (%.2f, %.2f, %.2f) | State: %s",
        debug_filtered_pts.size(), raw_res.cluster_points_count, raw_res.cx, raw_res.cy, raw_res.cz,
        ballistic_tracker_.state == TrackerState::TRACKING ? "TRACKING" : "IDLE"
    );

    // Phase 2: Ballistic State Tracking (Filter out stationary walls/ceiling & robot shaking)
    ClothTrackingResult res;
    bool is_actively_tracked = false;
    if (enable_ballistic_filter_) {
        is_actively_tracked = ballistic_tracker_.update(raw_res, stamp.seconds(), res);
    } else {
        res = raw_res;
        is_actively_tracked = raw_res.valid;
    }

    if (is_actively_tracked) {
        RCLCPP_DEBUG(this->get_logger(),
                     "TRACKED CLOTH: points=%d, pos=(%.2f, %.2f, %.2f)",
                     res.cluster_points_count, res.cx, res.cy, res.cz);

        last_cloth_x_ = res.cx;
        last_cloth_y_ = res.cy;
        last_cloth_z_ = res.cz;
        has_cloth_pose_ = true;

        if (ballistic_tracker_.tracked_count <= 1) {
            trajectory_points_.clear(); // Brand new launch: clear previous trajectory line
        }

        // Publish Centroid Point (Strictly real measured point)
        geometry_msgs::msg::PointStamped pt_msg;
        pt_msg.header.stamp = stamp;
        pt_msg.header.frame_id = "base_link";
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
            if (dx >= -0.25) {
                double dy = p_geom.y - prev_pt.y;
                double dz = p_geom.z - prev_pt.z;
                double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (dist >= 0.003) { // 3mm resolution: captures EVERY camera frame!
                    trajectory_points_.emplace_back(now_sec, p_geom);
                }
            }
        }

        if (trajectory_points_.size() > 1000) {
            trajectory_points_.erase(trajectory_points_.begin());
        }
    }

    // 1. Broadcast TF: base_link -> flying_cloth (Every frame while tracked or landed so RViz TF axes NEVER flicker or expire)
    if (has_cloth_pose_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = "flying_cloth";
        tf_msg.transform.translation.x = last_cloth_x_;
        tf_msg.transform.translation.y = last_cloth_y_;
        tf_msg.transform.translation.z = last_cloth_z_;
        tf_msg.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    // 2. Cloth Tip Ball Marker (Red Sphere, diameter 12cm at current or final landing position)
    if (has_cloth_pose_) {
        pub_marker_->publish(MarkerVisualizer::create_cloth_tip_marker(
            stamp, last_cloth_x_, last_cloth_y_, last_cloth_z_));
    }

    // 3. PERSISTENT VISUALIZATION: Strictly true measured flight trajectory (NO fake prediction lines!)
    // When actively tracking: publish every frame.
    // When landed/idle: publish at 10Hz so the line and markers stay solidly on RViz screen without disappearing.
    static double last_static_publish_sec = 0.0;
    bool should_publish_markers = is_actively_tracked || (now_sec - last_static_publish_sec >= 0.10);

    if (should_publish_markers && !trajectory_points_.empty()) {
        last_static_publish_sec = now_sec;

        // 1. Measured Flight Trajectory Line Strip (Bright Gold, strictly connects true camera detections)
        pub_marker_->publish(MarkerVisualizer::create_trajectory_line(stamp, trajectory_points_));

        // 2. True Measured Points Marker (Orange Spheres, exactly where camera detected cloth)
        pub_marker_->publish(MarkerVisualizer::create_measured_points(stamp, trajectory_points_));

        // Clean up obsolete prediction markers so they never linger in RViz
        visualization_msgs::msg::Marker del_apex;
        del_apex.header.stamp = stamp;
        del_apex.header.frame_id = "base_link";
        del_apex.ns = "parabola_apex";
        del_apex.id = 4;
        del_apex.action = visualization_msgs::msg::Marker::DELETE;
        pub_marker_->publish(del_apex);

        visualization_msgs::msg::Marker del_target;
        del_target.header.stamp = stamp;
        del_target.header.frame_id = "base_link";
        del_target.ns = "parabola_landing_target";
        del_target.id = 5;
        del_target.action = visualization_msgs::msg::Marker::DELETE;
        pub_marker_->publish(del_target);

        // 3. Measured Path (True measured points only)
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = "base_link";
        for (const auto& item : trajectory_points_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = stamp;
            pose.header.frame_id = "base_link";
            pose.pose.position = item.second;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        pub_path_->publish(path_msg);
    }
}

void ClothTrackerCudaNode::run_direct_realsense_loop() {
#ifdef HAVE_REALSENSE2
    try {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH, camera_intrin_.width, camera_intrin_.height, RS2_FORMAT_Z16, 60);
        if (enable_color_stream_) {
            cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, color_fps_);
        }

        rs_pipe_ = std::make_unique<rs2::pipeline>();
        rs2::pipeline_profile profile = rs_pipe_->start(cfg);

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

        sensor_msgs::msg::CameraInfo color_info_msg;
        if (enable_color_stream_) {
            try {
                auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
                rs2_intrinsics c_intrin = color_stream.get_intrinsics();
                color_info_msg.width = c_intrin.width;
                color_info_msg.height = c_intrin.height;
                color_info_msg.distortion_model = "plumb_bob";
                color_info_msg.d.resize(5, 0.0);
                for (int i = 0; i < 5; ++i) {
                    color_info_msg.d[i] = c_intrin.coeffs[i];
                }
                color_info_msg.k[0] = c_intrin.fx;
                color_info_msg.k[2] = c_intrin.ppx;
                color_info_msg.k[4] = c_intrin.fy;
                color_info_msg.k[5] = c_intrin.ppy;
                color_info_msg.k[8] = 1.0;

                color_info_msg.r[0] = 1.0;
                color_info_msg.r[4] = 1.0;
                color_info_msg.r[8] = 1.0;

                color_info_msg.p[0] = c_intrin.fx;
                color_info_msg.p[2] = c_intrin.ppx;
                color_info_msg.p[5] = c_intrin.fy;
                color_info_msg.p[6] = c_intrin.ppy;
                color_info_msg.p[10] = 1.0;
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to extract color intrinsics: %s", e.what());
                color_info_msg.width = 640;
                color_info_msg.height = 480;
                color_info_msg.distortion_model = "plumb_bob";
                color_info_msg.d.resize(5, 0.0);
                color_info_msg.k[0] = 600.0;
                color_info_msg.k[2] = 320.0;
                color_info_msg.k[4] = 600.0;
                color_info_msg.k[5] = 240.0;
                color_info_msg.k[8] = 1.0;
                color_info_msg.r[0] = 1.0; color_info_msg.r[4] = 1.0; color_info_msg.r[8] = 1.0;
                color_info_msg.p[0] = 600.0; color_info_msg.p[2] = 320.0;
                color_info_msg.p[5] = 600.0; color_info_msg.p[6] = 240.0;
                color_info_msg.p[10] = 1.0;
            }
        }

        RCLCPP_INFO(this->get_logger(), "RealSense Hardware Pipeline streaming: Depth %dx%d @ 60fps%s",
                    intrin.width, intrin.height,
                    enable_color_stream_ ? ", Color 640x480 (JPEG compressed + CameraInfo)" : "");

        std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        std::vector<uint8_t> compressed_buf;

        while (rclcpp::ok() && is_running_) {
            rs2::frameset frames = rs_pipe_->wait_for_frames(1000);
            rclcpp::Time now = this->get_clock()->now();

            // 1. Process Depth for CUDA cloth tracking (60fps native GPU)
            rs2::depth_frame depth = frames.get_depth_frame();
            if (depth) {
                const uint16_t* depth_ptr = reinterpret_cast<const uint16_t*>(depth.get_data());
                process_and_publish(depth_ptr, now);
            }

            // 2. Ultra-Lightweight JPEG Compressed Color Image Publishing (for rosbag verification)
            if (enable_color_stream_ && pub_compressed_color_) {
                rs2::video_frame color = frames.get_color_frame();
                if (color) {
                    cv::Mat color_mat(cv::Size(color.get_width(), color.get_height()), CV_8UC3,
                                      const_cast<void*>(color.get_data()), cv::Mat::AUTO_STEP);
                    compressed_buf.clear();
                    cv::imencode(".jpg", color_mat, compressed_buf, jpeg_params);

                    sensor_msgs::msg::CompressedImage comp_msg;
                    comp_msg.header.stamp = now;
                    comp_msg.header.frame_id = "camera_color_optical_frame";
                    comp_msg.format = "jpeg";
                    comp_msg.data = std::move(compressed_buf);
                    pub_compressed_color_->publish(comp_msg);

                    if (pub_color_info_) {
                        color_info_msg.header.stamp = now;
                        color_info_msg.header.frame_id = "camera_color_optical_frame";
                        pub_color_info_->publish(color_info_msg);
                    }
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<realsense_dust_cloth::ClothTrackerCudaNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
