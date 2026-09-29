#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>

namespace realsense_dust_cloth {

namespace {
geometry_msgs::msg::Point interpolate_catmull_rom(
    const geometry_msgs::msg::Point& p0,
    const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& p2,
    const geometry_msgs::msg::Point& p3,
    double t)
{
    double t2 = t * t;
    double t3 = t2 * t;

    geometry_msgs::msg::Point pt;
    pt.x = 0.5 * ((2.0 * p1.x) +
                  (-p0.x + p2.x) * t +
                  (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2 +
                  (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3);
    pt.y = 0.5 * ((2.0 * p1.y) +
                  (-p0.y + p2.y) * t +
                  (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2 +
                  (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3);
    pt.z = 0.5 * ((2.0 * p1.z) +
                  (-p0.z + p2.z) * t +
                  (2.0 * p0.z - 5.0 * p1.z + 4.0 * p2.z - p3.z) * t2 +
                  (-p0.z + 3.0 * p1.z - 3.0 * p2.z + p3.z) * t3);
    return pt;
}

// ParabolaFit is declared in cloth_tracker_cuda_node.hpp

ParabolaFit compute_most_probable_parabola(
    const std::vector<std::pair<double, geometry_msgs::msg::Point>>& raw_pts,
    double target_landing_z = 0.0)
{
    ParabolaFit res;
    if (raw_pts.size() < 5) return res;

    double sx4 = 0.0, sx3 = 0.0, sx2 = 0.0, sx1 = 0.0, s_n = 0.0;
    double sx2z = 0.0, sxz = 0.0, sz = 0.0;
    double sxy = 0.0, sy = 0.0;
    double z_mean_acc = 0.0;

    for (const auto& item : raw_pts) {
        double x = item.second.x;
        double y = item.second.y;
        double z = item.second.z;

        double x2 = x * x;
        double x3 = x2 * x;
        double x4 = x3 * x;

        sx4 += x4;
        sx3 += x3;
        sx2 += x2;
        sx1 += x;
        s_n += 1.0;

        sx2z += x2 * z;
        sxz += x * z;
        sz += z;

        sxy += x * y;
        sy += y;
        z_mean_acc += z;
    }

    // Solve 3x3 quadratic system for Z(X) = a*X^2 + b*X + c using Cramer's Rule
    double D = sx4 * (sx2 * s_n - sx1 * sx1) - sx3 * (sx3 * s_n - sx1 * sx2) + sx2 * (sx3 * sx1 - sx2 * sx2);
    if (std::abs(D) < 1e-9) return res;

    double Da = sx2z * (sx2 * s_n - sx1 * sx1) - sx3 * (sxz * s_n - sx1 * sz) + sx2 * (sxz * sx1 - sx2 * sz);
    double Db = sx4 * (sxz * s_n - sx1 * sz) - sx2z * (sx3 * s_n - sx1 * sx2) + sx2 * (sx3 * sz - sxz * sx2);
    double Dc = sx4 * (sx2 * sz - sxz * sx1) - sx3 * (sx3 * sz - sxz * sx2) + sx2z * (sx3 * sx1 - sx2 * sx2);

    double a = Da / D;
    double b = Db / D;
    double c = Dc / D;

    // Check concave downward (a < 0) for ballistic arc
    if (a >= 0.0) return res;

    // Solve 2x2 linear system for Y(X) = d*X + e
    double D_line = sx2 * s_n - sx1 * sx1;
    double d = (std::abs(D_line) > 1e-9) ? ((sxy * s_n - sx1 * sy) / D_line) : 0.0;
    double e = (std::abs(D_line) > 1e-9) ? ((sx2 * sy - sx1 * sxy) / D_line) : 0.0;

    res.a = a; res.b = b; res.c = c;
    res.d = d; res.e = e;
    res.x_start = raw_pts.front().second.x;

    // Apex (最高到達点)
    res.x_apex = -b / (2.0 * a);
    res.z_apex = a * res.x_apex * res.x_apex + b * res.x_apex + c;

    // Landing calculation (solve a*X^2 + b*X + (c - target_landing_z) = 0)
    double disc = b * b - 4.0 * a * (c - target_landing_z);
    if (disc >= 0.0) {
        double r1 = (-b + std::sqrt(disc)) / (2.0 * a);
        double r2 = (-b - std::sqrt(disc)) / (2.0 * a);
        res.x_land = std::max(r1, r2);
    } else {
        res.x_land = raw_pts.back().second.x + 0.3;
    }
    res.y_land = d * res.x_land + e;
    res.z_land = target_landing_z;

    // R^2 Confidence Score
    double z_mean = z_mean_acc / s_n;
    double ss_tot = 0.0, ss_res = 0.0;
    for (const auto& item : raw_pts) {
        double x = item.second.x;
        double z = item.second.z;
        double z_pred = a * x * x + b * x + c;
        ss_tot += (z - z_mean) * (z - z_mean);
        ss_res += (z - z_pred) * (z - z_pred);
    }
    res.r2 = (ss_tot > 1e-6) ? std::clamp(1.0 - ss_res / ss_tot, 0.0, 1.0) : 0.0;

    // Generate 150 smooth points along the mathematical parabola from launch to landing!
    double x_min = std::min(res.x_start, 0.35);
    double x_max = std::max(res.x_land, raw_pts.back().second.x);
    x_max = std::min(x_max, 6.0);
    int num_steps = 150;
    double dx = (x_max - x_min) / static_cast<double>(num_steps);

    for (int i = 0; i <= num_steps; ++i) {
        double px = x_min + i * dx;
        double py = d * px + e;
        double pz = a * px * px + b * px + c;
        if (pz < 0.0) pz = 0.0;
        geometry_msgs::msg::Point pt;
        pt.x = px;
        pt.y = py;
        pt.z = pz;
        res.curve_pts.push_back(pt);
    }

    res.valid = true;
    return res;
}
} // namespace



ClothTrackerCudaNode::ClothTrackerCudaNode(const rclcpp::NodeOptions& options)
    : Node("cloth_tracker_cuda_node", options) {
    declare_parameters();
    load_parameters();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Broadcast RealSense fixed pose (base_link -> camera_link -> camera_depth_optical_frame)
    publish_static_tf();

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

    RCLCPP_INFO(this->get_logger(), "RealSense Cloth CUDA Tracker Node initialized successfully with Static TF (base_link -> camera_link).");
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
    this->declare_parameter("max_z_m", 2.20);

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

        // Check if detected cluster is within launch ROI (initial eject zone)
        if (raw.cx >= launch_min_x && raw.cx <= launch_max_x &&
            raw.cy >= launch_min_y && raw.cy <= launch_max_y &&
            raw.cz >= launch_min_z && raw.cz <= launch_max_z)
        {
            state = TrackerState::TRACKING;
            x = raw.cx;
            y = raw.cy;
            z = raw.cz;
            vx = 1.5;
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
    } else { // TRACKING
        double dt = stamp_sec - last_stamp;
        if (dt <= 0.0 || dt > 0.35) {
            state = TrackerState::LANDED;
            return false;
        }

        // Ballistic prediction with aerodynamic drag deceleration
        double drag_factor = std::max(0.50, 1.0 - 1.2 * dt);
        double pred_x = x + vx * dt * drag_factor;
        double pred_y = y + vy * dt;
        double pred_z = z + vz * dt - 0.5 * 9.8 * dt * dt; // True gravity acceleration

        bool matched = false;
        if (raw.valid) {
            double dx_pred = raw.cx - pred_x;
            double dy_pred = raw.cy - pred_y;
            double dz_pred = raw.cz - pred_z;
            double dist_pred = std::sqrt(dx_pred * dx_pred + dy_pred * dy_pred + dz_pred * dz_pred);

            double dx_prev = raw.cx - x;
            double dy_prev = raw.cy - y;
            double dz_prev = raw.cz - z;
            double dist_prev = std::sqrt(dx_prev * dx_prev + dy_prev * dy_prev + dz_prev * dz_prev);

            // Accept cloth even with severe aerodynamic drag, tumbling, or sudden drop
            if (dist_pred <= 1.20 || dist_prev <= 1.20) {
                matched = true;
                double dt_safe = std::clamp(dt, 0.005, 0.100);

                double new_vx = (raw.cx - x) / dt_safe;
                double new_vy = (raw.cy - y) / dt_safe;
                double new_vz = (raw.cz - z) / dt_safe;

                vx = 0.5 * vx + 0.5 * new_vx;
                vy = 0.5 * vy + 0.5 * new_vy;
                vz = 0.5 * vz + 0.5 * new_vz;

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

                // Stop tracking immediately when cloth descends to floor or forward motion stops near ground
                if (z <= 0.15 || (z <= 0.28 && vx <= 0.4) || x >= 5.5) {
                    state = TrackerState::LANDED;
                }
                return true;
            }
        }

        if (!matched) {
            lost_count++;
            if (lost_count > 12 || pred_z <= 0.08 || pred_x >= 5.20) {
                state = TrackerState::LANDED;
                return false;
            }

            // Maintain continuous search gate
            x = pred_x;
            y = pred_y;
            z = pred_z;
            last_stamp = stamp_sec;
            return false;
        }
    }
    return false;
}


void ClothTrackerCudaNode::publish_static_tf() {
    if (!static_tf_broadcaster_) return;

    rclcpp::Time now = this->get_clock()->now();

    // 1. base_link -> camera_link
    geometry_msgs::msg::TransformStamped tf_cam;
    tf_cam.header.stamp = now;
    tf_cam.header.frame_id = "base_link";
    tf_cam.child_frame_id = "camera_link";
    tf_cam.transform.translation.x = camera_pose_.x_offset_m;
    tf_cam.transform.translation.y = 0.0;
    tf_cam.transform.translation.z = camera_pose_.height_offset_m;

    tf2::Quaternion q;
    double roll_rad = camera_pose_.roll_deg * (M_PI / 180.0);
    double pitch_rad = camera_pose_.pitch_deg * (M_PI / 180.0);
    q.setRPY(roll_rad, pitch_rad, 0.0);
    tf_cam.transform.rotation.x = q.x();
    tf_cam.transform.rotation.y = q.y();
    tf_cam.transform.rotation.z = q.z();
    tf_cam.transform.rotation.w = q.w();

    // 2. camera_link -> camera_depth_optical_frame
    geometry_msgs::msg::TransformStamped tf_depth_opt;
    tf_depth_opt.header.stamp = now;
    tf_depth_opt.header.frame_id = "camera_link";
    tf_depth_opt.child_frame_id = "camera_depth_optical_frame";
    tf_depth_opt.transform.translation.x = 0.0;
    tf_depth_opt.transform.translation.y = 0.0;
    tf_depth_opt.transform.translation.z = 0.0;
    tf2::Quaternion q_depth;
    q_depth.setRPY(-M_PI / 2.0, 0.0, -M_PI / 2.0);
    tf_depth_opt.transform.rotation.x = q_depth.x();
    tf_depth_opt.transform.rotation.y = q_depth.y();
    tf_depth_opt.transform.rotation.z = q_depth.z();
    tf_depth_opt.transform.rotation.w = q_depth.w();

    // 3. camera_link -> camera_color_optical_frame
    geometry_msgs::msg::TransformStamped tf_color_opt = tf_depth_opt;
    tf_color_opt.child_frame_id = "camera_color_optical_frame";

    static_tf_broadcaster_->sendTransform({tf_cam, tf_depth_opt, tf_color_opt});
}

void ClothTrackerCudaNode::publish_roi_markers(const rclcpp::Time& stamp, const ClothTrackingResult& raw_res) {
    if (!pub_roi_markers_) return;

    visualization_msgs::msg::MarkerArray markers;

    // 1. PassThrough ROI Box (Cyan Wireframe Cube)
    visualization_msgs::msg::Marker roi_marker;
    roi_marker.header.stamp = stamp;
    roi_marker.header.frame_id = "base_link";
    roi_marker.ns = "roi_passthrough";
    roi_marker.id = 0;
    roi_marker.type = visualization_msgs::msg::Marker::CUBE;
    roi_marker.action = visualization_msgs::msg::Marker::ADD;
    roi_marker.pose.position.x = 0.5 * (filter_cfg_.min_x_m + filter_cfg_.max_x_m);
    roi_marker.pose.position.y = 0.5 * (filter_cfg_.min_y_m + filter_cfg_.max_y_m);
    roi_marker.pose.position.z = 0.5 * (filter_cfg_.min_z_m + filter_cfg_.max_z_m);
    roi_marker.pose.orientation.w = 1.0;
    roi_marker.scale.x = std::max(0.01f, filter_cfg_.max_x_m - filter_cfg_.min_x_m);
    roi_marker.scale.y = std::max(0.01f, filter_cfg_.max_y_m - filter_cfg_.min_y_m);
    roi_marker.scale.z = std::max(0.01f, filter_cfg_.max_z_m - filter_cfg_.min_z_m);
    roi_marker.color.r = 0.0f;
    roi_marker.color.g = 0.85f;
    roi_marker.color.b = 1.0f;
    roi_marker.color.a = 0.10f;
    markers.markers.push_back(roi_marker);

    // 2. Launch Gate Box (Yellow Translucent Cube)
    if (enable_ballistic_filter_) {
        visualization_msgs::msg::Marker launch_marker;
        launch_marker.header.stamp = stamp;
        launch_marker.header.frame_id = "base_link";
        launch_marker.ns = "launch_gate";
        launch_marker.id = 1;
        launch_marker.type = visualization_msgs::msg::Marker::CUBE;
        launch_marker.action = visualization_msgs::msg::Marker::ADD;
        launch_marker.pose.position.x = 0.5 * (ballistic_tracker_.launch_min_x + ballistic_tracker_.launch_max_x);
        launch_marker.pose.position.y = 0.5 * (ballistic_tracker_.launch_min_y + ballistic_tracker_.launch_max_y);
        launch_marker.pose.position.z = 0.5 * (ballistic_tracker_.launch_min_z + ballistic_tracker_.launch_max_z);
        launch_marker.pose.orientation.w = 1.0;
        launch_marker.scale.x = std::max(0.01, ballistic_tracker_.launch_max_x - ballistic_tracker_.launch_min_x);
        launch_marker.scale.y = std::max(0.01, ballistic_tracker_.launch_max_y - ballistic_tracker_.launch_min_y);
        launch_marker.scale.z = std::max(0.01, ballistic_tracker_.launch_max_z - ballistic_tracker_.launch_min_z);
        launch_marker.color.r = 1.0f;
        launch_marker.color.g = 0.85f;
        launch_marker.color.b = 0.0f;
        launch_marker.color.a = 0.20f;
        markers.markers.push_back(launch_marker);
    }

    // 3. Real-time HUD Dashboard (High-visibility 3D Text positioned clearly above scene)
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.stamp = stamp;
    text_marker.header.frame_id = "base_link";
    text_marker.ns = "tracker_status_hud";
    text_marker.id = 2;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = 1.5;
    text_marker.pose.position.y = -0.9;
    text_marker.pose.position.z = 1.8;
    text_marker.scale.z = 0.16; // Clear large text

    char status_str[512];
    if (ballistic_tracker_.state == TrackerState::TRACKING) {
        text_marker.color.r = 0.2f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 0.3f;
        text_marker.color.a = 1.0f;
        double speed = std::sqrt(ballistic_tracker_.vx * ballistic_tracker_.vx +
                                 ballistic_tracker_.vy * ballistic_tracker_.vy +
                                 ballistic_tracker_.vz * ballistic_tracker_.vz);
        snprintf(status_str, sizeof(status_str),
                 "● [TRACKING ACTIVE]\nCloth: (X: %.2fm, Y: %.2fm, Z: %.2fm)\nSpeed: %.2f m/s (Vx: %.2f, Vz: %.2f)\nPoints: %d pts",
                 ballistic_tracker_.x, ballistic_tracker_.y, ballistic_tracker_.z,
                 speed, ballistic_tracker_.vx, ballistic_tracker_.vz, raw_res.cluster_points_count);
    } else {
        text_marker.color.r = 1.0f;
        text_marker.color.g = 0.9f;
        text_marker.color.b = 0.2f;
        text_marker.color.a = 0.95f;
        if (raw_res.valid) {
            snprintf(status_str, sizeof(status_str),
                     "○ [IDLE: Candidate Detected]\nPos: (X: %.2fm, Y: %.2fm, Z: %.2fm)\nCluster: %d pts\n(Waiting for launch velocity)",
                     raw_res.cx, raw_res.cy, raw_res.cz, raw_res.cluster_points_count);
        } else {
            snprintf(status_str, sizeof(status_str),
                     "○ [IDLE: Waiting for Throw]\nLaunch Area: X: %.2f~%.2fm, Z: %.2f~%.2fm\nCamera: Pitch %.1f deg, Height %.2fm",
                     ballistic_tracker_.launch_min_x, ballistic_tracker_.launch_max_x,
                     ballistic_tracker_.launch_min_z, ballistic_tracker_.launch_max_z,
                     camera_pose_.pitch_deg, camera_pose_.height_offset_m);
        }
    }
    text_marker.text = status_str;
    markers.markers.push_back(text_marker);

    // 4. Floor Distance Markers (1.0m, 2.0m, 3.0m, 3.5m)
    for (double dist = 1.0; dist <= 3.5; dist += 1.0) {
        visualization_msgs::msg::Marker dist_marker;
        dist_marker.header.stamp = stamp;
        dist_marker.header.frame_id = "base_link";
        dist_marker.ns = "floor_distance_marks";
        dist_marker.id = static_cast<int>(dist * 10);
        dist_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        dist_marker.action = visualization_msgs::msg::Marker::ADD;
        dist_marker.pose.position.x = dist;
        dist_marker.pose.position.y = 0.65;
        dist_marker.pose.position.z = 0.05;
        dist_marker.scale.z = 0.12;
        dist_marker.color.r = 0.7f;
        dist_marker.color.g = 0.8f;
        dist_marker.color.b = 1.0f;
        dist_marker.color.a = 0.85f;
        char dist_buf[32];
        snprintf(dist_buf, sizeof(dist_buf), "%.1fm", dist);
        dist_marker.text = dist_buf;
        markers.markers.push_back(dist_marker);
    }

    // 5. Landing point text marker
    if (!trajectory_points_.empty()) {
        const auto& last_pt = trajectory_points_.back().second;
        visualization_msgs::msg::Marker land_marker;
        land_marker.header.stamp = stamp;
        land_marker.header.frame_id = "base_link";
        land_marker.ns = "landing_point_text";
        land_marker.id = 99;
        land_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        land_marker.action = visualization_msgs::msg::Marker::ADD;
        land_marker.pose.position.x = last_pt.x;
        land_marker.pose.position.y = last_pt.y;
        land_marker.pose.position.z = last_pt.z + 0.18;
        land_marker.scale.z = 0.13;
        land_marker.color.r = 1.0f;
        land_marker.color.g = 0.3f;
        land_marker.color.b = 0.2f;
        land_marker.color.a = 1.0f;
        char land_buf[64];
        snprintf(land_buf, sizeof(land_buf), "🎯 Landing: %.2fm", last_pt.x);
        land_marker.text = land_buf;
        markers.markers.push_back(land_marker);
    }

    pub_roi_markers_->publish(markers);
}

static void publish_point_cloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
    const std::vector<float3>& pts,
    const rclcpp::Time& stamp,
    const std::string& frame_id)
{
    if (!pub || pts.empty()) return;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(pts.size());
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(pts.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const auto& p : pts) {
        *iter_x = p.x;
        *iter_y = p.y;
        *iter_z = p.z;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    pub->publish(cloud_msg);
}

void ClothTrackerCudaNode::process_and_publish(const uint16_t* depth_ptr, const rclcpp::Time& stamp) {
    double now_sec = stamp.seconds();
    if (last_frame_stamp_sec_ > 0.0 && now_sec < last_frame_stamp_sec_ - 0.5) {
        // Rosbag looped back to the beginning: clear all trajectory history and reset tracker
        trajectory_points_.clear();
        ballistic_tracker_.reset();
        bg_frames_collected_ = 0;
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
        publish_point_cloud(pub_filtered_cloud_, debug_filtered_pts, stamp, "base_link");
        publish_roi_markers(stamp, raw_res);
    }
    
    // Publish cyan cluster ONLY when actively flying; clear it immediately when landed/idle
    if (ballistic_tracker_.state == TrackerState::TRACKING && !debug_cluster_pts.empty())
    {
        publish_point_cloud(pub_cluster_cloud_, debug_cluster_pts, stamp, "base_link");
    }
    else
    {
        std::vector<float3> empty_pts;
        publish_point_cloud(pub_cluster_cloud_, empty_pts, stamp, "base_link");
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

        // 1. Broadcast TF: base_link -> flying_cloth
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = "flying_cloth";
        tf_msg.transform.translation.x = res.cx;
        tf_msg.transform.translation.y = res.cy;
        tf_msg.transform.translation.z = res.cz;
        tf_msg.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf_msg);

        // 2. Publish Centroid Point (Strictly TRUE measured point only, no artificial interpolation!)
        if (ballistic_tracker_.tracked_count <= 1) {
            trajectory_points_.clear(); // Brand new launch: clear previous trajectory line
            cached_parabola_ = ParabolaFit{};
        }

        geometry_msgs::msg::PointStamped pt_msg;
        pt_msg.header.stamp = stamp;
        pt_msg.header.frame_id = "base_link";
        pt_msg.point.x = res.cx;
        pt_msg.point.y = res.cy;
        pt_msg.point.z = res.cz;
        pub_point_->publish(pt_msg);

        // 3. Trajectory History & RViz Markers (Strictly true measured forward-moving points)
        geometry_msgs::msg::Point p_geom;
        p_geom.x = res.cx;
        p_geom.y = res.cy;
        p_geom.z = res.cz;

        // Only append if it's the first point or advancing forward (prevents backward warp/loops)
        if (trajectory_points_.empty()) {
            trajectory_points_.emplace_back(now_sec, p_geom);
        } else {
            const auto& prev_pt = trajectory_points_.back().second;
            double dx = p_geom.x - prev_pt.x;
            // Cloth must fly forward (+X) or have tiny motion
            if (dx >= -0.05) {
                double dy = p_geom.y - prev_pt.y;
                double dz = p_geom.z - prev_pt.z;
                double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (dist >= 0.003) { // 3mm resolution: captures EVERY camera frame!
                    trajectory_points_.emplace_back(now_sec, p_geom);
                }
            }
        }

        if (trajectory_points_.size() > 500) {
            trajectory_points_.erase(trajectory_points_.begin());
        }

        // Compute Mathematical Most Probable Ballistic Parabola (RANSAC / Least Squares)
        ParabolaFit parabola = compute_most_probable_parabola(trajectory_points_, 0.0);
        if (parabola.valid) {
            cached_parabola_ = parabola;
        }

        // Flying Cloth Tip Ball (Red Sphere, diameter 10cm at current true location)
        visualization_msgs::msg::Marker ball_marker;
        ball_marker.header.stamp = stamp;
        ball_marker.header.frame_id = "base_link";
        ball_marker.ns = "cloth_tip_ball";
        ball_marker.id = 1;
        ball_marker.type = visualization_msgs::msg::Marker::SPHERE;
        ball_marker.action = visualization_msgs::msg::Marker::ADD;
        ball_marker.pose.position = p_geom;
        ball_marker.pose.orientation.w = 1.0;
        ball_marker.scale.x = 0.10;
        ball_marker.scale.y = 0.10;
        ball_marker.scale.z = 0.10;
        ball_marker.color.r = 1.0f;
        ball_marker.color.g = 0.15f;
        ball_marker.color.b = 0.15f;
        ball_marker.color.a = 0.95f;
        pub_marker_->publish(ball_marker);
    } else {
        // When cloth has landed or idle, remove flying red ball marker
        visualization_msgs::msg::Marker del_ball;
        del_ball.header.stamp = stamp;
        del_ball.header.frame_id = "base_link";
        del_ball.ns = "cloth_tip_ball";
        del_ball.id = 1;
        del_ball.action = visualization_msgs::msg::Marker::DELETE;
        pub_marker_->publish(del_ball);
    }

    // 4. PERSISTENT VISUALIZATION:
    // When actively tracking: publish every frame for silky-smooth motion.
    // When landed/idle: throttle to 1Hz (RViz retains markers automatically, completely eliminates lag!)
    static double last_static_publish_sec = 0.0;
    bool should_publish_markers = is_actively_tracked;
    if (!is_actively_tracked && !trajectory_points_.empty()) {
        if (now_sec - last_static_publish_sec >= 1.0) {
            should_publish_markers = true;
            last_static_publish_sec = now_sec;
        }
    }

    if (should_publish_markers && !trajectory_points_.empty()) {
        // 1. Mathematical Parabola Line Strip (Bright Gold, 100% noise-free smooth curve)
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.stamp = stamp;
        line_marker.header.frame_id = "base_link";
        line_marker.ns = "cloth_cuda_trajectory";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.038;
        line_marker.color.r = 1.0f;
        line_marker.color.g = 0.85f;
        line_marker.color.b = 0.0f;
        line_marker.color.a = 0.95f;

        if (cached_parabola_.valid && !cached_parabola_.curve_pts.empty()) {
            line_marker.points = cached_parabola_.curve_pts;
        } else {
            for (const auto& item : trajectory_points_) {
                line_marker.points.push_back(item.second);
            }
        }
        pub_marker_->publish(line_marker);

        // 2. True Measured Points Marker (Orange Spheres, exactly where camera detected cloth)
        visualization_msgs::msg::Marker raw_points_marker;
        raw_points_marker.header.stamp = stamp;
        raw_points_marker.header.frame_id = "base_link";
        raw_points_marker.ns = "cloth_measured_points";
        raw_points_marker.id = 2;
        raw_points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        raw_points_marker.action = visualization_msgs::msg::Marker::ADD;
        raw_points_marker.scale.x = 0.045;
        raw_points_marker.scale.y = 0.045;
        raw_points_marker.scale.z = 0.045;
        raw_points_marker.color.r = 1.0f;
        raw_points_marker.color.g = 0.40f;
        raw_points_marker.color.b = 0.0f;
        raw_points_marker.color.a = 0.95f;
        for (const auto& item : trajectory_points_) {
            raw_points_marker.points.push_back(item.second);
        }
        pub_marker_->publish(raw_points_marker);

        // 3. TF Frame Coordinate Triad (Spaced every 4 points to avoid GPU overloading)
        visualization_msgs::msg::Marker axes_marker;
        axes_marker.header.stamp = stamp;
        axes_marker.header.frame_id = "base_link";
        axes_marker.ns = "cloth_trajectory_tf_axes";
        axes_marker.id = 3;
        axes_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        axes_marker.action = visualization_msgs::msg::Marker::ADD;
        axes_marker.scale.x = 0.007; // 7mm line thickness

        std_msgs::msg::ColorRGBA col_r, col_g, col_b;
        col_r.r = 1.0f; col_r.g = 0.1f; col_r.b = 0.1f; col_r.a = 0.95f;
        col_g.r = 0.1f; col_g.g = 1.0f; col_g.b = 0.1f; col_g.a = 0.95f;
        col_b.r = 0.1f; col_b.g = 0.4f; col_b.b = 1.0f; col_b.a = 0.95f;

        const double axis_len = 0.07; // 7cm axis length
        for (size_t i = 0; i < trajectory_points_.size(); i += 4) {
            const auto& orig = trajectory_points_[i].second;

            // X (Red)
            geometry_msgs::msg::Point px = orig;
            px.x += axis_len;
            axes_marker.points.push_back(orig); axes_marker.colors.push_back(col_r);
            axes_marker.points.push_back(px);   axes_marker.colors.push_back(col_r);

            // Y (Green)
            geometry_msgs::msg::Point py = orig;
            py.y += axis_len;
            axes_marker.points.push_back(orig); axes_marker.colors.push_back(col_g);
            axes_marker.points.push_back(py);   axes_marker.colors.push_back(col_g);

            // Z (Blue)
            geometry_msgs::msg::Point pz = orig;
            pz.z += axis_len;
            axes_marker.points.push_back(orig); axes_marker.colors.push_back(col_b);
            axes_marker.points.push_back(pz);   axes_marker.colors.push_back(col_b);
        }
        pub_marker_->publish(axes_marker);

        // 4. Parabolic Apex & Predicted Landing Markers
        if (cached_parabola_.valid) {
            // Apex (Purple Sphere)
            visualization_msgs::msg::Marker apex_marker;
            apex_marker.header.stamp = stamp;
            apex_marker.header.frame_id = "base_link";
            apex_marker.ns = "parabola_apex";
            apex_marker.id = 4;
            apex_marker.type = visualization_msgs::msg::Marker::SPHERE;
            apex_marker.action = visualization_msgs::msg::Marker::ADD;
            apex_marker.pose.position.x = cached_parabola_.x_apex;
            apex_marker.pose.position.y = cached_parabola_.d * cached_parabola_.x_apex + cached_parabola_.e;
            apex_marker.pose.position.z = cached_parabola_.z_apex;
            apex_marker.pose.orientation.w = 1.0;
            apex_marker.scale.x = 0.12; apex_marker.scale.y = 0.12; apex_marker.scale.z = 0.12;
            apex_marker.color.r = 0.8f; apex_marker.color.g = 0.2f; apex_marker.color.b = 1.0f; apex_marker.color.a = 0.95f;
            pub_marker_->publish(apex_marker);

            // Landing Target Cylinder / Ring (Emerald Green)
            visualization_msgs::msg::Marker target_marker;
            target_marker.header.stamp = stamp;
            target_marker.header.frame_id = "base_link";
            target_marker.ns = "parabola_landing_target";
            target_marker.id = 5;
            target_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            target_marker.action = visualization_msgs::msg::Marker::ADD;
            target_marker.pose.position.x = cached_parabola_.x_land;
            target_marker.pose.position.y = cached_parabola_.y_land;
            target_marker.pose.position.z = 0.02;
            target_marker.pose.orientation.w = 1.0;
            target_marker.scale.x = 0.25; target_marker.scale.y = 0.25; target_marker.scale.z = 0.04;
            target_marker.color.r = 0.0f; target_marker.color.g = 1.0f; target_marker.color.b = 0.4f; target_marker.color.a = 0.85f;
            pub_marker_->publish(target_marker);
        }

        // 5. Measured Path (True measured points only)
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

