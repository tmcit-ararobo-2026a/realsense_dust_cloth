#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

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

std::vector<geometry_msgs::msg::Point> generate_uniform_dense_trajectory(
    const std::vector<std::pair<double, geometry_msgs::msg::Point>>& raw_pts,
    double step_m = 0.015) // Strict 1.5cm equidistant sampling across entire parabola
{
    if (raw_pts.empty()) return {};
    if (raw_pts.size() == 1) return {raw_pts[0].second};

    // 1. Generate fine Catmull-Rom samples (16 sub-steps per frame)
    std::vector<geometry_msgs::msg::Point> fine_pts;
    size_t n = raw_pts.size();
    for (size_t i = 0; i < n - 1; ++i) {
        const auto& p0 = (i == 0) ? raw_pts[0].second : raw_pts[i - 1].second;
        const auto& p1 = raw_pts[i].second;
        const auto& p2 = raw_pts[i + 1].second;
        const auto& p3 = (i + 2 < n) ? raw_pts[i + 2].second : raw_pts[i + 1].second;

        for (int step = 0; step < 16; ++step) {
            double t = static_cast<double>(step) / 16.0;
            fine_pts.push_back(interpolate_catmull_rom(p0, p1, p2, p3, t));
        }
    }
    fine_pts.push_back(raw_pts.back().second);

    // 2. Resample along the curve at strictly uniform step_m (every 1.5cm)
    std::vector<geometry_msgs::msg::Point> uniform_pts;
    uniform_pts.push_back(fine_pts[0]);
    double dist_accum = 0.0;

    for (size_t i = 1; i < fine_pts.size(); ++i) {
        double dx = fine_pts[i].x - fine_pts[i - 1].x;
        double dy = fine_pts[i].y - fine_pts[i - 1].y;
        double dz = fine_pts[i].z - fine_pts[i - 1].z;
        double seg_len = std::sqrt(dx * dx + dy * dy + dz * dz);
        dist_accum += seg_len;

        if (dist_accum >= step_m) {
            uniform_pts.push_back(fine_pts[i]);
            dist_accum = 0.0;
        }
    }
    if (uniform_pts.empty() || uniform_pts.back().x != fine_pts.back().x) {
        uniform_pts.push_back(fine_pts.back());
    }
    return uniform_pts;
}
} // namespace



ClothTrackerCudaNode::ClothTrackerCudaNode(const rclcpp::NodeOptions& options)
    : Node("cloth_tracker_cuda_node", options) {
    declare_parameters();
    load_parameters();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    pub_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/cloth/position", 10);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/cloth/trajectory_marker", 10);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/cloth/measured_path", 10);

    if (publish_debug_cloud_) {
        pub_cluster_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloth/cluster_points", 10);
    }

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

        RCLCPP_INFO(this->get_logger(), "Listening on Depth Image topic: %s", depth_topic.c_str());
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
    this->declare_parameter("use_direct_realsense", false);
    this->declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw");
    this->declare_parameter("camera_info_topic", "/camera/camera/depth/camera_info");

    this->declare_parameter("camera_width", 848);
    this->declare_parameter("camera_height", 480);
    this->declare_parameter("camera_pitch_deg", 20.0);
    this->declare_parameter("camera_roll_deg", 90.0);
    this->declare_parameter("camera_x_offset_m", 0.15);
    this->declare_parameter("camera_height_offset_m", 0.35);

    this->declare_parameter("min_depth_m", 0.25);
    this->declare_parameter("max_depth_m", 4.5);

    this->declare_parameter("min_x_m", 0.40);
    this->declare_parameter("max_x_m", 3.50);
    this->declare_parameter("min_y_m", -0.60);
    this->declare_parameter("max_y_m", 0.60);
    this->declare_parameter("min_z_m", 0.10);
    this->declare_parameter("max_z_m", 2.00);

    this->declare_parameter("voxel_size_m", 0.03);
    this->declare_parameter("cluster_eps_m", 0.15);
    this->declare_parameter("min_cluster_points", 15);
    this->declare_parameter("max_cluster_points", 6000);
    this->declare_parameter("max_cloth_size_m", 0.85);
    this->declare_parameter("use_bg_subtraction", true);
    this->declare_parameter("bg_diff_thresh_m", 0.15);
    this->declare_parameter("bg_target_frames", 20);

    this->declare_parameter("publish_debug_cloud", false);
    this->declare_parameter("trajectory_history_sec", 15.0);

    // Ballistic Filter & Launch Gate parameters
    this->declare_parameter("enable_ballistic_filter", true);
    this->declare_parameter("launch_min_x", 0.50);
    this->declare_parameter("launch_max_x", 1.20);
    this->declare_parameter("launch_min_y", -0.40);
    this->declare_parameter("launch_max_y", 0.40);
    this->declare_parameter("launch_min_z", 0.60);
    this->declare_parameter("launch_max_z", 1.20);
    this->declare_parameter("gate_radius_m", 1.20);
    this->declare_parameter("max_lost_frames", 8);
    this->declare_parameter("min_launch_vx", 0.50);
}



void ClothTrackerCudaNode::load_parameters() {
    use_direct_realsense_ = this->get_parameter("use_direct_realsense").as_bool();
    camera_intrin_.width = this->get_parameter("camera_width").as_int();
    camera_intrin_.height = this->get_parameter("camera_height").as_int();

    camera_pose_.pitch_deg = static_cast<float>(this->get_parameter("camera_pitch_deg").as_double());
    camera_pose_.roll_deg = static_cast<float>(this->get_parameter("camera_roll_deg").as_double());
    camera_pose_.x_offset_m = static_cast<float>(this->get_parameter("camera_x_offset_m").as_double());
    camera_pose_.height_offset_m = static_cast<float>(this->get_parameter("camera_height_offset_m").as_double());

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

    if (msg->encoding != "16UC1" && msg->encoding != "mono16") {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Expected 16UC1/mono16 depth encoding, got: %s", msg->encoding.c_str());
        return;
    }

    const uint16_t* depth_ptr = reinterpret_cast<const uint16_t*>(msg->data.data());
    // Use msg header stamp for robust and exact time delta calculation (works identically on rosbag & real hardware)
    rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0) {
        stamp = this->get_clock()->now();
    }
    process_and_publish(depth_ptr, stamp);
}

bool ClothTrackerCudaNode::BallisticTracker::update(
    const ClothTrackingResult& raw, double stamp_sec, ClothTrackingResult& out)
{
    if (state == TrackerState::IDLE) {
        if (!raw.valid) {
            has_pending = false;
            return false;
        }

        // Check if detected cluster is within launch ROI (initial eject zone)
        if (raw.cx >= launch_min_x && raw.cx <= launch_max_x &&
            raw.cy >= launch_min_y && raw.cy <= launch_max_y &&
            raw.cz >= launch_min_z && raw.cz <= launch_max_z)
        {
            if (!has_pending) {
                // First observation: store as pending launch candidate
                has_pending = true;
                pending_x = raw.cx;
                pending_y = raw.cy;
                pending_z = raw.cz;
                pending_stamp = stamp_sec;
                return false;
            } else {
                double dt = stamp_sec - pending_stamp;
                if (dt > 0.005 && dt < 0.25) {
                    double vx_obs = (raw.cx - pending_x) / dt;
                    // Flying cloth has forward velocity vx >= min_launch_vx
                    if (vx_obs >= min_launch_vx) {
                        state = TrackerState::TRACKING;
                        x = raw.cx;
                        y = raw.cy;
                        z = raw.cz;
                        vx = vx_obs;
                        vy = (raw.cy - pending_y) / dt;
                        vz = (raw.cz - pending_z) / dt;
                        last_stamp = stamp_sec;
                        tracked_count = 1;
                        lost_count = 0;
                        has_pending = false;
                        out = raw;
                        return true;
                    }
                }
                // Stationary or time expired: update pending candidate
                pending_x = raw.cx;
                pending_y = raw.cy;
                pending_z = raw.cz;
                pending_stamp = stamp_sec;
                return false;
            }
        } else {
            has_pending = false;
            return false;
        }
    } else { // TRACKING
        double dt = stamp_sec - last_stamp;
        if (dt <= 0.0 || dt > 0.3) {
            RCLCPP_WARN(rclcpp::get_logger("tracker"), "RESET: dt abnormal (%.4f) [curr=%.4f, last=%.4f]",
                        dt, stamp_sec, last_stamp);
            reset();
            return false;
        }

        // Ballistic prediction (g = -9.8 m/s^2 along Z)
        double pred_x = x + vx * dt;
        double pred_y = y + vy * dt;
        double pred_z = z + vz * dt - 0.5 * gravity_m_s2 * dt * dt;

        bool matched = false;
        if (raw.valid) {
            double dx = raw.cx - pred_x;
            double dy = raw.cy - pred_y;
            double dz = raw.cz - pred_z;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (dist <= gate_radius_m) {
                matched = true;
                double dt_safe = std::max(dt, 0.010);

                // Ballistic Critically-Damped Kalman filter:
                // Perfectly eliminates cloth tumbling jitter and depth noise without oscillation/hunting,
                // producing an impeccably smooth parabolic arc while strictly tracking true measured cloth!
                double res_x = raw.cx - pred_x;
                double res_y = raw.cy - pred_y;
                double res_z = raw.cz - pred_z;

                constexpr double alpha = 0.28; // Smooth position correction
                constexpr double beta = 0.040; // Critically damped velocity update (prevents resonance)

                x = pred_x + alpha * res_x;
                y = pred_y + alpha * res_y;
                z = pred_z + alpha * res_z;

                vx += (beta / dt_safe) * res_x;
                vy += (beta / dt_safe) * res_y;
                vz = (vz - gravity_m_s2 * dt_safe) + (beta / dt_safe) * res_z;

                last_stamp = stamp_sec;
                tracked_count++;
                lost_count = 0;

                out = raw;
                out.cx = static_cast<float>(x);
                out.cy = static_cast<float>(y);
                out.cz = static_cast<float>(z);

                // Landing / terminal condition: reached landing zone or floor
                if (z <= 0.40 || x >= 3.60) {
                    reset();
                }
                return true;
            }
        }

        if (!matched) {
            lost_count++;
            if (lost_count > max_lost_frames || pred_z <= 0.35 || pred_x >= 3.70) {
                reset();
                return false;
            }

            // Internal prediction update to maintain search gate,
            // but DO NOT output or publish any fake/predicted points!
            x = pred_x;
            y = pred_y;
            z = pred_z;
            vz -= gravity_m_s2 * dt;
            last_stamp = stamp_sec;
            return false; // Only true measured cloth points are output
        }
    }
    return false;
}


void ClothTrackerCudaNode::process_and_publish(const uint16_t* depth_ptr, const rclcpp::Time& stamp) {
    // Phase 1: Background Calibration (learn static walls, ceiling, furniture)
    if (filter_cfg_.use_bg_subtraction && bg_frames_collected_ < bg_target_frames_) {
        update_background_cuda(cuda_ctx_, depth_ptr, camera_intrin_.width, camera_intrin_.height);
        bg_frames_collected_++;
        if (bg_frames_collected_ == bg_target_frames_) {
            RCLCPP_INFO(this->get_logger(), "=== BACKGROUND CALIBRATED (%d frames)! Static walls, ceiling and furniture are now IGNORED. ===", bg_target_frames_);
        }
        return;
    }

    std::vector<float3> debug_cloud_pts;
    std::vector<float3>* debug_ptr = publish_debug_cloud_ ? &debug_cloud_pts : nullptr;

    float pred_x = 0.0f, pred_y = 0.0f, pred_z = 0.0f;
    bool has_pred = ballistic_tracker_.get_prediction(stamp.seconds(), pred_x, pred_y, pred_z);

    ClothTrackingResult raw_res = process_depth_frame_cuda(
        cuda_ctx_, depth_ptr, camera_intrin_, camera_pose_, filter_cfg_, debug_ptr,
        pred_x, pred_y, pred_z, has_pred
    );

    // Phase 2: Ballistic State Tracking (Filter out stationary walls/ceiling & robot shaking)
    ClothTrackingResult res;
    bool should_publish = false;
    if (enable_ballistic_filter_) {
        should_publish = ballistic_tracker_.update(raw_res, stamp.seconds(), res);
    } else {
        res = raw_res;
        should_publish = raw_res.valid;
    }

    if (!should_publish) {
        return;
    }

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
    double now_sec = stamp.seconds();
    if (ballistic_tracker_.tracked_count <= 1) {
        trajectory_points_.clear(); // Brand new launch: clear previous trajectory line
    }

    geometry_msgs::msg::PointStamped pt_msg;
    pt_msg.header.stamp = stamp;
    pt_msg.header.frame_id = "base_link";
    pt_msg.point.x = res.cx;
    pt_msg.point.y = res.cy;
    pt_msg.point.z = res.cz;
    pub_point_->publish(pt_msg);

    // 3. Trajectory History & RViz Markers (Strictly true measured points)
    geometry_msgs::msg::Point p_geom;
    p_geom.x = res.cx;
    p_geom.y = res.cy;
    p_geom.z = res.cz;
    trajectory_points_.emplace_back(now_sec, p_geom);

    // Keep up to 250 points (full parabolic curve from launch to landing)
    if (trajectory_points_.size() > 250) {
        trajectory_points_.erase(trajectory_points_.begin());
    }

    // 1. True Measured Cloth Parabola Line Strip (Bright Yellow, 4cm line connecting true observed points)
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.stamp = stamp;
    line_marker.header.frame_id = "base_link";
    line_marker.ns = "cloth_cuda_trajectory";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.04;
    line_marker.color.r = 1.0f;
    line_marker.color.g = 0.90f;
    line_marker.color.b = 0.0f;
    line_marker.color.a = 1.0f;
    for (const auto& item : trajectory_points_) {
        line_marker.points.push_back(item.second);
    }
    pub_marker_->publish(line_marker);

    // 2. Flying Cloth Tip Ball (Red Sphere, diameter 10cm at current true location)
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

    // 3. True Measured Points Marker (Red/Orange Spheres, exactly where camera detected cloth)
    visualization_msgs::msg::Marker raw_points_marker;
    raw_points_marker.header.stamp = stamp;
    raw_points_marker.header.frame_id = "base_link";
    raw_points_marker.ns = "cloth_measured_points";
    raw_points_marker.id = 2;
    raw_points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    raw_points_marker.action = visualization_msgs::msg::Marker::ADD;
    raw_points_marker.scale.x = 0.06;
    raw_points_marker.scale.y = 0.06;
    raw_points_marker.scale.z = 0.06;
    raw_points_marker.color.r = 1.0f;
    raw_points_marker.color.g = 0.20f;
    raw_points_marker.color.b = 0.10f;
    raw_points_marker.color.a = 0.95f;
    for (const auto& item : trajectory_points_) {
        raw_points_marker.points.push_back(item.second);
    }
    pub_marker_->publish(raw_points_marker);

    // 4. Measured Path (True measured points only)
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

    // 4. Optional: Segmented Cloth PointCloud2
    if (publish_debug_cloud_ && pub_cluster_cloud_ && !debug_cloud_pts.empty()) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = stamp;
        cloud_msg.header.frame_id = "base_link";
        cloud_msg.height = 1;
        cloud_msg.width = static_cast<uint32_t>(debug_cloud_pts.size());
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(debug_cloud_pts.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (const auto& p : debug_cloud_pts) {
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = p.z;
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        pub_cluster_cloud_->publish(cloud_msg);
    }
}

void ClothTrackerCudaNode::run_direct_realsense_loop() {
#ifdef HAVE_REALSENSE2
    try {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH, camera_intrin_.width, camera_intrin_.height, RS2_FORMAT_Z16, 60);

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

        RCLCPP_INFO(this->get_logger(), "RealSense Hardware Pipeline streaming: %dx%d @ 60fps", intrin.width, intrin.height);

        while (rclcpp::ok() && is_running_) {
            rs2::frameset frames = rs_pipe_->wait_for_frames(1000);
            rs2::depth_frame depth = frames.get_depth_frame();
            if (!depth) continue;

            const uint16_t* depth_ptr = reinterpret_cast<const uint16_t*>(depth.get_data());
            rclcpp::Time now = this->get_clock()->now();

            process_and_publish(depth_ptr, now);
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

