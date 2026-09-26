#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace realsense_dust_cloth {

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

        rclcpp::QoS qos(5);
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
    this->declare_parameter("camera_x_offset_m", 0.25);
    this->declare_parameter("camera_height_offset_m", 0.35);

    this->declare_parameter("min_depth_m", 0.25);
    this->declare_parameter("max_depth_m", 6.0);

    this->declare_parameter("min_x_m", 0.30);
    this->declare_parameter("max_x_m", 6.0);
    this->declare_parameter("min_y_m", -1.5);
    this->declare_parameter("max_y_m", 1.5);
    this->declare_parameter("min_z_m", 0.05);
    this->declare_parameter("max_z_m", 3.5);

    this->declare_parameter("voxel_size_m", 0.03);
    this->declare_parameter("cluster_eps_m", 0.15);
    this->declare_parameter("min_cluster_points", 10);
    this->declare_parameter("max_cluster_points", 1500);
    this->declare_parameter("max_cloth_size_m", 0.65);

    this->declare_parameter("publish_debug_cloud", false);
    this->declare_parameter("trajectory_history_sec", 3.0);
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

    publish_debug_cloud_ = this->get_parameter("publish_debug_cloud").as_bool();
    trajectory_history_sec_ = this->get_parameter("trajectory_history_sec").as_double();
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
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received depth frame: %dx%d, processing via CUDA...", msg->width, msg->height);
    process_and_publish(depth_ptr, msg->header.stamp);
}

void ClothTrackerCudaNode::process_and_publish(const uint16_t* depth_ptr, const rclcpp::Time& stamp) {
    std::vector<float3> debug_cloud_pts;
    std::vector<float3>* debug_ptr = publish_debug_cloud_ ? &debug_cloud_pts : nullptr;

    ClothTrackingResult res = process_depth_frame_cuda(
        cuda_ctx_, depth_ptr, camera_intrin_, camera_pose_, filter_cfg_, debug_ptr
    );

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "CUDA Result: valid=%s, points=%d, pos=(%.2f, %.2f, %.2f)",
                         res.valid ? "TRUE" : "FALSE", res.cluster_points_count, res.cx, res.cy, res.cz);

    if (!res.valid) {
        return;
    }

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

    // 2. Publish Centroid Point
    geometry_msgs::msg::PointStamped pt_msg;
    pt_msg.header.stamp = stamp;
    pt_msg.header.frame_id = "base_link";
    pt_msg.point.x = res.cx;
    pt_msg.point.y = res.cy;
    pt_msg.point.z = res.cz;
    pub_point_->publish(pt_msg);

    // 3. Trajectory History & RViz Marker
    double now_sec = stamp.seconds();
    if (!trajectory_points_.empty()) {
        double last_t = trajectory_points_.back().first;
        if ((now_sec - last_t) > 1.5) {
            trajectory_points_.clear();
        }
    }

    geometry_msgs::msg::Point p_geom;
    p_geom.x = res.cx;
    p_geom.y = res.cy;
    p_geom.z = res.cz;
    trajectory_points_.emplace_back(now_sec, p_geom);

    // Filter points older than trajectory_history_sec_
    std::vector<std::pair<double, geometry_msgs::msg::Point>> valid_traj;
    for (const auto& item : trajectory_points_) {
        if ((now_sec - item.first) <= trajectory_history_sec_) {
            valid_traj.push_back(item);
        }
    }
    trajectory_points_ = std::move(valid_traj);

    // Marker
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = "base_link";
    marker.ns = "cloth_cuda_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.035; // 3.5cm thickness
    marker.color.r = 1.0f;
    marker.color.g = 0.85f;
    marker.color.b = 0.0f;
    marker.color.a = 0.95f;
    for (const auto& item : trajectory_points_) {
        marker.points.push_back(item.second);
    }
    pub_marker_->publish(marker);

    // Path
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

