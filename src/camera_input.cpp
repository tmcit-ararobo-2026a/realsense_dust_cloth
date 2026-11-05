// 画像トピックの受信、カメラTF、深度からgn10へ送る点群の作成。
#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace realsense_dust_cloth {

namespace {
constexpr float METERS_TO_MM = 1000.0f;
constexpr float MAX_UINT16_DEPTH_MM = 65535.0f;
constexpr double DEG_TO_RAD = M_PI / 180.0;
} // namespace

void ClothTrackerCudaNode::setup_camera_input() {
    if (input_source_ != "depth")
        return;
    if (use_direct_realsense_) {
        setup_direct_camera();
    } else {
        setup_camera_subscriptions();
    }
}

void ClothTrackerCudaNode::setup_camera_subscriptions() {
    const std::string depth_topic = get_parameter("depth_topic").as_string();
    const std::string camera_info_topic = get_parameter("camera_info_topic").as_string();
    const std::string color_topic = get_parameter("color_topic").as_string();
    auto depth_qos = rclcpp::SensorDataQoS().keep_last(depth_queue_size_);

    rclcpp::QoS color_qos(color_queue_size_);
    color_qos.best_effort();

    depth_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    color_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions depth_sub_opts;
    depth_sub_opts.callback_group = depth_cb_group_;

    rclcpp::SubscriptionOptions color_sub_opts;
    color_sub_opts.callback_group = color_cb_group_;

    sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, rclcpp::SensorDataQoS(),
        std::bind(&ClothTrackerCudaNode::camera_info_callback, this, std::placeholders::_1));

    sub_depth_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic, depth_qos,
        std::bind(&ClothTrackerCudaNode::depth_image_callback, this, std::placeholders::_1),
        depth_sub_opts);

    if (enable_color_stream_) {
        sub_color_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            color_topic, color_qos,
            std::bind(&ClothTrackerCudaNode::color_image_callback, this, std::placeholders::_1),
            color_sub_opts);

        sub_compressed_color_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            color_topic + "/compressed", color_qos,
            std::bind(&ClothTrackerCudaNode::compressed_color_callback, this,
                      std::placeholders::_1),
            color_sub_opts);
    }
    RCLCPP_INFO(this->get_logger(),
                "Listening on Depth Image topic: %s, Color Image topic: %s (+/compressed)",
                depth_topic.c_str(), color_topic.c_str());
}

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
    tf_color_opt.child_frame_id = color_optical_frame_;
    tf_color_opt.transform.rotation.x = q_opt.x();
    tf_color_opt.transform.rotation.y = q_opt.y();
    tf_color_opt.transform.rotation.z = q_opt.z();
    tf_color_opt.transform.rotation.w = q_opt.w();
    transforms.push_back(tf_color_opt);

    geometry_msgs::msg::TransformStamped tf_depth_opt = tf_color_opt;
    tf_depth_opt.child_frame_id = depth_optical_frame_;
    transforms.push_back(tf_depth_opt);

    tf_static_broadcaster_->sendTransform(transforms);
    RCLCPP_INFO(this->get_logger(),
                "Static TF published directly from config: %s -> %s (x=%.2f, z=%.2f, pitch=%.1f "
                "deg, roll=%.1f deg)",
                camera_parent_frame_.c_str(), camera_frame_.c_str(), camera_pose_.x_offset_m,
                camera_pose_.height_offset_m, camera_pose_.pitch_deg, camera_pose_.roll_deg);
}

void ClothTrackerCudaNode::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    std::lock_guard<std::recursive_mutex> lock(processing_mutex_);
    if (!intrinsics_ready_) {
        camera_intrin_.fx = static_cast<float>(msg->k[0]);
        camera_intrin_.fy = static_cast<float>(msg->k[4]);
        camera_intrin_.cx = static_cast<float>(msg->k[2]);
        camera_intrin_.cy = static_cast<float>(msg->k[5]);
        camera_intrin_.width = msg->width;
        camera_intrin_.height = msg->height;
        intrinsics_ready_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Camera intrinsics loaded: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f, %dx%d",
                    camera_intrin_.fx, camera_intrin_.fy, camera_intrin_.cx, camera_intrin_.cy,
                    camera_intrin_.width, camera_intrin_.height);
    }
}

void ClothTrackerCudaNode::depth_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    std::lock_guard<std::recursive_mutex> lock(processing_mutex_);
    camera_intrin_.width = msg->width;
    camera_intrin_.height = msg->height;
    if (!intrinsics_ready_) {
        publish_input_status("WAITING_FOR_CAMERA_INFO", false);
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
                depth_u16_buf[i] = (d_mm > MAX_UINT16_DEPTH_MM)
                                       ? static_cast<uint16_t>(MAX_UINT16_DEPTH_MM)
                                       : static_cast<uint16_t>(d_mm);
            }
        }
        depth_ptr = depth_u16_buf.data();
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), warning_log_throttle_ms_,
                             "Expected 16UC1/mono16 or 32FC1 depth encoding, got: %s",
                             msg->encoding.c_str());
        return;
    }

    // Use msg header stamp for robust and exact time delta calculation (works identically on rosbag & real hardware)
    rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0) {
        stamp = this->get_clock()->now();
    }
    process_and_publish(depth_ptr, stamp);
}

void ClothTrackerCudaNode::process_and_publish(const uint16_t* depth_ptr,
                                               const rclcpp::Time& stamp) {
    std::lock_guard<std::recursive_mutex> lock(processing_mutex_);
    double now_sec = stamp.seconds();
    if (last_frame_stamp_sec_ > 0.0 &&
        now_sec < last_frame_stamp_sec_ - bag_rewind_tolerance_sec_) {
        // Rosbag looped back to the beginning: clear all trajectory history and reset tracker
        trajectory_points_.clear();
        ballistic_tracker_.reset();
        has_cloth_pose_ = false;
        last_external_stamp_sec_ = -1.0;
        last_external_wall_sec_ = -1.0;
    }
    last_frame_stamp_sec_ = now_sec;

    if (std::abs((now() - stamp).seconds()) > image_max_age_sec_) {
        publish_input_status("IMAGE_TIME_INVALID", false);
        return;
    }
    try {
        const auto points = prepare_depth_cloud_cuda(cuda_ctx_, depth_ptr, camera_intrin_,
                                                     camera_pose_, filter_cfg_);
        publish_filter_input(points, stamp);
    } catch (const std::exception& error) {
        publish_input_status("FILTER_INPUT_ERROR", false);
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), status_log_throttle_ms_,
                              "Point export: %s", error.what());
    }
}

} // namespace realsense_dust_cloth
