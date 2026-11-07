// USB直結のRealSenseから深度と色画像を取得する。
#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"
#include <chrono>
#include <cmath>
#include <cstring>

namespace realsense_dust_cloth {

#ifdef HAVE_REALSENSE2
namespace {
constexpr size_t PLUMB_BOB_COEFF_COUNT = 5;
constexpr int REALSENSE_RECONNECT_DELAY_MS = 500;
constexpr int REALSENSE_STOP_POLL_SLICE_MS = 50;
sensor_msgs::msg::CameraInfo build_camera_info_msg(uint32_t width, uint32_t height, double fx,
                                                   double fy, double cx, double cy,
                                                   const std::string& frame_id,
                                                   const float* coeffs = nullptr) {
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
#endif

void ClothTrackerCudaNode::setup_direct_camera() {
    const std::string depth_topic = get_parameter("depth_topic").as_string();
    const std::string camera_info_topic = get_parameter("camera_info_topic").as_string();
    const std::string color_topic = get_parameter("color_topic").as_string();
#ifdef HAVE_REALSENSE2
    const auto sensor_qos = rclcpp::SensorDataQoS();
    pub_raw_depth_ = this->create_publisher<sensor_msgs::msg::Image>(depth_topic, sensor_qos);
    pub_depth_info_ =
        this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, sensor_qos);
    if (enable_color_stream_) {
        pub_raw_color_ = this->create_publisher<sensor_msgs::msg::Image>(color_topic, sensor_qos);
        pub_compressed_color_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/camera/camera/color/image_raw/compressed", sensor_qos);
        pub_color_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/color/camera_info", sensor_qos);
    }

    RCLCPP_INFO(this->get_logger(),
                "Starting Direct librealsense2 C++ API loop (Hardware Native)...");
    is_running_ = true;
    realsense_thread_ = std::thread(&ClothTrackerCudaNode::run_direct_realsense_loop, this);
#else
    (void)depth_topic;
    (void)camera_info_topic;
    (void)color_topic;
    throw std::runtime_error("use_direct_realsense=true requires a librealsense2 build; select the "
                             "Unity/rosbag YAML for topic input");
#endif
}

void ClothTrackerCudaNode::run_direct_realsense_loop() {
#ifdef HAVE_REALSENSE2
    // USB瞬断や射出時の電磁ノイズで単発タイムアウトしても、スレッドを終了させず自動再接続する。
    while (rclcpp::ok() && is_running_) {
        try {
            int req_width = 0;
            int req_height = 0;
            float fallback_scale = 0.001f;
            {
                std::lock_guard<std::recursive_mutex> lock(processing_mutex_);
                req_width = camera_intrin_.width;
                req_height = camera_intrin_.height;
                fallback_scale = camera_intrin_.depth_scale;
            }

            rs2::config cfg;
            cfg.enable_stream(RS2_STREAM_DEPTH, req_width, req_height, RS2_FORMAT_Z16, depth_fps_);
            if (enable_color_stream_) {
                cfg.enable_stream(RS2_STREAM_COLOR, req_width, req_height, RS2_FORMAT_BGR8,
                                  color_fps_);
            }

            rs_pipe_ = std::make_unique<rs2::pipeline>();
            rs2::pipeline_profile profile = rs_pipe_->start(cfg);

            float device_depth_scale = fallback_scale;
            try {
                auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
                const float sensor_scale = depth_sensor.get_depth_scale();
                if (std::isfinite(sensor_scale) && sensor_scale > 0.0f) {
                    device_depth_scale = sensor_scale;
                }
                if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
                    depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.0f);
                }
                if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
                    if (rs_laser_power_ < 0.0) {
                        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
                        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max);
                        RCLCPP_INFO(this->get_logger(),
                                    "RealSense Laser Power set to maximum (%.1f mW) for high-speed "
                                    "tracking",
                                    range.max);
                    } else {
                        depth_sensor.set_option(RS2_OPTION_LASER_POWER,
                                                static_cast<float>(rs_laser_power_));
                        RCLCPP_INFO(this->get_logger(), "RealSense Laser Power set to %.1f mW",
                                    rs_laser_power_);
                    }
                }
                if (depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                    if (rs_exposure_us_ > 0) {
                        depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
                        if (depth_sensor.supports(RS2_OPTION_EXPOSURE)) {
                            depth_sensor.set_option(RS2_OPTION_EXPOSURE,
                                                    static_cast<float>(rs_exposure_us_));
                            RCLCPP_INFO(this->get_logger(),
                                        "RealSense Depth Manual Exposure set to %d us",
                                        rs_exposure_us_);
                        }
                    } else {
                        depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
                        RCLCPP_INFO(this->get_logger(), "RealSense Depth Auto Exposure enabled");
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Could not set RealSense sensor options: %s",
                            e.what());
            }

            auto depth_stream =
                profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
            rs2_intrinsics intrin = depth_stream.get_intrinsics();
            {
                // ROSコールバック群とのデータ競合を防ぐためロック下で内部状態とGPU領域を更新する。
                std::lock_guard<std::recursive_mutex> lock(processing_mutex_);
                camera_intrin_.fx = intrin.fx;
                camera_intrin_.fy = intrin.fy;
                camera_intrin_.cx = intrin.ppx;
                camera_intrin_.cy = intrin.ppy;
                camera_intrin_.width = intrin.width;
                camera_intrin_.height = intrin.height;
                camera_intrin_.depth_scale = device_depth_scale;
                intrinsics_ready_ = true;

                const int stream_pixels = intrin.width * intrin.height;
                if (stream_pixels > cuda_ctx_.max_pixels) {
                    destroy_cuda_tracker(cuda_ctx_);
                    init_cuda_tracker(cuda_ctx_, intrin.width, intrin.height);
                }
            }

            sensor_msgs::msg::CameraInfo color_info_msg;
            if (enable_color_stream_) {
                try {
                    auto color_stream =
                        profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
                    rs2_intrinsics c_intrin = color_stream.get_intrinsics();
                    color_info_msg = build_camera_info_msg(
                        c_intrin.width, c_intrin.height, c_intrin.fx, c_intrin.fy, c_intrin.ppx,
                        c_intrin.ppy, color_optical_frame_, c_intrin.coeffs);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Failed to extract color intrinsics: %s",
                                e.what());
                }
            }

            RCLCPP_INFO(
                this->get_logger(),
                "RealSense Hardware Pipeline streaming: Depth %dx%d @ %dfps (Color enabled=%d)",
                intrin.width, intrin.height, depth_fps_, static_cast<int>(enable_color_stream_));

            sensor_msgs::msg::CameraInfo depth_info_msg =
                build_camera_info_msg(intrin.width, intrin.height, intrin.fx, intrin.fy, intrin.ppx,
                                      intrin.ppy, depth_optical_frame_, intrin.coeffs);

            while (rclcpp::ok() && is_running_) {
                rs2::frameset frames = rs_pipe_->wait_for_frames(realsense_wait_timeout_ms_);
                rclcpp::Time now = this->get_clock()->now();

                rs2::depth_frame depth = frames.get_depth_frame();
                if (depth) {
                    const uint16_t* depth_ptr = reinterpret_cast<const uint16_t*>(depth.get_data());
                    process_and_publish(depth_ptr, now);

                    // 生深度はYAMLで許可され、かつ購読者がいる場合だけ配信。CameraInfoとは独立。
                    if (publish_raw_depth_ && pub_raw_depth_ &&
                        pub_raw_depth_->get_subscription_count() > 0) {
                        sensor_msgs::msg::Image depth_img;
                        depth_img.header.stamp = now;
                        depth_img.header.frame_id = depth_optical_frame_;
                        depth_img.height = depth.get_height();
                        depth_img.width = depth.get_width();
                        depth_img.encoding = "16UC1";
                        depth_img.is_bigendian = false;
                        depth_img.step = depth.get_stride_in_bytes();
                        size_t depth_size =
                            static_cast<size_t>(depth.get_stride_in_bytes()) * depth.get_height();
                        depth_img.data.resize(depth_size);
                        std::memcpy(depth_img.data.data(), depth.get_data(), depth_size);
                        pub_raw_depth_->publish(depth_img);
                    }
                    if (pub_depth_info_ && pub_depth_info_->get_subscription_count() > 0) {
                        depth_info_msg.header.stamp = now;
                        pub_depth_info_->publish(depth_info_msg);
                    }
                }

                if (enable_color_stream_ &&
                    (pub_compressed_color_ || pub_rotated_color_ || pub_raw_color_)) {
                    rs2::video_frame color = frames.get_color_frame();
                    if (color) {
                        cv::Mat color_mat(cv::Size(color.get_width(), color.get_height()), CV_8UC3,
                                          const_cast<void*>(color.get_data()), cv::Mat::AUTO_STEP);
                        push_color_frame(color_mat, now,
                                         pub_color_info_ && color_info_msg.width > 0
                                             ? &color_info_msg
                                             : nullptr);
                    }
                }
            }

            rs_pipe_->stop();
            rs_pipe_.reset();
            break;
        } catch (const rs2::error& e) {
            RCLCPP_WARN(this->get_logger(), "RealSense stream interrupted (%s); reconnecting...",
                        e.what());
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "RealSense loop exception (%s); reconnecting...",
                        e.what());
        }

        if (rs_pipe_) {
            try {
                rs_pipe_->stop();
            } catch (const std::exception& stop_err) {
                RCLCPP_DEBUG(this->get_logger(), "RealSense stop cleanup note: %s",
                             stop_err.what());
            }
            rs_pipe_.reset();
        }

        if (!rclcpp::ok() || !is_running_) {
            break;
        }
        {
            std::lock_guard<std::recursive_mutex> lock(processing_mutex_);
            publish_input_status("REALSENSE_DISCONNECTED", false);
        }
        for (int elapsed = 0;
             elapsed < REALSENSE_RECONNECT_DELAY_MS && rclcpp::ok() && is_running_;
             elapsed += REALSENSE_STOP_POLL_SLICE_MS) {
            std::this_thread::sleep_for(std::chrono::milliseconds(REALSENSE_STOP_POLL_SLICE_MS));
        }
    }
#endif
}

} // namespace realsense_dust_cloth
