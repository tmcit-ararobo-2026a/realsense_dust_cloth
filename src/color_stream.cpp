// 深度処理を止めずに、最新の色画像だけを非同期で配信する。
#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace realsense_dust_cloth {

namespace {
constexpr int ROTATION_90_CW_DEG = 90;
constexpr int ROTATION_180_DEG = 180;
constexpr int ROTATION_270_CW_DEG = 270;
sensor_msgs::msg::Image mat_to_image_msg(const cv::Mat& mat, const rclcpp::Time& stamp,
                                         const std::string& frame_id) {
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

} // namespace

void ClothTrackerCudaNode::push_color_frame(const cv::Mat& bgr_mat, const rclcpp::Time& stamp,
                                            const sensor_msgs::msg::CameraInfo* info) {
    if (!color_worker_running_ || bgr_mat.empty())
        return;
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
            if (!color_worker_running_)
                break;
            task = std::move(latest_color_task_);
            latest_color_task_.has_new_frame = false;
        }

        try {
            if (task.is_compressed) {
                if (task.compressed_data.empty() || !pub_rotated_color_ ||
                    pub_rotated_color_->get_subscription_count() == 0) {
                    continue;
                }
                cv::Mat decoded = cv::imdecode(task.compressed_data, cv::IMREAD_COLOR);
                if (!decoded.empty()) {
                    pub_rotated_color_->publish(
                        mat_to_image_msg(decoded, task.stamp, task.frame_id));
                }
                continue;
            }

            if (task.bgr_frame.empty())
                continue;

            // 1. Raw color publication if requested
            if (pub_raw_color_ && pub_raw_color_->get_subscription_count() > 0) {
                pub_raw_color_->publish(
                    mat_to_image_msg(task.bgr_frame, task.stamp, color_optical_frame_));
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
                pub_rotated_color_->publish(
                    mat_to_image_msg(rotated_frame, task.stamp, color_optical_frame_));
            }

            // 4. JPEG Compressed Color (/camera/camera/color/image_raw/compressed)
            if (pub_compressed_color_) {
                const std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
                compressed_buf.clear();
                cv::imencode(".jpg", rotated_frame, compressed_buf, jpeg_params);

                sensor_msgs::msg::CompressedImage comp_msg;
                comp_msg.header.stamp = task.stamp;
                comp_msg.header.frame_id = color_optical_frame_;
                comp_msg.format = "jpeg";
                comp_msg.data = std::move(compressed_buf);
                pub_compressed_color_->publish(comp_msg);
            }

            // 5. Camera Info
            if (pub_color_info_) {
                if (task.has_color_info) {
                    task.color_info.header.stamp = task.stamp;
                    task.color_info.header.frame_id = color_optical_frame_;
                    pub_color_info_->publish(task.color_info);
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), warning_log_throttle_ms_,
                                 "Color worker loop error: %s", e.what());
        }
    }
}

void ClothTrackerCudaNode::color_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    if (!pub_compressed_color_ && !pub_rotated_color_ && !pub_raw_color_)
        return;
    raw_color_received_ = true;

    try {
        cv::Mat frame;
        if (msg->encoding == "bgr8") {
            frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uchar*>(msg->data.data()),
                            msg->step);
        } else if (msg->encoding == "rgb8") {
            cv::Mat rgb_mat(msg->height, msg->width, CV_8UC3, const_cast<uchar*>(msg->data.data()),
                            msg->step);
            cv::cvtColor(rgb_mat, frame, cv::COLOR_RGB2BGR);
        } else if (msg->encoding == "mono8") {
            frame = cv::Mat(msg->height, msg->width, CV_8UC1, const_cast<uchar*>(msg->data.data()),
                            msg->step);
        } else {
            return;
        }

        rclcpp::Time stamp(msg->header.stamp);
        if (stamp.nanoseconds() == 0) {
            stamp = this->get_clock()->now();
        }
        push_color_frame(frame, stamp);
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), warning_log_throttle_ms_,
                             "Color decode error: %s", e.what());
    }
}

void ClothTrackerCudaNode::compressed_color_callback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
    // When replaying a rosbag that only recorded /camera/camera/color/image_raw/compressed,
    // offload JPEG decoding to the asynchronous color worker thread so depth tracking never stalls.
    if (raw_color_received_ || !pub_rotated_color_ ||
        pub_rotated_color_->get_subscription_count() == 0 || msg->data.empty()) {
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

} // namespace realsense_dust_cloth
