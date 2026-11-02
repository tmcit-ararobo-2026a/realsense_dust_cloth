#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"
#include "realsense_dust_cloth/point_cloud_io.hpp"
#include <chrono>
#include <cmath>

namespace realsense_dust_cloth {
namespace {
double wall_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

void ClothTrackerCudaNode::setup_external_cloud() {
    pub_filter_prediction_ = create_publisher<visualization_msgs::msg::Marker>(
        get_parameter("external_prediction_topic").as_string(), publisher_queue_size_);
    pub_cropped_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        cropped_cloud_topic_, rclcpp::SensorDataQoS());
    sub_dynamic_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        dynamic_cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ClothTrackerCudaNode::dynamic_cloud_callback, this, std::placeholders::_1));
    if (input_source_ == "cropped_cloud") {
        if (cropped_replay_topic_ == cropped_cloud_topic_)
            throw std::invalid_argument("Replay input/output topics must differ to avoid a feedback loop");
        sub_cropped_replay_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cropped_replay_topic_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::recursive_mutex> guard(processing_mutex_);
                const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
                if (std::abs((now() - stamp).seconds()) > image_max_age_sec_) return;
                try {
                    publish_filter_input(read_xyz_cloud(*msg, camera_parent_frame_, cuda_ctx_.max_pixels), stamp);
                } catch (const std::exception& error) {
                    publish_background_state("FILTER_INPUT_ERROR", false);
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), status_log_throttle_ms_,
                                         "Replay input: %s", error.what());
                }
            });
    }
    // A recorded dynamic cloud is already filtered. Recorded status messages must not override this replay.
    if (input_source_ != "dynamic_cloud") {
        sub_filter_status_ = create_subscription<std_msgs::msg::String>(
            get_parameter("external_filter_status_topic").as_string(), rclcpp::QoS(1).transient_local(),
            [this](std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::recursive_mutex> guard(processing_mutex_);
                if (msg->data != "READY") publish_background_state("FILTER_" + msg->data, false);
            });
    }
    external_watchdog_ = create_wall_timer(std::chrono::duration<double>(external_poll_sec_), [this] {
        std::lock_guard<std::recursive_mutex> guard(processing_mutex_);
        if (last_external_wall_sec_ < 0 ||
            wall_seconds() - last_external_wall_sec_ > external_result_timeout_sec_) {
            if (ballistic_tracker_.state == TrackerState::TRACKING)
                publish_tracking_result(ClothTrackingResult{}, now(), {}, {});
            publish_background_state("RESULT_TIMEOUT", false);
        }
    });
}

void ClothTrackerCudaNode::publish_filter_input(const std::vector<float3>& points,
                                               const rclcpp::Time& stamp) {
    visualization_msgs::msg::Marker region;
    region.header.stamp = stamp;
    region.header.frame_id = camera_parent_frame_;
    region.ns = "cloth_filter_prediction";
    region.id = 0;
    region.type = visualization_msgs::msg::Marker::SPHERE;
    region.pose.orientation.w = 1.0;
    float px = 0, py = 0, pz = 0;
    const bool protect = ballistic_tracker_.state == TrackerState::TRACKING &&
                         ballistic_tracker_.launch_displacement_confirmed &&
                         ballistic_tracker_.get_prediction(stamp.seconds(), px, py, pz);
    region.action = protect ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    region.pose.position.x = px;
    region.pose.position.y = py;
    region.pose.position.z = pz;
    region.scale.x = region.scale.y = region.scale.z = external_keep_radius_m_ * 2;
    pub_filter_prediction_->publish(region);
    MarkerVisualizer::publish_point_cloud(pub_cropped_cloud_, points, stamp, camera_parent_frame_);
}

void ClothTrackerCudaNode::dynamic_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::recursive_mutex> guard(processing_mutex_);
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    if (std::abs((now() - stamp).seconds()) > image_max_age_sec_) {
        publish_background_state("IMAGE_TIME_INVALID", false);
        return;
    }
    if (last_external_stamp_sec_ >= 0 && stamp.seconds() < last_external_stamp_sec_ - bag_rewind_tolerance_sec_) {
        ballistic_tracker_.reset();
        trajectory_points_.clear();
        has_cloth_pose_ = false;
        prev_state_ = TrackerState::IDLE;
        last_external_stamp_sec_ = -1;
    }
    if (stamp.seconds() <= last_external_stamp_sec_) return;
    try {
        const auto points = read_xyz_cloud(*msg, camera_parent_frame_, cuda_ctx_.max_pixels);
        float px = 0, py = 0, pz = 0;
        bool predicted = ballistic_tracker_.get_prediction(stamp.seconds(), px, py, pz);
        auto search_filter = filter_cfg_;
        if (!predicted && ballistic_tracker_.is_pending_launch) {
            // Only the first observation must be in the launch box. A fast cloth may
            // leave that box before the next camera frame confirms its displacement.
            const double dt = stamp.seconds() - ballistic_tracker_.first_detect_stamp;
            if (dt > 0 && dt <= ballistic_tracker_.launch_timeout_sec) {
                px = ballistic_tracker_.anchor_x;
                py = ballistic_tracker_.anchor_y;
                pz = ballistic_tracker_.anchor_z;
                const double speed_bound = std::sqrt(
                    std::pow(ballistic_tracker_.max_vx_m_s, 2) +
                    std::pow(ballistic_tracker_.max_abs_vy_m_s, 2) +
                    std::pow(std::max(std::abs(ballistic_tracker_.min_vz_m_s), ballistic_tracker_.max_vz_m_s), 2));
                search_filter.gate_radius_m = speed_bound * dt + ballistic_tracker_.centroid_position_tolerance_m;
                search_filter.min_tracking_points = search_filter.min_cluster_points;
                predicted = true;
            }
        }
        std::vector<float3> cluster, filtered;
        const auto result = process_point_cloud_cuda(cuda_ctx_, points, search_filter,
            publish_debug_cloud_ ? &cluster : nullptr, publish_debug_cloud_ ? &filtered : nullptr,
            px, py, pz, predicted);
        if (cudaGetLastError() != cudaSuccess) throw std::runtime_error("CUDA point tracking failed");
        last_external_wall_sec_ = wall_seconds();
        last_external_stamp_sec_ = stamp.seconds();
        publish_tracking_result(result, stamp, cluster, filtered);
    } catch (const std::exception& error) {
        publish_background_state("TRACKING_ERROR", false);
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), status_log_throttle_ms_,
                             "Dynamic tracking: %s", error.what());
    }
}
} // namespace realsense_dust_cloth
