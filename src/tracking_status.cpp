#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"

namespace realsense_dust_cloth {
void ClothTrackerCudaNode::setup_tracking_status() {
    const auto qos = rclcpp::QoS(1).transient_local();
    pub_input_status_ = create_publisher<std_msgs::msg::String>("/cloth/background_status", qos);
    pub_ready_ = create_publisher<std_msgs::msg::Bool>("/cloth/ready", qos);
    pub_tracking_state_ = create_publisher<std_msgs::msg::String>("/cloth/tracking_state", qos);
    publish_input_status("INITIALIZING", false);
}

void ClothTrackerCudaNode::publish_input_status(const std::string& state, bool ready) {
    std_msgs::msg::String status;
    status.data = state;
    if (state != last_input_status_) {
        pub_input_status_->publish(status);
        last_input_status_ = state;
    }
    std_msgs::msg::Bool value;
    value.data = ready;
    if (!ready_published_ || ready != last_ready_) {
        pub_ready_->publish(value);
        last_ready_ = ready;
        ready_published_ = true;
    }
    switch (ballistic_tracker_.state) {
    case TrackerState::TRACKING:
        status.data = "TRACKING";
        break;
    case TrackerState::LANDED:
        status.data = "LANDED";
        break;
    case TrackerState::LOST:
        status.data = "LOST";
        break;
    case TrackerState::RESTING:
        status.data = "RESTING";
        break;
    default:
        status.data = "IDLE";
        break;
    }
    if (status.data != last_tracking_status_) {
        pub_tracking_state_->publish(status);
        last_tracking_status_ = status.data;
    }
}
} // namespace realsense_dust_cloth
