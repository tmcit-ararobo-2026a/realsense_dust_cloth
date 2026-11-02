#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"

namespace realsense_dust_cloth {
void ClothTrackerCudaNode::setup_tracking_status() {
    const auto qos = rclcpp::QoS(1).transient_local();
    pub_background_status_ = create_publisher<std_msgs::msg::String>("/cloth/background_status", qos);
    pub_ready_ = create_publisher<std_msgs::msg::Bool>("/cloth/ready", qos);
    pub_tracking_state_ = create_publisher<std_msgs::msg::String>("/cloth/tracking_state", qos);
    publish_background_state("INITIALIZING", false);
}

void ClothTrackerCudaNode::publish_background_state(const std::string& state, bool ready) {
    std_msgs::msg::String status;
    status.data = state;
    pub_background_status_->publish(status);
    std_msgs::msg::Bool value;
    value.data = ready;
    pub_ready_->publish(value);
    switch (ballistic_tracker_.state) {
        case TrackerState::TRACKING: status.data = "TRACKING"; break;
        case TrackerState::LANDED: status.data = "LANDED"; break;
        case TrackerState::LOST: status.data = "LOST"; break;
        case TrackerState::RESTING: status.data = "RESTING"; break;
        default: status.data = "IDLE"; break;
    }
    pub_tracking_state_->publish(status);
}
} // namespace realsense_dust_cloth
