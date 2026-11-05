// ノードの起動・終了。各処理の実装は役割ごとのcppに置く。
#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"

namespace realsense_dust_cloth {

ClothTrackerCudaNode::ClothTrackerCudaNode(const rclcpp::NodeOptions& options)
    : Node("cloth_tracker_cuda_node", options) {
    declare_and_load_parameters();
    setup_tracking_status();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_static_transforms();
    pub_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/cloth/position",
                                                                          publisher_queue_size_);
    pub_landing_pos_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        landing_position_topic_, publisher_queue_size_);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/cloth/trajectory_marker", publisher_queue_size_);
    pub_path_ =
        this->create_publisher<nav_msgs::msg::Path>("/cloth/measured_path", publisher_queue_size_);
    const auto sensor_qos = rclcpp::SensorDataQoS();
    pub_rotated_color_ =
        this->create_publisher<sensor_msgs::msg::Image>("/cloth/camera_view", sensor_qos);

    // Debug publishers (always available for RViz visualization & verification)
    pub_roi_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/cloth/debug/roi_markers", publisher_queue_size_);
    pub_cluster_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloth/debug/cluster_points", sensor_qos);
    pub_filtered_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloth/debug/filtered_points", sensor_qos);

    setup_external_cloud();

    // Initialize CUDA memory and stream
    cuda_ctx_.ring_frames = filter_cfg_.bg_ring_frames > 0 ? filter_cfg_.bg_ring_frames : 6;
    if (!init_cuda_tracker(cuda_ctx_, camera_intrin_.width, camera_intrin_.height)) {
        throw std::runtime_error("Failed to initialize CUDA tracker context");
    }

    setup_camera_input();

    // Start asynchronous color worker thread
    color_worker_running_ = enable_color_stream_;
    if (enable_color_stream_)
        color_worker_thread_ = std::thread(&ClothTrackerCudaNode::color_worker_loop, this);

    RCLCPP_INFO(this->get_logger(), "RealSense Cloth CUDA Tracker Node initialized successfully.");
}

ClothTrackerCudaNode::~ClothTrackerCudaNode() {
    is_running_ = false;
    color_worker_running_ = false;
    color_cv_.notify_all();
    if (color_worker_thread_.joinable()) {
        color_worker_thread_.join();
    }
    if (realsense_thread_.joinable()) {
        realsense_thread_.join();
    }
    destroy_cuda_tracker(cuda_ctx_);
}

} // namespace realsense_dust_cloth
