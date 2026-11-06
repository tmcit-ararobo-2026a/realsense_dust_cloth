// 実際のノードとCUDAを使用し、入力経路の分離・背景復帰・候補探索を検証する。
#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"
#include <cassert>
#include <iostream>
#include <fstream>
#include <iterator>

namespace realsense_dust_cloth {
struct ClothTrackerNodeTestAccess {
    static void verify(ClothTrackerCudaNode& node) {
        // 深度経路にgn10の入力もタイムアウト監視も接続しない。
        assert(!node.external_watchdog_);
        assert(!node.sub_dynamic_cloud_);
        assert(!node.sub_filter_status_);
        assert(!node.pub_cropped_cloud_);

        const auto original_history = node.cuda_ctx_.ring_frames;
        assert(original_history == node.filter_cfg_.bg_ring_frames);
        std::vector<uint16_t> depth(node.camera_intrin_.width * node.camera_intrin_.height, 0);
        for (auto terminal : {TrackerState::IDLE, TrackerState::LOST,
                              TrackerState::LANDED, TrackerState::RESTING}) {
            node.ballistic_tracker_.reset();
            node.ballistic_tracker_.state = terminal;
            node.ballistic_tracker_.is_pending_launch = true;
            node.bg_state_ = ClothTrackerCudaNode::BackgroundState::FROZEN;
            node.shot_freeze_until_sec_ = node.now().seconds() - 1;
            node.last_background_update_sec_ = -1;
            node.process_and_publish(depth.data(), node.now());
            assert(node.bg_state_ == ClothTrackerCudaNode::BackgroundState::ROLLING);
            assert(node.cuda_ctx_.bg_ready);
            assert(!node.ballistic_tracker_.is_pending_launch);
        }

        node.ballistic_tracker_.reset();
        node.ballistic_tracker_.state = TrackerState::TRACKING;
        node.ballistic_tracker_.last_stamp = node.now().seconds() - .01;
        node.ballistic_tracker_.launch_stamp = node.ballistic_tracker_.last_stamp;
        node.bg_state_ = ClothTrackerCudaNode::BackgroundState::FROZEN;
        node.shot_freeze_until_sec_ = node.now().seconds() - 1;
        node.process_and_publish(depth.data(), node.now());
        assert(node.bg_state_ == ClothTrackerCudaNode::BackgroundState::FROZEN);
        assert(node.ballistic_tracker_.state == TrackerState::TRACKING);

        // 射出確認中の次フレームは発射枠外も探す（直接深度と外部点群で共通）。
        node.ballistic_tracker_.reset();
        auto& tracker = node.ballistic_tracker_;
        tracker.is_pending_launch = true;
        tracker.anchor_x = 1.0;
        tracker.anchor_y = 0;
        tracker.anchor_z = 1.6;
        tracker.first_detect_stamp = 10;
        float x = 0, y = 0, z = 0;
        bool predicted = false;
        const auto pending = node.candidate_search_filter(10.05, x, y, z, predicted);
        assert(predicted && x == 1 && z == 1.6f);
        assert(pending.gate_radius_m > .5);
        tracker.state = TrackerState::TRACKING;
        tracker.x = 2;
        tracker.y = 0;
        tracker.z = 2;
        tracker.vx = tracker.vy = tracker.vz = 0;
        tracker.last_stamp = 10;
        tracker.lost_count = 1;
        const auto expanded = node.candidate_search_filter(10.01, x, y, z, predicted);
        assert(predicted);
        assert(expanded.gate_radius_m > tracker.tube_radius_m);

        // GPUでも通常ゲート外・拡張ゲート内のクラスタが採用されること。
        CudaTrackerContext context;
        assert(init_cuda_tracker(context, 32, 32));
        FilterConfig filter;
        filter.min_tracking_points = 4;
        filter.min_cloth_size_m = .03;
        filter.cluster_eps_m = .1;
        filter.gate_radius_m = .35;
        std::vector<float3> points;
        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 4; ++j)
                points.push_back(make_float3(2.38f, (i - 2) * .01f, 2 + (j - 1.5f) * .01f));
        assert(!process_point_cloud_cuda(context, points, filter, nullptr, nullptr, 2, 0, 2, true).valid);
        filter.gate_radius_m = .39;
        assert(process_point_cloud_cuda(context, points, filter, nullptr, nullptr, 2, 0, 2, true).valid);
        // 点数の多い別クラスタより、予測近傍の雑巾を優先する。
        filter.prediction_distance_weight = 2;
        filter.cluster_size_weight = .05;
        filter.max_cluster_points = 8000;
        points.clear();
        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 4; ++j)
                points.push_back(make_float3(2.05f, (i - 2) * .012f, 2 + (j - 1.5f) * .012f));
        for (int i = 0; i < 20; ++i)
            for (int j = 0; j < 20; ++j)
                points.push_back(make_float3(2.30f, (i - 9.5f) * .004f, 2 + (j - 9.5f) * .004f));
        const auto closest = process_point_cloud_cuda(context, points, filter, nullptr, nullptr, 2, 0, 2, true);
        assert(closest.valid && closest.cx < 2.1f && closest.cluster_points_count == 20);
        destroy_cuda_tracker(context);

        // 購読者なしではデバッグ抽出しない。表示周期は追跡とは独立。
        node.debug_frame_due_ = true;
        assert(!node.want_cluster_debug() && !node.want_filtered_debug());
        node.last_debug_publish_ = {};
        assert(node.debug_frame_due());
        assert(!node.debug_frame_due());
        node.last_debug_publish_ -= std::chrono::seconds(1);
        assert(node.debug_frame_due());
        const auto rejected = node.set_parameters_atomically({rclcpp::Parameter("debug_publish_hz", 0.0)});
        assert(!rejected.successful);
        assert(node.debug_publish_hz_ > 0);
        assert(node.set_parameters_atomically({rclcpp::Parameter("debug_publish_hz", 10.0)}).successful);
        assert(node.debug_publish_hz_ == 10);
        // 背景更新予定のフレームで現れた雑巾を、検出前に背景へ吸収しない。
        node.ballistic_tracker_.reset();
        node.bg_state_ = ClothTrackerCudaNode::BackgroundState::ROLLING;
        node.camera_intrin_.fx = node.camera_intrin_.fy = 433.0f;
        node.camera_intrin_.cx = 320.0f;
        node.camera_intrin_.cy = 240.0f;
        reset_background_cuda(node.cuda_ctx_);
        std::fill(depth.begin(), depth.end(), 4000);
        node.process_and_publish(depth.data(), node.now());
        for (int v = 200; v <= 280; ++v)
            for (int u = 28; u <= 108; ++u)
                depth[v * node.camera_intrin_.width + u] = 675;
        node.last_background_update_sec_ = -1;
        node.process_and_publish(depth.data(), node.now());
        assert(node.ballistic_tracker_.is_pending_launch);
        assert(node.bg_state_ == ClothTrackerCudaNode::BackgroundState::FROZEN);
        node.ballistic_tracker_.reset();
        std::fill(depth.begin(), depth.end(), 0);
        assert(node.set_parameters_atomically({rclcpp::Parameter("use_bg_subtraction", false)}).successful);
        node.process_and_publish(depth.data(), node.now());
        assert(node.cuda_ctx_.ring_count == 0);
        assert(!node.cuda_ctx_.bg_ready);

        // RViz相当の購読者を接続し、実際の配信数が入力フレーム数より少ないこと。
        auto viewer = std::make_shared<rclcpp::Node>("cloth_debug_test_viewer");
        int received = 0;
        auto subscription = viewer->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloth/debug/filtered_points", rclcpp::SensorDataQoS(),
            [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr) { ++received; });
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(viewer);
        for (int i = 0; i < 200 && node.pub_filtered_cloud_->get_subscription_count() == 0; ++i) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        assert(node.pub_filtered_cloud_->get_subscription_count() > 0);
        node.last_debug_publish_ = {};
        int frames = 0;
        const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(350);
        while (std::chrono::steady_clock::now() < end) {
            node.process_and_publish(depth.data(), node.now());
            executor.spin_some();
            ++frames;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        executor.spin_some();
        assert(received >= 2 && received <= 4);
        assert(frames > received * 2);

        int states = 0, paths = 0, markers = 0;
        auto status_subscription = viewer->create_subscription<std_msgs::msg::String>(
            "/cloth/tracking_state", rclcpp::QoS(1).transient_local(),
            [&](std_msgs::msg::String::ConstSharedPtr) { ++states; });
        auto path_subscription = viewer->create_subscription<nav_msgs::msg::Path>(
            "/cloth/measured_path", 10, [&](nav_msgs::msg::Path::ConstSharedPtr) { ++paths; });
        auto marker_subscription = viewer->create_subscription<visualization_msgs::msg::Marker>(
            "/cloth/trajectory_marker", 10,
            [&](visualization_msgs::msg::Marker::ConstSharedPtr) { ++markers; });
        for (int i = 0; i < 200 && (states == 0 || node.pub_path_->get_subscription_count() == 0); ++i) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        assert(states == 1); // 後から接続しても最新の状態を受信できる。
        for (int i = 0; i < 20; ++i) {
            node.publish_input_status("READY", true);
            executor.spin_some();
        }
        assert(states == 1); // 状態が同じフレームを毎回配信しない。
        node.ballistic_tracker_.state = TrackerState::LOST;
        node.publish_input_status("READY", true);
        node.has_cloth_pose_ = true;
        node.trajectory_points_.clear();
        geometry_msgs::msg::Point first, last;
        first.x = 1; first.z = 1.6;
        last.x = 2; last.z = 1;
        node.trajectory_points_.emplace_back(node.now().seconds(), first);
        node.trajectory_points_.emplace_back(node.now().seconds(), last);
        node.last_static_publish_sec_ = 0;
        const auto display_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
        while (std::chrono::steady_clock::now() < display_end) {
            node.publish_tracking_visualization(node.now(), false);
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        executor.spin_some();
        assert(states == 2);
        assert(paths >= 2 && paths <= 4);
        assert(markers == paths * 3);
    }
};
} // namespace realsense_dust_cloth

int main(int argc, char** argv) {
    int devices = 0;
    if (cudaGetDeviceCount(&devices) != cudaSuccess || devices == 0)
        return 77;
    assert(argc == 2);
    auto contents = [](const std::string& path) {
        std::ifstream input(path);
        return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
    };
    const auto yaml_before = contents(argv[1]);
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", argv[1], "-p", "use_direct_realsense:=false"});
    {
        realsense_dust_cloth::ClothTrackerCudaNode node(options);
        realsense_dust_cloth::ClothTrackerNodeTestAccess::verify(node);
    }
    assert(contents(argv[1]) == yaml_before);
    rclcpp::shutdown();
    std::cout << "Direct input isolation, terminal background recovery, pending/growing search and debug policy passed\n";
}
