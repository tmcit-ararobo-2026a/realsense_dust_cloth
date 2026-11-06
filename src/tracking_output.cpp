// 採用された実測位置・着地・軌跡・表示を配信する。
#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"
#include "realsense_dust_cloth/marker_visualizer.hpp"

namespace realsense_dust_cloth {

bool ClothTrackerCudaNode::debug_frame_due() {
    const auto now = std::chrono::steady_clock::now();
    if (last_debug_publish_.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(now - last_debug_publish_).count() < 1.0 / debug_publish_hz_)
        return false;
    last_debug_publish_ = now;
    return true;
}

bool ClothTrackerCudaNode::want_cluster_debug() const {
    return publish_debug_cloud_ && debug_frame_due_ && pub_cluster_cloud_ &&
           pub_cluster_cloud_->get_subscription_count() > 0;
}

bool ClothTrackerCudaNode::want_filtered_debug() const {
    return publish_debug_cloud_ && debug_frame_due_ && pub_filtered_cloud_ &&
           pub_filtered_cloud_->get_subscription_count() > 0;
}

void ClothTrackerCudaNode::publish_roi_markers(const rclcpp::Time& stamp,
                                               const ClothTrackingResult& raw_res) {
    if (!pub_roi_markers_)
        return;

    double speed = std::sqrt(ballistic_tracker_.vx * ballistic_tracker_.vx +
                             ballistic_tracker_.vy * ballistic_tracker_.vy +
                             ballistic_tracker_.vz * ballistic_tracker_.vz);

    auto markers = MarkerVisualizer::create_roi_markers(
        stamp, filter_cfg_, ballistic_tracker_.launch_min_x, ballistic_tracker_.launch_max_x,
        ballistic_tracker_.launch_min_y, ballistic_tracker_.launch_max_y,
        ballistic_tracker_.launch_min_z, ballistic_tracker_.launch_max_z,
        ballistic_tracker_.state == TrackerState::TRACKING, ballistic_tracker_.x,
        ballistic_tracker_.y, ballistic_tracker_.z, ballistic_tracker_.vx, ballistic_tracker_.vz,
        speed, raw_res, camera_pose_, trajectory_points_, camera_parent_frame_);

    pub_roi_markers_->publish(markers);
}

void ClothTrackerCudaNode::publish_tracking_result(const ClothTrackingResult& raw_res,
                                                   const rclcpp::Time& stamp,
                                                   const std::vector<float3>& debug_cluster_pts,
                                                   const std::vector<float3>& debug_filtered_pts) {
    // 候補を追跡器へ渡し、採用された実測と状態の変化を配信する。
    ClothTrackingResult res;
    const bool is_actively_tracked = ballistic_tracker_.update(raw_res, stamp.seconds(), res);

    publish_input_status("READY", ballistic_tracker_.state != TrackerState::TRACKING);

    // Publish Debug Point Clouds & ROI Markers after tracker state update
    if (publish_debug_cloud_ && debug_frame_due_) {
        if (pub_filtered_cloud_ && pub_filtered_cloud_->get_subscription_count() > 0) {
            MarkerVisualizer::publish_point_cloud(pub_filtered_cloud_, debug_filtered_pts, stamp,
                                                  camera_parent_frame_);
        }
        if (pub_roi_markers_ && pub_roi_markers_->get_subscription_count() > 0) {
            publish_roi_markers(stamp, raw_res);
        }
    }

    // 今回採用したクラスタだけを表示し、それ以外では表示を消す（購読者がいる時のみ配信）。
    if (want_cluster_debug()) {
        if (is_actively_tracked && !debug_cluster_pts.empty()) {
            MarkerVisualizer::publish_point_cloud(pub_cluster_cloud_, debug_cluster_pts, stamp,
                                                  camera_parent_frame_);
        } else {
            std::vector<float3> empty_pts;
            MarkerVisualizer::publish_point_cloud(pub_cluster_cloud_, empty_pts, stamp,
                                                  camera_parent_frame_);
        }
    }

    // ログ周期はYAMLのstatus_log_throttle_msに従う。
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), status_log_throttle_ms_,
                         "[ClothTracker] Debug search points: %zu | Candidate cluster: %d pts at "
                         "(%.2f, %.2f, %.2f) | State: %s",
                         debug_filtered_pts.size(), raw_res.cluster_points_count, raw_res.cx,
                         raw_res.cy, raw_res.cz,
                         ballistic_tracker_.state == TrackerState::TRACKING ? "TRACKING" : "IDLE");

    publish_tracking_transition(raw_res, res, stamp, is_actively_tracked);
    if (is_actively_tracked) {
        remember_observation(res, stamp);
    }
    publish_tracking_visualization(stamp, is_actively_tracked);
}

// 状態が変わった瞬間だけ、着地通知と終了理由を出力する。
void ClothTrackerCudaNode::publish_tracking_transition(const ClothTrackingResult& raw_res,
                                                       const ClothTrackingResult& res,
                                                       const rclcpp::Time& stamp,
                                                       bool is_actively_tracked) {
    if (is_actively_tracked && ballistic_tracker_.state == TrackerState::LANDED) {
        last_cloth_x_ = res.cx;
        last_cloth_y_ = res.cy;
        last_cloth_z_ = res.cz;
    }
    if (prev_state_ == TrackerState::TRACKING && ballistic_tracker_.state == TrackerState::LANDED) {
        if (pub_landing_pos_) {
            geometry_msgs::msg::PointStamped landing_msg;
            landing_msg.header.stamp = stamp;
            landing_msg.header.frame_id = camera_parent_frame_;
            landing_msg.point.x = last_cloth_x_;
            landing_msg.point.y = last_cloth_y_;
            landing_msg.point.z = last_cloth_z_;
            pub_landing_pos_->publish(landing_msg);
        }

        RCLCPP_INFO(this->get_logger(),
                    "========================================================================\n"
                    "*** CLOTH TRACKING FINISHED/LANDED at (%.2f, %.2f, %.2f)! Recorded %zu flight "
                    "points. ***\n"
                    "========================================================================",
                    last_cloth_x_, last_cloth_y_, last_cloth_z_, trajectory_points_.size());
    } else if (prev_state_ == TrackerState::TRACKING &&
               ballistic_tracker_.state == TrackerState::LOST) {
        RCLCPP_INFO(get_logger(),
                    "Tracking rejected: %s; candidate=(%.3f, %.3f, %.3f), points=%d; last=(%.3f, "
                    "%.3f, %.3f), velocity=(%.3f, %.3f, %.3f)",
                    ballistic_tracker_.last_rejection_reason.c_str(), raw_res.cx, raw_res.cy,
                    raw_res.cz, raw_res.cluster_points_count, ballistic_tracker_.x,
                    ballistic_tracker_.y, ballistic_tracker_.z, ballistic_tracker_.vx,
                    ballistic_tracker_.vy, ballistic_tracker_.vz);
        RCLCPP_INFO(this->get_logger(),
                    "========================================================================\n"
                    "*** CLOTH TRACKING CUT OFF (LOST) at (%.2f, %.2f, %.2f)! Recorded %zu flight "
                    "points. ***\n"
                    "========================================================================",
                    last_cloth_x_, last_cloth_y_, last_cloth_z_, trajectory_points_.size());
    }
    prev_state_ = ballistic_tracker_.state;
}

// 採用された位置だけを履歴に残す。欠測中の予測は履歴に入れない。
void ClothTrackerCudaNode::remember_observation(const ClothTrackingResult& res,
                                                const rclcpp::Time& stamp) {
    const double now_sec = stamp.seconds();
    RCLCPP_INFO(this->get_logger(),
                "[ClothTracker] >>> TRACKED CLOTH: points=%d, pos=(%.2f, %.2f, %.2f) <<<",
                res.cluster_points_count, res.cx, res.cy, res.cz);

    last_cloth_x_ = res.cx;
    last_cloth_y_ = res.cy;
    last_cloth_z_ = res.cz;
    has_cloth_pose_ = true;

    if (ballistic_tracker_.tracked_count <= 1) {
        trajectory_points_.clear(); // Brand new launch: clear previous trajectory line
        {
            geometry_msgs::msg::Point launch_pt;
            launch_pt.x = res.cx;
            launch_pt.y = res.cy;
            launch_pt.z = res.cz;
            trajectory_points_.emplace_back(ballistic_tracker_.launch_stamp, launch_pt);
        }
        RCLCPP_INFO(this->get_logger(),
                    "========================================================================\n"
                    "*** LAUNCH CONFIRMED at (%.2f, %.2f, %.2f) with %d pts! Tracking locked! ***\n"
                    "========================================================================",
                    res.cx, res.cy, res.cz, res.cluster_points_count);
    }

    // Publish Centroid Point (Strictly real measured point)
    geometry_msgs::msg::PointStamped pt_msg;
    pt_msg.header.stamp = stamp;
    pt_msg.header.frame_id = camera_parent_frame_;
    pt_msg.point.x = res.cx;
    pt_msg.point.y = res.cy;
    pt_msg.point.z = res.cz;
    pub_point_->publish(pt_msg);

    // Trajectory History (Strictly true measured points advancing forward)
    geometry_msgs::msg::Point p_geom;
    p_geom.x = res.cx;
    p_geom.y = res.cy;
    p_geom.z = res.cz;

    if (trajectory_points_.empty()) {
        trajectory_points_.emplace_back(now_sec, p_geom);
    } else {
        const auto& prev_pt = trajectory_points_.back().second;
        double dx = p_geom.x - prev_pt.x;
        double dy = p_geom.y - prev_pt.y;
        double dz = p_geom.z - prev_pt.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist >= min_trajectory_step_m_) {
            trajectory_points_.emplace_back(now_sec, p_geom);
        }
    }

    while (trajectory_history_sec_ > 0.0 && trajectory_points_.size() > 1 &&
           (now_sec - trajectory_points_.front().first) > trajectory_history_sec_) {
        trajectory_points_.erase(trajectory_points_.begin());
    }
    if (trajectory_points_.size() > max_trajectory_points_) {
        trajectory_points_.erase(trajectory_points_.begin());
    }
}

void ClothTrackerCudaNode::publish_tracking_visualization(const rclcpp::Time& stamp,
                                                          bool is_actively_tracked) {
    const double now_sec = stamp.seconds();
    // TFの時刻を更新するのは実測を採用したフレームだけ。
    if (has_cloth_pose_ && is_actively_tracked) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = camera_parent_frame_;
        tf_msg.child_frame_id = cloth_tf_frame_;
        tf_msg.transform.translation.x = last_cloth_x_;
        tf_msg.transform.translation.y = last_cloth_y_;
        tf_msg.transform.translation.z = last_cloth_z_;
        tf_msg.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    // 最後に採用した位置を赤い球で表示する。
    if (has_cloth_pose_) {
        pub_marker_->publish(MarkerVisualizer::create_cloth_tip_marker(
            stamp, last_cloth_x_, last_cloth_y_, last_cloth_z_, camera_parent_frame_));
    }

    // 追跡中は毎フレーム、終了後はYAMLの周期で実測軌跡を再配信する。
    bool should_publish_markers =
        is_actively_tracked ||
        (std::abs(now_sec - last_static_publish_sec_) >= static_marker_interval_sec_);

    if (should_publish_markers && !trajectory_points_.empty()) {
        last_static_publish_sec_ = now_sec;

        // 1. Measured Flight Trajectory Line Strip (Bright Gold, strictly connects true camera detections)
        pub_marker_->publish(MarkerVisualizer::create_trajectory_line(stamp, trajectory_points_,
                                                                      camera_parent_frame_));

        // 2. True Measured Points Marker (Orange Spheres, exactly where camera detected cloth)
        pub_marker_->publish(MarkerVisualizer::create_measured_points(stamp, trajectory_points_,
                                                                      camera_parent_frame_));

        // 3. Measured Path (True measured points only)
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = camera_parent_frame_;
        for (const auto& item : trajectory_points_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = stamp;
            pose.header.frame_id = camera_parent_frame_;
            pose.pose.position = item.second;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        pub_path_->publish(path_msg);
    }
}

} // namespace realsense_dust_cloth
