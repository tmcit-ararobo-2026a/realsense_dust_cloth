#pragma once

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "realsense_dust_cloth/cuda_kernels.cuh"

namespace realsense_dust_cloth {

class MarkerVisualizer {
public:
    /**
     * @brief Builds ROI boundary boxes, launch gate, HUD dashboard text, and floor distance markers.
     */
    static visualization_msgs::msg::MarkerArray create_roi_markers(
        const rclcpp::Time& stamp,
        const FilterConfig& filter_cfg,
        bool enable_ballistic_filter,
        double launch_min_x, double launch_max_x,
        double launch_min_y, double launch_max_y,
        double launch_min_z, double launch_max_z,
        bool is_tracking,
        double track_x, double track_y, double track_z,
        double track_vx, double track_vz, double track_speed,
        const ClothTrackingResult& raw_res,
        const CameraPoseConfig& camera_pose,
        const std::vector<std::pair<double, geometry_msgs::msg::Point>>& trajectory_points,
        const std::string& frame_id = "base_link")
    {
        visualization_msgs::msg::MarkerArray markers;

        // 1. PassThrough ROI Box (Cyan Wireframe Cube)
        visualization_msgs::msg::Marker roi_marker;
        roi_marker.header.stamp = stamp;
        roi_marker.header.frame_id = frame_id;
        roi_marker.ns = "roi_passthrough";
        roi_marker.id = 0;
        roi_marker.type = visualization_msgs::msg::Marker::CUBE;
        roi_marker.action = visualization_msgs::msg::Marker::ADD;
        roi_marker.pose.position.x = 0.5 * (filter_cfg.min_x_m + filter_cfg.max_x_m);
        roi_marker.pose.position.y = 0.5 * (filter_cfg.min_y_m + filter_cfg.max_y_m);
        roi_marker.pose.position.z = 0.5 * (filter_cfg.min_z_m + filter_cfg.max_z_m);
        roi_marker.pose.orientation.w = 1.0;
        roi_marker.scale.x = std::max(0.01f, filter_cfg.max_x_m - filter_cfg.min_x_m);
        roi_marker.scale.y = std::max(0.01f, filter_cfg.max_y_m - filter_cfg.min_y_m);
        roi_marker.scale.z = std::max(0.01f, filter_cfg.max_z_m - filter_cfg.min_z_m);
        roi_marker.color.r = 0.0f;
        roi_marker.color.g = 0.85f;
        roi_marker.color.b = 1.0f;
        roi_marker.color.a = 0.10f;
        markers.markers.push_back(roi_marker);

        // 2. Launch Gate Box (Yellow Translucent Cube)
        if (enable_ballistic_filter) {
            visualization_msgs::msg::Marker launch_marker;
            launch_marker.header.stamp = stamp;
            launch_marker.header.frame_id = frame_id;
            launch_marker.ns = "launch_gate";
            launch_marker.id = 1;
            launch_marker.type = visualization_msgs::msg::Marker::CUBE;
            launch_marker.action = visualization_msgs::msg::Marker::ADD;
            launch_marker.pose.position.x = 0.5 * (launch_min_x + launch_max_x);
            launch_marker.pose.position.y = 0.5 * (launch_min_y + launch_max_y);
            launch_marker.pose.position.z = 0.5 * (launch_min_z + launch_max_z);
            launch_marker.pose.orientation.w = 1.0;
            launch_marker.scale.x = std::max(0.01, launch_max_x - launch_min_x);
            launch_marker.scale.y = std::max(0.01, launch_max_y - launch_min_y);
            launch_marker.scale.z = std::max(0.01, launch_max_z - launch_min_z);
            launch_marker.color.r = 1.0f;
            launch_marker.color.g = 0.85f;
            launch_marker.color.b = 0.0f;
            launch_marker.color.a = 0.20f;
            markers.markers.push_back(launch_marker);
        }

        // 3. Real-time HUD Dashboard (High-visibility 3D Text positioned clearly above scene)
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.stamp = stamp;
        text_marker.header.frame_id = frame_id;
        text_marker.ns = "tracker_status_hud";
        text_marker.id = 2;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = 1.5;
        text_marker.pose.position.y = -0.9;
        text_marker.pose.position.z = 1.8;
        text_marker.scale.z = 0.16;

        char status_str[512];
        if (is_tracking) {
            text_marker.color.r = 0.2f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 0.3f;
            text_marker.color.a = 1.0f;
            snprintf(status_str, sizeof(status_str),
                     "● [TRACKING ACTIVE]\nCloth: (X: %.2fm, Y: %.2fm, Z: %.2fm)\nSpeed: %.2f m/s (Vx: %.2f, Vz: %.2f)\nPoints: %d pts",
                     track_x, track_y, track_z,
                     track_speed, track_vx, track_vz, raw_res.cluster_points_count);
        } else {
            text_marker.color.r = 1.0f;
            text_marker.color.g = 0.9f;
            text_marker.color.b = 0.2f;
            text_marker.color.a = 0.95f;
            if (raw_res.valid) {
                snprintf(status_str, sizeof(status_str),
                         "○ [IDLE: Candidate Detected]\nPos: (X: %.2fm, Y: %.2fm, Z: %.2fm)\nCluster: %d pts\n(Waiting for launch velocity)",
                         raw_res.cx, raw_res.cy, raw_res.cz, raw_res.cluster_points_count);
            } else {
                snprintf(status_str, sizeof(status_str),
                         "○ [IDLE: Waiting for Throw]\nLaunch Area: X: %.2f~%.2fm, Z: %.2f~%.2fm\nCamera: Pitch %.1f deg, Height %.2fm",
                         launch_min_x, launch_max_x,
                         launch_min_z, launch_max_z,
                         camera_pose.pitch_deg, camera_pose.height_offset_m);
            }
        }
        text_marker.text = status_str;
        markers.markers.push_back(text_marker);

        // 4. Floor Distance Markers (1.0m, 2.0m, 3.0m, 3.5m)
        for (double dist = 1.0; dist <= 3.5; dist += 1.0) {
            visualization_msgs::msg::Marker dist_marker;
            dist_marker.header.stamp = stamp;
            dist_marker.header.frame_id = frame_id;
            dist_marker.ns = "floor_distance_marks";
            dist_marker.id = static_cast<int>(dist * 10);
            dist_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            dist_marker.action = visualization_msgs::msg::Marker::ADD;
            dist_marker.pose.position.x = dist;
            dist_marker.pose.position.y = 0.65;
            dist_marker.pose.position.z = 0.05;
            dist_marker.scale.z = 0.12;
            dist_marker.color.r = 0.7f;
            dist_marker.color.g = 0.8f;
            dist_marker.color.b = 1.0f;
            dist_marker.color.a = 0.85f;
            char dist_buf[32];
            snprintf(dist_buf, sizeof(dist_buf), "%.1fm", dist);
            dist_marker.text = dist_buf;
            markers.markers.push_back(dist_marker);
        }

        // 5. Landing point text marker
        if (!trajectory_points.empty()) {
            const auto& last_pt = trajectory_points.back().second;
            visualization_msgs::msg::Marker land_marker;
            land_marker.header.stamp = stamp;
            land_marker.header.frame_id = frame_id;
            land_marker.ns = "landing_point_text";
            land_marker.id = 99;
            land_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            land_marker.action = visualization_msgs::msg::Marker::ADD;
            land_marker.pose.position.x = last_pt.x;
            land_marker.pose.position.y = last_pt.y;
            land_marker.pose.position.z = last_pt.z + 0.18;
            land_marker.scale.z = 0.13;
            land_marker.color.r = 1.0f;
            land_marker.color.g = 0.3f;
            land_marker.color.b = 0.2f;
            land_marker.color.a = 1.0f;
            char land_buf[64];
            snprintf(land_buf, sizeof(land_buf), "🎯 Landing: %.2fm", last_pt.x);
            land_marker.text = land_buf;
            markers.markers.push_back(land_marker);
        }

        return markers;
    }

    /**
     * @brief Creates the current cloth tip sphere marker.
     */
    static visualization_msgs::msg::Marker create_cloth_tip_marker(
        const rclcpp::Time& stamp, double x, double y, double z,
        const std::string& frame_id = "base_link")
    {
        visualization_msgs::msg::Marker ball_marker;
        ball_marker.header.stamp = stamp;
        ball_marker.header.frame_id = frame_id;
        ball_marker.ns = "cloth_tip_ball";
        ball_marker.id = 1;
        ball_marker.type = visualization_msgs::msg::Marker::SPHERE;
        ball_marker.action = visualization_msgs::msg::Marker::ADD;
        ball_marker.pose.position.x = x;
        ball_marker.pose.position.y = y;
        ball_marker.pose.position.z = z;
        ball_marker.pose.orientation.w = 1.0;
        ball_marker.scale.x = 0.12;
        ball_marker.scale.y = 0.12;
        ball_marker.scale.z = 0.12;
        ball_marker.color.r = 1.0f;
        ball_marker.color.g = 0.15f;
        ball_marker.color.b = 0.15f;
        ball_marker.color.a = 0.95f;
        return ball_marker;
    }

    /**
     * @brief Creates the measured flight trajectory line strip.
     */
    static visualization_msgs::msg::Marker create_trajectory_line(
        const rclcpp::Time& stamp,
        const std::vector<std::pair<double, geometry_msgs::msg::Point>>& points,
        const std::string& frame_id = "base_link")
    {
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.stamp = stamp;
        line_marker.header.frame_id = frame_id;
        line_marker.ns = "cloth_cuda_trajectory";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.040; // 4cm thickness
        line_marker.color.r = 1.0f;
        line_marker.color.g = 0.85f;
        line_marker.color.b = 0.0f;
        line_marker.color.a = 0.95f;
        for (const auto& item : points) {
            line_marker.points.push_back(item.second);
        }
        return line_marker;
    }

    /**
     * @brief Creates the measured points list (orange spheres).
     */
    static visualization_msgs::msg::Marker create_measured_points(
        const rclcpp::Time& stamp,
        const std::vector<std::pair<double, geometry_msgs::msg::Point>>& points,
        const std::string& frame_id = "base_link")
    {
        visualization_msgs::msg::Marker raw_points_marker;
        raw_points_marker.header.stamp = stamp;
        raw_points_marker.header.frame_id = frame_id;
        raw_points_marker.ns = "cloth_measured_points";
        raw_points_marker.id = 2;
        raw_points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        raw_points_marker.action = visualization_msgs::msg::Marker::ADD;
        raw_points_marker.scale.x = 0.045;
        raw_points_marker.scale.y = 0.045;
        raw_points_marker.scale.z = 0.045;
        raw_points_marker.color.r = 1.0f;
        raw_points_marker.color.g = 0.40f;
        raw_points_marker.color.b = 0.0f;
        raw_points_marker.color.a = 0.95f;
        for (const auto& item : points) {
            raw_points_marker.points.push_back(item.second);
        }
        return raw_points_marker;
    }

    /**
     * @brief Helper to publish float3 point clouds as sensor_msgs::msg::PointCloud2.
     */
    static void publish_point_cloud(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
        const std::vector<float3>& pts,
        const rclcpp::Time& stamp,
        const std::string& frame_id)
    {
        if (!pub) return;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = stamp;
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.height = 1;
        cloud_msg.width = static_cast<uint32_t>(pts.size());
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(pts.size());

        if (!pts.empty()) {
            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

            for (const auto& p : pts) {
                *iter_x = p.x;
                *iter_y = p.y;
                *iter_z = p.z;
                ++iter_x;
                ++iter_y;
                ++iter_z;
            }
        }

        pub->publish(cloud_msg);
    }
};

} // namespace realsense_dust_cloth

