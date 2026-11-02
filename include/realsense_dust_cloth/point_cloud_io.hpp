#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cuda_runtime.h>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace realsense_dust_cloth {
// Accept organized XYZ FLOAT32 clouds, including row padding. Never reinterpret unaligned bytes.
inline std::vector<float3> read_xyz_cloud(const sensor_msgs::msg::PointCloud2& msg,
                                        const std::string& expected_frame, size_t max_points) {
    if (msg.header.frame_id != expected_frame)
        throw std::invalid_argument("Point cloud frame does not match camera_parent_frame");
    const sensor_msgs::msg::PointField *xf = nullptr, *yf = nullptr, *zf = nullptr;
    for (const auto& field : msg.fields) {
        if (field.name == "x") xf = &field;
        if (field.name == "y") yf = &field;
        if (field.name == "z") zf = &field;
    }
    auto valid = [&](const auto* field) {
        return field && field->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
               field->count == 1 && size_t(field->offset) + sizeof(float) <= msg.point_step;
    };
    const size_t count = size_t(msg.width) * msg.height;
    if (!valid(xf) || !valid(yf) || !valid(zf) || msg.is_bigendian || count > max_points ||
        size_t(msg.row_step) < size_t(msg.width) * msg.point_step ||
        msg.data.size() < size_t(msg.row_step) * msg.height)
        throw std::invalid_argument("Invalid or oversized XYZ PointCloud2");
    std::vector<float3> points;
    points.reserve(count);
    for (size_t row = 0; row < msg.height; ++row) {
        for (size_t col = 0; col < msg.width; ++col) {
            const auto* ptr = msg.data.data() + row * msg.row_step + col * msg.point_step;
            float3 point;
            std::memcpy(&point.x, ptr + xf->offset, sizeof(float));
            std::memcpy(&point.y, ptr + yf->offset, sizeof(float));
            std::memcpy(&point.z, ptr + zf->offset, sizeof(float));
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
                points.push_back(point);
        }
    }
    return points;
}
} // namespace realsense_dust_cloth
