#include "realsense_dust_cloth/point_cloud_io.hpp"
#include <cassert>
using namespace realsense_dust_cloth;
int main() {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "base_link";
    cloud.width = 1;
    cloud.height = 2;
    cloud.point_step = 16;
    cloud.row_step = 24;
    cloud.data.resize(48);
    for (int i = 0; i < 3; ++i) {
        sensor_msgs::msg::PointField field;
        field.name = std::string(1, "xyz"[i]);
        field.offset = i * sizeof(float);
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        cloud.fields.push_back(field);
    }
    float first[3]{1, 2, 3}, second[3]{4, 5, 6};
    std::memcpy(cloud.data.data(), first, sizeof(first));
    std::memcpy(cloud.data.data() + cloud.row_step, second, sizeof(second));
    const auto points = read_xyz_cloud(cloud, "base_link", 10);
    assert(points.size() == 2 && points[0].x == 1 && points[1].z == 6);
    bool rejected = false;
    cloud.data.pop_back();
    try {
        read_xyz_cloud(cloud, "base_link", 10);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);
}
