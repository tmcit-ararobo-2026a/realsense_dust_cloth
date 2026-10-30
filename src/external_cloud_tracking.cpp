#include "realsense_dust_cloth/cloth_tracker_cuda_node.hpp"
#include <chrono>
#include <cmath>
#include <cstring>

namespace realsense_dust_cloth {
namespace {
double wall_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
void ClothTrackerCudaNode::setup_external_cloud() {
    pub_filter_prediction_=create_publisher<visualization_msgs::msg::Marker>(
        get_parameter("external_prediction_topic").as_string(),10);
    pub_cropped_cloud_=create_publisher<sensor_msgs::msg::PointCloud2>(cropped_cloud_topic_,rclcpp::SensorDataQoS());
    sub_dynamic_cloud_=create_subscription<sensor_msgs::msg::PointCloud2>(dynamic_cloud_topic_,
        rclcpp::SensorDataQoS(),std::bind(&ClothTrackerCudaNode::dynamic_cloud_callback,this,std::placeholders::_1));
    sub_filter_status_=create_subscription<std_msgs::msg::String>(
        get_parameter("external_filter_status_topic").as_string(),rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::SharedPtr msg) {
            std::lock_guard<std::recursive_mutex> guard(processing_mutex_);
            if (msg->data!="READY") publish_background_state("EXTERNAL_"+msg->data,false);
        });
    external_watchdog_=create_wall_timer(std::chrono::duration<double>(external_poll_sec_),[this] {
        std::lock_guard<std::recursive_mutex> guard(processing_mutex_);
        check_robot_motion_state(now());
        if (enable_cmd_vel_watch_ && is_robot_moving_) {
            publish_background_state("MOVING_OR_SETTLING",false); return;
        }
        if (last_external_wall_sec_<0 || wall_seconds()-last_external_wall_sec_>external_result_timeout_sec_) {
            if (ballistic_tracker_.state==TrackerState::TRACKING) {
                publish_tracking_result(ClothTrackingResult{},now(),{},{});
            }
            publish_background_state("EXTERNAL_RESULT_TIMEOUT",false);
        }
    });
}

void ClothTrackerCudaNode::dynamic_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::recursive_mutex> guard(processing_mutex_);
    const rclcpp::Time stamp(msg->header.stamp,get_clock()->get_clock_type());
    if (msg->header.frame_id!=camera_parent_frame_) {
        publish_background_state("EXTERNAL_FRAME_INVALID",false); return;
    }
    if (std::abs((now()-stamp).seconds())>map_max_image_age_sec_ ||
        stamp.seconds()<=last_external_stamp_sec_) {
        publish_background_state("EXTERNAL_TIME_INVALID",false); return;
    }
    check_robot_motion_state(stamp);
    if (enable_cmd_vel_watch_ && is_robot_moving_) {
        publish_background_state("MOVING_OR_SETTLING",false); return;
    }
    const sensor_msgs::msg::PointField *xf=nullptr,*yf=nullptr,*zf=nullptr;
    for (const auto& f : msg->fields) {
        if (f.name=="x") xf=&f;
        if (f.name=="y") yf=&f;
        if (f.name=="z") zf=&f;
    }
    auto valid=[&](const sensor_msgs::msg::PointField* f) {
        return f && f->datatype==sensor_msgs::msg::PointField::FLOAT32 && f->count==1 &&
            size_t(f->offset)+sizeof(float)<=msg->point_step;
    };
    if (!valid(xf)||!valid(yf)||!valid(zf)||msg->is_bigendian||
        size_t(msg->row_step)<size_t(msg->width)*msg->point_step||
        msg->data.size()<size_t(msg->row_step)*msg->height) {
        publish_background_state("EXTERNAL_LAYOUT_INVALID",false); return;
    }
    std::vector<float3> points;
    points.reserve(size_t(msg->width)*msg->height);
    for (size_t row=0;row<msg->height;++row) for (size_t col=0;col<msg->width;++col) {
        const auto* ptr=msg->data.data()+row*msg->row_step+col*msg->point_step;
        float3 p;
        std::memcpy(&p.x,ptr+xf->offset,4); std::memcpy(&p.y,ptr+yf->offset,4); std::memcpy(&p.z,ptr+zf->offset,4);
        points.push_back(p);
    }
    float px=0,py=0,pz=0;
    const bool predicted=ballistic_tracker_.get_prediction(stamp.seconds(),px,py,pz);
    std::vector<float3> cluster,filtered;
    try {
        const auto result=process_point_cloud_cuda(cuda_ctx_,points,filter_cfg_,
            publish_debug_cloud_?&cluster:nullptr,publish_debug_cloud_?&filtered:nullptr,
            px,py,pz,predicted);
        if (cudaGetLastError()!=cudaSuccess) throw std::runtime_error("CUDA point tracking failed");
        last_external_wall_sec_=wall_seconds(); last_external_stamp_sec_=stamp.seconds();
        publish_tracking_result(result,stamp,cluster,filtered);
    } catch (const std::exception& error) {
        publish_background_state("EXTERNAL_TRACKING_ERROR",false);
        RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),1000,"Dynamic tracking: %s",error.what());
    }
}
} // namespace realsense_dust_cloth
