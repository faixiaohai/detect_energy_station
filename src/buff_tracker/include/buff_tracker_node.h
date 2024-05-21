#ifndef BUFF_TRACKER_NODE_H
#define BUFF_TRACKER_NODE_H

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <interfaces/msg/buff.hpp>

#include <buff_tracker.h>

namespace buff_auto_aim {

class BuffTrackerNode : public rclcpp::Node {
public:
    explicit BuffTrackerNode(const rclcpp::NodeOptions &options);

    void SubDetectorInfoCallback(const interfaces::msg::Buff &buff_msg);

    void SubCameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_msg);

    void DeclareParam();
    
private:
    //subscription
    rclcpp::Subscription<interfaces::msg::Buff>::SharedPtr m_buff_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_camera_info_sub;
    cv::Mat m_intrinsic_matrix;
    cv::Mat m_distortion_vector;  
    std::shared_ptr<BuffTracker> m_tracker; 
    
};



} // buff_auto_aim namespace

#endif // BUFF_TRACKER_NODE_H