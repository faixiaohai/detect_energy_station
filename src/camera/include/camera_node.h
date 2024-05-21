#ifndef CAMERA_NODE_H
#define CAMERA_NODE_H

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/camera_publisher.h>
#include <image_transport/camera_publisher.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <camera_start.h>

class CameraCaptureNode : public rclcpp::Node {
public:
    CameraCaptureNode(const rclcpp::NodeOptions& options);

    void HandleCameraEnableRequest(
    const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>& response);

    void InitCamera();

    void PublishData();

    
    
private:
    // fun
    image_transport::CameraPublisher m_camera_pub;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_service;
    std::shared_ptr<camera_info_manager::CameraInfoManager> m_camera_info_manager;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_camera_info_pub;
    // parm
    sensor_msgs::msg::CameraInfo m_camera_info;
    sensor_msgs::msg::Image m_image_msg;
    Camera m_camera;
    std::string m_camera_name;
    std::string m_camera_info_url;
    camera_info_manager::CameraInfo m_camera_info_msg;
    MV_IMAGE_BASIC_INFO m_camera_hik_info;
    bool m_enable;
    std::string m_mess;
    std::thread m_thread;

    
};

#endif // CAMERA_CAPTURE_NODE_H







