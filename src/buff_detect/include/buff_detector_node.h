#ifndef BUFF_DETECTOR_NODE_H
#define BUFF_DETECTOR_NODE_H

#include <iostream>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>


#include <opencv2/opencv.hpp>




#include <buff_pnp_solver.h>
#include <interfaces/msg/buff.hpp>
#include <inference.h>



namespace buff_auto_aim {
class BuffDetectorNode: public rclcpp::Node {
public:
    explicit BuffDetectorNode(const rclcpp::NodeOptions& options);
private:
    // Info
    sensor_msgs::msg::CameraInfo m_cam_info;
    interfaces::msg::Buff m_buff;
    cv::Point2f m_cam_center;
    std::shared_ptr<BuffPnP> m_buff_pnp;
    cv::Mat m_img;
    std::shared_ptr<OpenvinoInference> m_inference;
    Match m_match;
    ImageInformation m_info;

    // Subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_img_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_cam_info_sub;

    // Publisher
    rclcpp::Publisher<interfaces::msg::Buff>::SharedPtr m_buffs_pub;

    void DeclareParams();

    void SubCamInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info);

    void SubImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
    
    void OrientationFromRvec(const cv::Mat& rvec, geometry_msgs::msg::Quaternion& q);


};
}
#endif // BUFF_DETECTOR_NODE_H